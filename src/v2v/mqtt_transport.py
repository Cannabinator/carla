"""
MQTT Transport Layer for V2V Communication

Provides a real network transport for BSM messages using MQTT pub/sub.
This layer sits between BSM creation and consumption, enabling:
- Real message serialization/deserialization over the wire
- Natural insertion point for encryption (TLS and/or payload-level)
- Measurable network latency and overhead
- Topic-based message routing per vehicle

Architecture:
    Vehicle creates BSM → serialize → [encrypt] → MQTT publish
    MQTT subscribe → [decrypt] → deserialize → BSM consumed by neighbors

Topic Structure:
    v2v/bsm/{vehicle_id}       - BSM broadcasts per vehicle
    v2v/emergency/{vehicle_id}  - Emergency/priority messages
    v2v/cooperative/{vehicle_id} - Sensor data sharing

The transport is OPTIONAL. When disabled, V2VNetworkEnhanced falls back
to its original in-process dict-based message passing.
"""

import json
import time
import logging
import threading
from dataclasses import asdict
from typing import Dict, Optional, Callable, List, Any

from .messages import BSMCore, BrakingStatus, VehicleType

logger = logging.getLogger(__name__)

# Lazy import paho-mqtt so the module can be imported without the dependency
_paho_mqtt = None


def _get_paho():
    """Lazy import of paho.mqtt.client to avoid hard dependency."""
    global _paho_mqtt
    if _paho_mqtt is None:
        try:
            import paho.mqtt.client as mqtt
            _paho_mqtt = mqtt
        except ImportError:
            raise ImportError(
                "paho-mqtt is required for MQTT transport. "
                "Install it with: pip install paho-mqtt>=2.0.0"
            )
    return _paho_mqtt


class BSMSerializer:
    """
    Serialize/deserialize BSMCore messages for MQTT transmission.
    
    Uses JSON as the wire format for readability and debugging.
    This is the natural boundary for adding encryption:
    serialize → encrypt → publish  /  receive → decrypt → deserialize
    """
    
    @staticmethod
    def serialize(bsm: BSMCore) -> bytes:
        """
        Serialize BSMCore to bytes for MQTT payload.
        
        Args:
            bsm: BSMCore message to serialize
            
        Returns:
            JSON-encoded bytes
        """
        data = asdict(bsm)
        # Convert enum values to ints for JSON compatibility
        data['vehicle_type'] = int(data['vehicle_type'])
        data['brake_status'] = int(data['brake_status'])
        return json.dumps(data, separators=(',', ':')).encode('utf-8')
    
    @staticmethod
    def deserialize(payload: bytes) -> BSMCore:
        """
        Deserialize bytes from MQTT payload to BSMCore.
        
        Args:
            payload: JSON-encoded bytes
            
        Returns:
            BSMCore instance
        """
        data = json.loads(payload.decode('utf-8'))
        # Convert int values back to enums
        data['vehicle_type'] = VehicleType(data['vehicle_type'])
        data['brake_status'] = BrakingStatus(data['brake_status'])
        return BSMCore(**data)


class MQTTTransport:
    """
    MQTT-based transport layer for V2V BSM messages.
    
    Each vehicle publishes its BSM to topic 'v2v/bsm/{vehicle_id}'.
    All vehicles subscribe to 'v2v/bsm/+' (wildcard) to receive
    BSMs from every other vehicle. Range filtering is still done
    at the application layer (V2VNetworkEnhanced._discover_neighbors).
    
    This class is designed to be used as an optional component of
    V2VNetworkEnhanced. When enabled, messages flow through MQTT
    instead of being stored directly in Python dicts.
    
    Encryption Integration Points:
        1. TLS: Set tls_enabled=True and provide cert paths in MQTTConfig
        2. Payload encryption: Subclass BSMSerializer and override
           serialize/deserialize to add encrypt/decrypt steps
        3. Per-message: Use on_pre_publish / on_post_receive hooks
    """
    
    # Topic patterns
    TOPIC_BSM = "v2v/bsm/{vehicle_id}"
    TOPIC_EMERGENCY = "v2v/emergency/{vehicle_id}"
    TOPIC_COOPERATIVE = "v2v/cooperative/{vehicle_id}"
    TOPIC_BSM_WILDCARD = "v2v/bsm/+"
    
    def __init__(self,
                 broker_host: str = "localhost",
                 broker_port: int = 1883,
                 client_id: str = "v2v_network",
                 qos: int = 1,
                 keepalive: int = 60,
                 tls_enabled: bool = False,
                 tls_ca_certs: Optional[str] = None,
                 tls_certfile: Optional[str] = None,
                 tls_keyfile: Optional[str] = None,
                 serializer: Optional[BSMSerializer] = None,
                 on_pre_publish: Optional[Callable[[int, bytes], bytes]] = None,
                 on_post_receive: Optional[Callable[[int, bytes], bytes]] = None):
        """
        Initialize MQTT transport.
        
        Args:
            broker_host: MQTT broker hostname/IP
            broker_port: MQTT broker port (1883 plain, 8883 TLS)
            client_id: MQTT client identifier
            qos: MQTT QoS level (0=at most once, 1=at least once, 2=exactly once)
            keepalive: MQTT keepalive interval in seconds
            tls_enabled: Enable TLS/SSL encryption at transport level
            tls_ca_certs: Path to CA certificate file (for TLS)
            tls_certfile: Path to client certificate file (for mTLS)
            tls_keyfile: Path to client private key file (for mTLS)
            serializer: Custom BSMSerializer (override for encryption)
            on_pre_publish: Hook called before publishing (payload transform)
                           Signature: (vehicle_id, serialized_bytes) -> transformed_bytes
            on_post_receive: Hook called after receiving (payload transform)
                            Signature: (vehicle_id, raw_bytes) -> transformed_bytes
        """
        mqtt = _get_paho()
        
        self.broker_host = broker_host
        self.broker_port = broker_port
        self.qos = qos
        self.keepalive = keepalive
        self.serializer = serializer or BSMSerializer()
        
        # Encryption hooks for research
        self.on_pre_publish = on_pre_publish
        self.on_post_receive = on_post_receive
        
        # MQTT client setup
        self._client = mqtt.Client(
            callback_api_version=mqtt.CallbackAPIVersion.VERSION2,
            client_id=client_id,
            protocol=mqtt.MQTTv5
        )
        self._client.on_connect = self._on_connect
        self._client.on_message = self._on_message
        self._client.on_disconnect = self._on_disconnect
        
        # TLS configuration
        self.tls_enabled = tls_enabled
        if tls_enabled:
            self._client.tls_set(
                ca_certs=tls_ca_certs,
                certfile=tls_certfile,
                keyfile=tls_keyfile
            )
        
        # Received BSM messages from other vehicles (thread-safe)
        self._received_bsm: Dict[int, BSMCore] = {}
        self._lock = threading.Lock()
        
        # Connection state
        self._connected = False
        self._subscribed = False
        
        # Statistics for research metrics
        self.stats = {
            'messages_published': 0,
            'messages_received': 0,
            'bytes_published': 0,
            'bytes_received': 0,
            'publish_errors': 0,
            'deserialize_errors': 0,
            'avg_publish_latency_ms': 0.0,
            'avg_receive_latency_ms': 0.0,
        }
        self._publish_latencies: List[float] = []
        self._receive_latencies: List[float] = []
        
        logger.info(f"MQTT transport created: {broker_host}:{broker_port} "
                    f"(QoS={qos}, TLS={tls_enabled})")
    
    def connect(self) -> bool:
        """
        Connect to MQTT broker.
        
        Returns:
            True if connection was initiated successfully
        """
        try:
            logger.info(f"Connecting to MQTT broker at {self.broker_host}:{self.broker_port}...")
            self._client.connect(self.broker_host, self.broker_port, self.keepalive)
            self._client.loop_start()
            
            # Wait for connection (up to 5 seconds)
            timeout = 5.0
            start = time.time()
            while not self._connected and (time.time() - start) < timeout:
                time.sleep(0.1)
            
            if self._connected:
                logger.info("MQTT transport connected successfully")
            else:
                logger.warning("MQTT connection timed out after 5s")
            
            return self._connected
        except Exception as e:
            logger.error(f"MQTT connection failed: {e}")
            return False
    
    def disconnect(self):
        """Disconnect from MQTT broker."""
        try:
            self._client.loop_stop()
            self._client.disconnect()
            self._connected = False
            logger.info("MQTT transport disconnected")
        except Exception as e:
            logger.error(f"MQTT disconnect error: {e}")
    
    def publish_bsm(self, vehicle_id: int, bsm: BSMCore) -> bool:
        """
        Publish a BSM message for a vehicle.
        
        Args:
            vehicle_id: Vehicle identifier
            bsm: BSMCore message to publish
            
        Returns:
            True if published successfully
        """
        if not self._connected:
            logger.debug("MQTT not connected, skipping publish")
            return False
        
        try:
            start_time = time.perf_counter()
            
            # Serialize
            payload = self.serializer.serialize(bsm)
            
            # Pre-publish hook (encryption insertion point)
            if self.on_pre_publish:
                payload = self.on_pre_publish(vehicle_id, payload)
            
            # Publish to vehicle-specific topic
            topic = self.TOPIC_BSM.format(vehicle_id=vehicle_id)
            result = self._client.publish(topic, payload, qos=self.qos)
            
            # Track latency
            elapsed_ms = (time.perf_counter() - start_time) * 1000
            self._publish_latencies.append(elapsed_ms)
            
            # Update stats
            self.stats['messages_published'] += 1
            self.stats['bytes_published'] += len(payload)
            
            # Keep rolling average (last 100 samples)
            if len(self._publish_latencies) > 100:
                self._publish_latencies = self._publish_latencies[-100:]
            self.stats['avg_publish_latency_ms'] = (
                sum(self._publish_latencies) / len(self._publish_latencies)
            )
            
            return result.rc == 0
            
        except Exception as e:
            self.stats['publish_errors'] += 1
            logger.error(f"MQTT publish error for vehicle {vehicle_id}: {e}")
            return False
    
    def get_received_bsm(self) -> Dict[int, BSMCore]:
        """
        Get all received BSM messages (thread-safe copy).
        
        Returns:
            Dict mapping vehicle_id to latest BSMCore received via MQTT
        """
        with self._lock:
            return self._received_bsm.copy()
    
    def get_received_bsm_for(self, vehicle_id: int) -> Optional[BSMCore]:
        """
        Get received BSM for a specific vehicle.
        
        Args:
            vehicle_id: Vehicle identifier
            
        Returns:
            BSMCore or None if no message received from that vehicle
        """
        with self._lock:
            return self._received_bsm.get(vehicle_id)
    
    def get_stats(self) -> dict:
        """Get MQTT transport statistics for research analysis."""
        return self.stats.copy()
    
    @property
    def is_connected(self) -> bool:
        """Check if MQTT transport is connected."""
        return self._connected
    
    # --- MQTT Callbacks ---
    
    def _on_connect(self, client, userdata, flags, reason_code, properties=None):
        """Called when connected to MQTT broker."""
        mqtt = _get_paho()
        if reason_code == 0 or reason_code == mqtt.CONNACK_ACCEPTED:
            self._connected = True
            # Subscribe to all BSM topics
            client.subscribe(self.TOPIC_BSM_WILDCARD, qos=self.qos)
            self._subscribed = True
            logger.info(f"MQTT connected, subscribed to {self.TOPIC_BSM_WILDCARD}")
        else:
            self._connected = False
            logger.error(f"MQTT connection refused: {reason_code}")
    
    def _on_message(self, client, userdata, msg):
        """Called when a BSM message is received from another vehicle."""
        try:
            start_time = time.perf_counter()
            
            # Extract vehicle_id from topic: v2v/bsm/{vehicle_id}
            topic_parts = msg.topic.split('/')
            if len(topic_parts) != 3:
                logger.warning(f"Unexpected topic format: {msg.topic}")
                return
            
            vehicle_id = int(topic_parts[2])
            payload = msg.payload
            
            # Post-receive hook (decryption insertion point)
            if self.on_post_receive:
                payload = self.on_post_receive(vehicle_id, payload)
            
            # Deserialize
            bsm = self.serializer.deserialize(payload)
            
            # Store received BSM (thread-safe)
            with self._lock:
                self._received_bsm[vehicle_id] = bsm
            
            # Track latency
            elapsed_ms = (time.perf_counter() - start_time) * 1000
            self._receive_latencies.append(elapsed_ms)
            
            # Update stats
            self.stats['messages_received'] += 1
            self.stats['bytes_received'] += len(msg.payload)
            
            if len(self._receive_latencies) > 100:
                self._receive_latencies = self._receive_latencies[-100:]
            self.stats['avg_receive_latency_ms'] = (
                sum(self._receive_latencies) / len(self._receive_latencies)
            )
            
        except (json.JSONDecodeError, ValueError, TypeError) as e:
            self.stats['deserialize_errors'] += 1
            logger.error(f"MQTT deserialize error on {msg.topic}: {e}")
        except Exception as e:
            logger.error(f"MQTT message handling error: {e}")
    
    def _on_disconnect(self, client, userdata, flags, reason_code, properties=None):
        """Called when disconnected from MQTT broker."""
        self._connected = False
        self._subscribed = False
        if reason_code != 0:
            logger.warning(f"MQTT unexpected disconnect (rc={reason_code}), will auto-reconnect")
        else:
            logger.info("MQTT disconnected cleanly")
