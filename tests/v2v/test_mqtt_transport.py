#!/usr/bin/env python3
"""
MQTT Transport Layer Tests
Tests serialization, transport logic, and integration with V2VNetworkEnhanced.
All tests run without an actual MQTT broker (fully mocked).
"""

import unittest
import sys
import json
import time
from unittest.mock import Mock, MagicMock, patch, PropertyMock

sys.path.insert(0, '/home/workstation/carla')

from src.v2v.messages import BSMCore, VehicleType, BrakingStatus
from src.v2v.mqtt_transport import BSMSerializer, MQTTTransport


class TestBSMSerializer(unittest.TestCase):
    """Test BSM serialization/deserialization for MQTT payloads."""
    
    def _make_bsm(self, vehicle_id=1, speed=15.0, heading=90.0):
        """Create a test BSMCore."""
        return BSMCore(
            timestamp=1000.0,
            msg_count=42,
            vehicle_id=vehicle_id,
            vehicle_type=VehicleType.PASSENGER_CAR,
            latitude=100.5,
            longitude=-50.3,
            elevation=1.2,
            position_accuracy=0.5,
            speed=speed,
            heading=heading,
            steering_angle=5.0,
            longitudinal_accel=1.5,
            lateral_accel=0.3,
            vertical_accel=0.0,
            yaw_rate=2.0,
            vehicle_length=4.5,
            vehicle_width=1.8,
            vehicle_height=1.5,
            brake_status=BrakingStatus.OFF,
            brake_pressure=0.0,
            transmission_state="forward",
            throttle_confidence=100.0,
            brake_confidence=100.0,
            steering_confidence=100.0
        )
    
    def test_serialize_produces_bytes(self):
        """Serialization should produce non-empty bytes."""
        bsm = self._make_bsm()
        payload = BSMSerializer.serialize(bsm)
        
        self.assertIsInstance(payload, bytes)
        self.assertGreater(len(payload), 0)
    
    def test_serialize_is_valid_json(self):
        """Serialized payload should be valid JSON."""
        bsm = self._make_bsm()
        payload = BSMSerializer.serialize(bsm)
        
        data = json.loads(payload.decode('utf-8'))
        self.assertIsInstance(data, dict)
        self.assertIn('vehicle_id', data)
        self.assertIn('speed', data)
    
    def test_roundtrip_preserves_data(self):
        """Serialize then deserialize should produce identical BSM."""
        original = self._make_bsm(vehicle_id=7, speed=25.5, heading=180.0)
        
        payload = BSMSerializer.serialize(original)
        restored = BSMSerializer.deserialize(payload)
        
        self.assertEqual(restored.vehicle_id, original.vehicle_id)
        self.assertEqual(restored.speed, original.speed)
        self.assertEqual(restored.heading, original.heading)
        self.assertEqual(restored.latitude, original.latitude)
        self.assertEqual(restored.longitude, original.longitude)
        self.assertEqual(restored.elevation, original.elevation)
        self.assertEqual(restored.msg_count, original.msg_count)
        self.assertEqual(restored.brake_status, original.brake_status)
        self.assertEqual(restored.vehicle_type, original.vehicle_type)
        self.assertEqual(restored.steering_angle, original.steering_angle)
        self.assertEqual(restored.longitudinal_accel, original.longitudinal_accel)
        self.assertEqual(restored.transmission_state, original.transmission_state)
    
    def test_enum_roundtrip(self):
        """Enum values should survive serialization roundtrip."""
        bsm = self._make_bsm()
        bsm.vehicle_type = VehicleType.TRUCK
        bsm.brake_status = BrakingStatus.ENGAGED
        
        payload = BSMSerializer.serialize(bsm)
        restored = BSMSerializer.deserialize(payload)
        
        self.assertEqual(restored.vehicle_type, VehicleType.TRUCK)
        self.assertEqual(restored.brake_status, BrakingStatus.ENGAGED)
    
    def test_deserialize_invalid_json_raises(self):
        """Deserializing invalid JSON should raise."""
        with self.assertRaises((json.JSONDecodeError, Exception)):
            BSMSerializer.deserialize(b'not valid json{{{')
    
    def test_multiple_vehicles_distinct_payloads(self):
        """Different vehicle BSMs should produce different serialized payloads."""
        bsm1 = self._make_bsm(vehicle_id=1, speed=10.0)
        bsm2 = self._make_bsm(vehicle_id=2, speed=20.0)
        
        p1 = BSMSerializer.serialize(bsm1)
        p2 = BSMSerializer.serialize(bsm2)
        
        self.assertNotEqual(p1, p2)


class TestMQTTTransportUnit(unittest.TestCase):
    """
    Test MQTTTransport logic without a real broker.
    Uses mock paho.mqtt.client to test publish/receive flows.
    """
    
    @patch('src.v2v.mqtt_transport._get_paho')
    def test_publish_bsm_calls_client_publish(self, mock_get_paho):
        """Publishing BSM should call MQTT client.publish with correct topic."""
        # Setup mock
        mock_mqtt = MagicMock()
        mock_client_class = MagicMock()
        mock_client = MagicMock()
        mock_client_class.return_value = mock_client
        mock_mqtt.Client = mock_client_class
        mock_mqtt.CallbackAPIVersion.VERSION2 = 2
        mock_mqtt.MQTTv5 = 5
        mock_mqtt.CONNACK_ACCEPTED = 0
        mock_get_paho.return_value = mock_mqtt
        
        # Mock publish result
        publish_result = MagicMock()
        publish_result.rc = 0
        mock_client.publish.return_value = publish_result
        
        transport = MQTTTransport(broker_host='localhost', broker_port=1883)
        transport._connected = True  # Simulate connected state
        
        bsm = BSMCore(
            timestamp=1000.0, msg_count=1, vehicle_id=5,
            speed=15.0, heading=90.0
        )
        
        result = transport.publish_bsm(5, bsm)
        
        self.assertTrue(result)
        mock_client.publish.assert_called_once()
        
        # Verify topic matches pattern
        call_args = mock_client.publish.call_args
        self.assertEqual(call_args[0][0], 'v2v/bsm/5')
        
        # Verify stats updated
        self.assertEqual(transport.stats['messages_published'], 1)
        self.assertGreater(transport.stats['bytes_published'], 0)
    
    @patch('src.v2v.mqtt_transport._get_paho')
    def test_publish_when_disconnected_returns_false(self, mock_get_paho):
        """Publishing when not connected should return False without error."""
        mock_mqtt = MagicMock()
        mock_client_class = MagicMock()
        mock_mqtt.Client = mock_client_class
        mock_mqtt.CallbackAPIVersion.VERSION2 = 2
        mock_mqtt.MQTTv5 = 5
        mock_get_paho.return_value = mock_mqtt
        
        transport = MQTTTransport()
        transport._connected = False
        
        bsm = BSMCore(timestamp=1.0, msg_count=0, vehicle_id=1)
        result = transport.publish_bsm(1, bsm)
        
        self.assertFalse(result)
    
    @patch('src.v2v.mqtt_transport._get_paho')
    def test_on_message_stores_received_bsm(self, mock_get_paho):
        """Receiving an MQTT message should store the deserialized BSM."""
        mock_mqtt = MagicMock()
        mock_client_class = MagicMock()
        mock_mqtt.Client = mock_client_class
        mock_mqtt.CallbackAPIVersion.VERSION2 = 2
        mock_mqtt.MQTTv5 = 5
        mock_get_paho.return_value = mock_mqtt
        
        transport = MQTTTransport()
        
        # Simulate incoming message
        bsm = BSMCore(
            timestamp=1000.0, msg_count=10, vehicle_id=3,
            speed=20.0, heading=45.0, latitude=100.0, longitude=50.0
        )
        payload = BSMSerializer.serialize(bsm)
        
        mock_msg = MagicMock()
        mock_msg.topic = 'v2v/bsm/3'
        mock_msg.payload = payload
        
        transport._on_message(None, None, mock_msg)
        
        # Verify stored
        received = transport.get_received_bsm()
        self.assertIn(3, received)
        self.assertEqual(received[3].vehicle_id, 3)
        self.assertEqual(received[3].speed, 20.0)
        self.assertEqual(transport.stats['messages_received'], 1)
    
    @patch('src.v2v.mqtt_transport._get_paho')
    def test_on_message_invalid_payload_increments_error(self, mock_get_paho):
        """Invalid message payload should increment error count, not crash."""
        mock_mqtt = MagicMock()
        mock_client_class = MagicMock()
        mock_mqtt.Client = mock_client_class
        mock_mqtt.CallbackAPIVersion.VERSION2 = 2
        mock_mqtt.MQTTv5 = 5
        mock_get_paho.return_value = mock_mqtt
        
        transport = MQTTTransport()
        
        mock_msg = MagicMock()
        mock_msg.topic = 'v2v/bsm/1'
        mock_msg.payload = b'invalid json data'
        
        # Should not raise
        transport._on_message(None, None, mock_msg)
        
        self.assertEqual(transport.stats['deserialize_errors'], 1)
        self.assertEqual(len(transport.get_received_bsm()), 0)
    
    @patch('src.v2v.mqtt_transport._get_paho')
    def test_pre_publish_hook_transforms_payload(self, mock_get_paho):
        """Pre-publish hook should transform the payload before publishing."""
        mock_mqtt = MagicMock()
        mock_client_class = MagicMock()
        mock_client = MagicMock()
        mock_client_class.return_value = mock_client
        mock_mqtt.Client = mock_client_class
        mock_mqtt.CallbackAPIVersion.VERSION2 = 2
        mock_mqtt.MQTTv5 = 5
        mock_get_paho.return_value = mock_mqtt
        
        publish_result = MagicMock()
        publish_result.rc = 0
        mock_client.publish.return_value = publish_result
        
        # Hook that reverses bytes (simulates encryption)
        def mock_encrypt(vehicle_id, payload):
            return payload[::-1]
        
        transport = MQTTTransport(on_pre_publish=mock_encrypt)
        transport._connected = True
        
        bsm = BSMCore(timestamp=1.0, msg_count=0, vehicle_id=1, speed=10.0)
        original_payload = BSMSerializer.serialize(bsm)
        
        transport.publish_bsm(1, bsm)
        
        # The published payload should be reversed (encrypted)
        call_args = mock_client.publish.call_args
        published_payload = call_args[0][1]
        self.assertEqual(published_payload, original_payload[::-1])
    
    @patch('src.v2v.mqtt_transport._get_paho')
    def test_post_receive_hook_transforms_payload(self, mock_get_paho):
        """Post-receive hook should transform payload before deserialization."""
        mock_mqtt = MagicMock()
        mock_client_class = MagicMock()
        mock_mqtt.Client = mock_client_class
        mock_mqtt.CallbackAPIVersion.VERSION2 = 2
        mock_mqtt.MQTTv5 = 5
        mock_get_paho.return_value = mock_mqtt
        
        # Hook that reverses bytes (simulates decryption)
        def mock_decrypt(vehicle_id, payload):
            return payload[::-1]
        
        transport = MQTTTransport(on_post_receive=mock_decrypt)
        
        # Create a BSM and reverse its serialization (simulating encrypted payload)
        bsm = BSMCore(timestamp=1.0, msg_count=0, vehicle_id=2, speed=15.0)
        real_payload = BSMSerializer.serialize(bsm)
        encrypted_payload = real_payload[::-1]  # Reversed = "encrypted"
        
        mock_msg = MagicMock()
        mock_msg.topic = 'v2v/bsm/2'
        mock_msg.payload = encrypted_payload
        
        transport._on_message(None, None, mock_msg)
        
        # The hook should reverse it back, making it deserializable
        received = transport.get_received_bsm()
        self.assertIn(2, received)
        self.assertEqual(received[2].speed, 15.0)
    
    @patch('src.v2v.mqtt_transport._get_paho')
    def test_get_stats_returns_copy(self, mock_get_paho):
        """Stats should return a copy, not the internal dict."""
        mock_mqtt = MagicMock()
        mock_client_class = MagicMock()
        mock_mqtt.Client = mock_client_class
        mock_mqtt.CallbackAPIVersion.VERSION2 = 2
        mock_mqtt.MQTTv5 = 5
        mock_get_paho.return_value = mock_mqtt
        
        transport = MQTTTransport()
        stats = transport.get_stats()
        
        self.assertIsInstance(stats, dict)
        self.assertIn('messages_published', stats)
        self.assertIn('messages_received', stats)
        self.assertIn('avg_publish_latency_ms', stats)
        
        # Mutating returned dict should not affect internal state
        stats['messages_published'] = 9999
        self.assertEqual(transport.stats['messages_published'], 0)


class TestV2VNetworkWithMQTTDisabled(unittest.TestCase):
    """
    Verify that V2VNetworkEnhanced works exactly as before when MQTT is disabled.
    This ensures backward compatibility.
    """
    
    def test_no_mqtt_by_default(self):
        """Network should have no MQTT transport when no config is passed."""
        from src.v2v.network_enhanced import V2VNetworkEnhanced
        from tests.v2v.test_v2v_basic import MockWorld
        
        world = MockWorld()
        v2v = V2VNetworkEnhanced(max_range=100.0, world=world)
        
        self.assertIsNone(v2v._mqtt_transport)
        self.assertFalse(v2v.mqtt_enabled)
        self.assertIsNone(v2v.get_mqtt_stats())
    
    def test_disabled_mqtt_config(self):
        """Network should have no MQTT transport when config.enabled=False."""
        from src.v2v.network_enhanced import V2VNetworkEnhanced
        from src.config import MQTTConfig
        from tests.v2v.test_v2v_basic import MockWorld
        
        world = MockWorld()
        mqtt_cfg = MQTTConfig(enabled=False)
        v2v = V2VNetworkEnhanced(max_range=100.0, world=world, mqtt_config=mqtt_cfg)
        
        self.assertIsNone(v2v._mqtt_transport)
        self.assertFalse(v2v.mqtt_enabled)
    
    def test_existing_functionality_unchanged(self):
        """Core V2V flow should work identically without MQTT."""
        from src.v2v.network_enhanced import V2VNetworkEnhanced
        from tests.v2v.test_v2v_basic import MockVehicle, MockWorld
        
        world = MockWorld()
        v2v = V2VNetworkEnhanced(max_range=100.0, world=world)
        
        v1 = MockVehicle(1, 0, 0)
        v2 = MockVehicle(2, 50, 0)
        
        for v in [v1, v2]:
            world.add_vehicle(v)
            v2v.register(v.id, v)
        
        v2v.update(force=True)
        
        # BSMs should be created
        bsm1 = v2v.get_bsm(1)
        self.assertIsNotNone(bsm1)
        self.assertEqual(bsm1.vehicle_id, 1)
        
        # Neighbors should be discovered
        neighbors = v2v.get_neighbors(1)
        neighbor_ids = [n.vehicle_id for n in neighbors]
        self.assertIn(2, neighbor_ids)
        
        # Stats should work
        stats = v2v.get_network_stats()
        self.assertGreater(stats['total_messages_sent'], 0)
    
    def test_shutdown_without_mqtt_is_safe(self):
        """Calling shutdown without MQTT should not raise."""
        from src.v2v.network_enhanced import V2VNetworkEnhanced
        from tests.v2v.test_v2v_basic import MockWorld
        
        world = MockWorld()
        v2v = V2VNetworkEnhanced(max_range=100.0, world=world)
        
        # Should not raise
        v2v.shutdown()


class TestMQTTConfigDataclass(unittest.TestCase):
    """Test MQTTConfig dataclass defaults and construction."""
    
    def test_defaults(self):
        """Default MQTTConfig should have MQTT disabled."""
        from src.config import MQTTConfig
        
        cfg = MQTTConfig()
        self.assertFalse(cfg.enabled)
        self.assertEqual(cfg.broker_host, 'localhost')
        self.assertEqual(cfg.broker_port, 1883)
        self.assertEqual(cfg.qos, 1)
        self.assertFalse(cfg.tls_enabled)
        self.assertIsNone(cfg.tls_ca_certs)
    
    def test_custom_config(self):
        """MQTTConfig should accept custom values."""
        from src.config import MQTTConfig
        
        cfg = MQTTConfig(
            enabled=True,
            broker_host='192.168.1.100',
            broker_port=8883,
            tls_enabled=True,
            tls_ca_certs='/path/to/ca.pem'
        )
        
        self.assertTrue(cfg.enabled)
        self.assertEqual(cfg.broker_host, '192.168.1.100')
        self.assertEqual(cfg.broker_port, 8883)
        self.assertTrue(cfg.tls_enabled)
        self.assertEqual(cfg.tls_ca_certs, '/path/to/ca.pem')


def run_tests():
    """Run all MQTT tests."""
    loader = unittest.TestLoader()
    suite = unittest.TestSuite()
    
    suite.addTests(loader.loadTestsFromTestCase(TestBSMSerializer))
    suite.addTests(loader.loadTestsFromTestCase(TestMQTTTransportUnit))
    suite.addTests(loader.loadTestsFromTestCase(TestV2VNetworkWithMQTTDisabled))
    suite.addTests(loader.loadTestsFromTestCase(TestMQTTConfigDataclass))
    
    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(suite)
    
    print("\n" + "=" * 70)
    print("MQTT TEST SUMMARY")
    print("=" * 70)
    print(f"Tests run: {result.testsRun}")
    print(f"Successes: {result.testsRun - len(result.failures) - len(result.errors)}")
    print(f"Failures: {len(result.failures)}")
    print(f"Errors: {len(result.errors)}")
    print("=" * 70)
    
    return result.wasSuccessful()


if __name__ == '__main__':
    success = run_tests()
    sys.exit(0 if success else 1)
