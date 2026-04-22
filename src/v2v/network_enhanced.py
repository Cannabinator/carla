"""
Enhanced V2V Network Manager

Implements SAE J2735 BSM-based V2V communication using a physics-accurate
DSRC/WAVE (IEEE 802.11p) channel model. Operates at a 2 Hz application
update rate aligned with CARLA world.tick(), with no external broker or
external processes.
"""

import carla
from typing import Dict, List, Optional, Tuple
import time
import logging
import math

from .messages import (
    BSMCore, BSMPartII, V2VEnhancedMessage, DENM, DENMCauseCode,
    create_bsm_from_carla, calculate_threat_level,
    PRIORITY_ROUTINE, PRIORITY_HIGH, PRIORITY_EMERGENCY,
    V2V_RANGE_MEDIUM, SHARE_SENSOR_DATA_DISTANCE
)
from .dsrc_channel import DSRCChannel, DSRCConfig

logger = logging.getLogger(__name__)


class V2VNetworkEnhanced:
    """
    Enhanced V2V Network Manager with SAE J2735 BSM protocol support.

    Features:
    - 2 Hz tick rate for V2V updates, gated on CARLA simulation time
    - SAE J2735 Basic Safety Message (BSM) protocol
    - IEEE 802.11p DSRC/WAVE channel model (log-distance path loss +
      log-normal shadowing + CSMA/CA contention)
    - Cooperative perception and threat assessment
    - Deterministic, in-process operation — no external broker required
    """
    
    def __init__(self,
                 max_range: float = V2V_RANGE_MEDIUM,
                 update_rate_hz: float = 2.0,
                 enable_cooperative_perception: bool = True,
                 world=None,
                 dsrc_config: Optional[DSRCConfig] = None):
        """
        Initialize enhanced V2V network.

        Args:
            max_range: Application-layer cooperative awareness zone in meters.
                       Vehicles outside this radius are excluded from neighbor
                       discovery regardless of physical channel reachability.
            update_rate_hz: BSM application update frequency (default 2 Hz per
                            SAE J2735 §7.1 minimum beacon interval).
            enable_cooperative_perception: Enable sensor data sharing.
            world: CARLA World instance (optional, auto-detected on first register).
            dsrc_config: DSRCConfig for the IEEE 802.11p channel model. When None,
                         ETSI EN 302 663 Class A OBU defaults are used.
        """
        self.max_range = max_range
        self.update_rate_hz = update_rate_hz
        self.update_interval = 1.0 / update_rate_hz  # 0.5 s for 2 Hz
        self.enable_coop_perception = enable_cooperative_perception

        # Vehicle registry
        self.vehicles: Dict[int, carla.Actor] = {}
        self.bsm_messages: Dict[int, BSMCore] = {}
        self.enhanced_messages: Dict[int, V2VEnhancedMessage] = {}

        # Message counters (0-127, wraps around per SAE J2735)
        self.msg_counters: Dict[int, int] = {}

        # Network topology — populated each 2 Hz tick
        self.neighbors: Dict[int, List[int]] = {}   # vehicle_id → [neighbor_ids]
        self.distances: Dict[Tuple[int, int], float] = {}  # (id1, id2) → metres

        # Threat assessment
        self.threats: Dict[Tuple[int, int], dict] = {}  # (ego, other) → threat_info

        # Previous speeds for BSM acceleration estimation
        self.prev_speeds: Dict[int, float] = {}

        # Timing
        self.last_update_time = 0.0
        self.last_update_sim_time: Optional[float] = None
        self.world = world

        # Statistics
        self.stats = {
            'total_messages_sent': 0,
            'total_messages_received': 0,
            'average_neighbors': 0.0,        # spatial avg over vehicles at last update
            'cumulative_avg_neighbors': 0.0,  # running Welford time-average
            'max_neighbors': 0,
            'cooperative_shares': 0,
            'total_update_count': 0,          # number of 2 Hz ticks executed
            'measured_update_hz': 0.0,        # achieved update rate (wall clock)
        }
        self._first_update_wall_time: Optional[float] = None

        # DENM store: (station_id, action_id) → DENM
        self.denm_store: Dict[Tuple[int, int], DENM] = {}

        # DSRC/WAVE (IEEE 802.11p) channel model
        self._dsrc_channel = DSRCChannel(dsrc_config)

        logger.info(
            "V2V Network initialized: %.1f Hz, range %.0fm "
            "[DSRC/WAVE channel: TX=%.0f dBm, n_LOS=%.2f, n_NLOS=%.2f, CBR=%.2f]",
            update_rate_hz, max_range,
            self._dsrc_channel.config.tx_power_dbm,
            self._dsrc_channel.config.path_loss_exponent_los,
            self._dsrc_channel.config.path_loss_exponent_nlos,
            self._dsrc_channel.config.channel_busy_ratio,
        )
    
    def register(self, vehicle_id: int, vehicle: carla.Actor):
        """
        Register a vehicle in the V2V network.
        
        Args:
            vehicle_id: Unique identifier
            vehicle: CARLA vehicle actor
        """
        self.vehicles[vehicle_id] = vehicle
        self.msg_counters[vehicle_id] = 0
        self.neighbors[vehicle_id] = []
        self.prev_speeds[vehicle_id] = 0.0
        
        if self.world is None:
            self.world = vehicle.get_world()
        
        logger.debug(f"Vehicle {vehicle_id} registered to V2V network")
    
    def unregister(self, vehicle_id: int):
        """Remove vehicle from network"""
        self.vehicles.pop(vehicle_id, None)
        self.bsm_messages.pop(vehicle_id, None)
        self.enhanced_messages.pop(vehicle_id, None)
        self.msg_counters.pop(vehicle_id, None)
        self.neighbors.pop(vehicle_id, None)
        self.prev_speeds.pop(vehicle_id, None)

        logger.debug(f"Vehicle {vehicle_id} unregistered from V2V network")
    
    def should_update(self) -> bool:
        """Check if enough time has passed for 2 Hz update"""
        current_time = time.time()
        return (current_time - self.last_update_time) >= self.update_interval
    
    def update(self, snapshot=None, force: bool = False) -> bool:
        """
        Update V2V network at 2 Hz rate.
        
        Args:
            snapshot: CARLA WorldSnapshot (fresh data)
            force: Force update regardless of timing
        
        Returns:
            True if update was performed
        """
        if snapshot is None and self.world:
            snapshot = self.world.get_snapshot()
        
        if snapshot is None:
            logger.warning("No snapshot available for V2V update")
            return False

        sim_time = self._extract_snapshot_time(snapshot)

        # Gate by simulation time when available for deterministic 2 Hz behavior.
        if not force:
            if sim_time is not None and self.last_update_sim_time is not None:
                if (sim_time - self.last_update_sim_time) < self.update_interval:
                    return False
            elif not self.should_update():
                return False

        current_time = time.time()
        if sim_time is not None and self.last_update_sim_time is not None:
            delta_time = max(sim_time - self.last_update_sim_time, 1e-6)
        elif self.last_update_time > 0.0:
            delta_time = max(current_time - self.last_update_time, 1e-6)
        else:
            delta_time = self.update_interval

        self.last_update_time = current_time
        if sim_time is not None:
            self.last_update_sim_time = sim_time

        successful_sends = 0

        # ── Phase 1: Build BSMs from current CARLA snapshot ──────────────────
        for vehicle_id, vehicle in self.vehicles.items():
            bsm = self._create_bsm(vehicle, vehicle_id, snapshot, delta_time)
            self.bsm_messages[vehicle_id] = bsm
            successful_sends += 1

            # Wrap SAE J2735 msg_count field (0-127)
            self.msg_counters[vehicle_id] = (self.msg_counters[vehicle_id] + 1) % 128
            self.prev_speeds[vehicle_id] = bsm.speed

        # ── Phase 2: Run DSRC/WAVE channel model ─────────────────────────────
        # Positions come from bsm.latitude/longitude which carry CARLA world
        # X/Y coordinates (not geodetic degrees — see create_bsm_from_carla).
        positions: Dict[int, Tuple[float, float]] = {
            vid: (bsm.latitude, bsm.longitude)
            for vid, bsm in self.bsm_messages.items()
        }
        self._dsrc_channel.broadcast_all(self.bsm_messages, positions)

        # ── Phase 3: Neighbor discovery using channel-filtered BSM view ───────
        self._discover_neighbors()

        # ── Phase 4: Threat assessment ────────────────────────────────────────
        self._assess_threats()

        # ── Phase 5: Statistics ───────────────────────────────────────────────
        self._update_stats()
        self.stats['total_messages_sent'] += successful_sends

        logger.debug(
            "V2V update: %d vehicles, avg %.1f neighbours, channel PRR=%.3f",
            len(self.vehicles),
            self.stats['average_neighbors'],
            self._dsrc_channel.stats.get('prr', 1.0),
        )
        return True
    
    def _create_bsm(self, vehicle: carla.Actor, vehicle_id: int, 
                    snapshot, delta_time: float) -> BSMCore:
        """Create BSM message from CARLA vehicle"""
        prev_speed = self.prev_speeds.get(vehicle_id, 0.0)
        msg_count = self.msg_counters.get(vehicle_id, 0)
        
        return create_bsm_from_carla(
            vehicle, vehicle_id, msg_count,
            snapshot=snapshot,  # Pass snapshot for fresh data
            prev_velocity=prev_speed,
            delta_time=delta_time
        )

    def _extract_snapshot_time(self, snapshot) -> Optional[float]:
        """Extract simulation time in seconds from a CARLA WorldSnapshot."""
        try:
            if hasattr(snapshot, 'timestamp') and snapshot.timestamp is not None:
                if hasattr(snapshot.timestamp, 'elapsed_seconds'):
                    return float(snapshot.timestamp.elapsed_seconds)
        except Exception:
            return None
        return None
    
    def _discover_neighbors(self):
        """
        Discover neighbours within the cooperative awareness zone.

        Two-stage filter:
          1. Application layer: geometric distance ≤ max_range.
          2. DSRC channel layer: BSM must have survived the IEEE 802.11p
             path-loss + shadowing + CSMA/CA PRR check in broadcast_all().

        Distances are always refreshed for all in-range pairs regardless of
        channel delivery result, so get_distance() and the REST API always
        reflect current geometry.
        """
        vehicle_ids = list(self.vehicles.keys())
        self.distances.clear()

        for vid in vehicle_ids:
            self.neighbors[vid] = []

        for vid1 in vehicle_ids:
            bsm1 = self.bsm_messages.get(vid1)
            if not bsm1:
                continue

            # BSMs that vid1 successfully received via DSRC channel this tick
            received_by_vid1 = self._dsrc_channel.get_received(vid1)

            for vid2 in vehicle_ids:
                if vid1 == vid2:
                    continue

                bsm2 = self.bsm_messages.get(vid2)
                if not bsm2:
                    continue

                dx = bsm2.latitude - bsm1.latitude
                dy = bsm2.longitude - bsm1.longitude
                distance = math.sqrt(dx * dx + dy * dy)

                # Always record geometric distance for APIs / stats
                self.distances[(vid1, vid2)] = distance
                self.distances[(vid2, vid1)] = distance

                # Neighbour only if within zone AND BSM was delivered by channel
                if distance <= self.max_range and vid2 in received_by_vid1:
                    self.neighbors[vid1].append(vid2)
    
    def _assess_threats(self):
        """Assess collision threats between vehicles"""
        self.threats.clear()
        
        for vid1, neighbors in self.neighbors.items():
            bsm1 = self.bsm_messages.get(vid1)
            if not bsm1:
                continue
            
            for vid2 in neighbors:
                bsm2 = self.bsm_messages.get(vid2)
                if not bsm2:
                    continue
                
                threat_level, ttc, distance = calculate_threat_level(bsm1, bsm2)
                
                self.threats[(vid1, vid2)] = {
                    'level': threat_level,
                    'ttc': ttc,
                    'distance': distance,
                    'timestamp': bsm1.timestamp
                }
    
    def _update_stats(self):
        """Update network statistics (called once per 2 Hz update)."""
        n = self.stats['total_update_count'] + 1
        self.stats['total_update_count'] = n

        if self.neighbors:
            neighbor_counts = [len(nbrs) for nbrs in self.neighbors.values()]
            instant_avg = sum(neighbor_counts) / len(neighbor_counts)
            self.stats['average_neighbors'] = instant_avg
            self.stats['max_neighbors'] = max(
                self.stats['max_neighbors'], max(neighbor_counts)
            )
            # Welford running mean for time-averaged neighbours
            prev_avg = self.stats['cumulative_avg_neighbors']
            self.stats['cumulative_avg_neighbors'] = prev_avg + (instant_avg - prev_avg) / n

            # Each neighbour relationship represents a successfully received BSM
            self.stats['total_messages_received'] += sum(neighbor_counts)
        else:
            self.stats['average_neighbors'] = 0.0

        # Measured update Hz (uses wall clock; available from 2nd update onward)
        now = time.time()
        if self._first_update_wall_time is None:
            self._first_update_wall_time = now
        elapsed_wall = now - self._first_update_wall_time
        if n > 1 and elapsed_wall > 0.0:
            # Exclude the first update from the denominator to avoid startup skew
            self.stats['measured_update_hz'] = (n - 1) / elapsed_wall
        
    def get_neighbors(self, vehicle_id: int) -> List[BSMCore]:
        """
        Get BSM messages from neighboring vehicles.
        
        Args:
            vehicle_id: Ego vehicle ID
        
        Returns:
            List of BSMCore messages from neighbors
        """
        neighbor_ids = self.neighbors.get(vehicle_id, [])
        return [self.bsm_messages[nid] for nid in neighbor_ids 
                if nid in self.bsm_messages]
    
    def get_bsm(self, vehicle_id: int) -> Optional[BSMCore]:
        """Get BSM message for specific vehicle"""
        return self.bsm_messages.get(vehicle_id)
    
    def get_all_bsm(self) -> Dict[int, BSMCore]:
        """Get all BSM messages"""
        return self.bsm_messages.copy()
    
    def get_threats(self, vehicle_id: int) -> List[dict]:
        """
        Get threat assessment for ego vehicle.
        
        Args:
            vehicle_id: Ego vehicle ID
        
        Returns:
            List of threat dictionaries
        """
        threats = []
        for (vid1, vid2), threat_info in self.threats.items():
            if vid1 == vehicle_id:
                threat_info_copy = threat_info.copy()
                threat_info_copy['other_vehicle_id'] = vid2
                threats.append(threat_info_copy)
        
        # Sort by threat level (highest first)
        threats.sort(key=lambda x: x['level'], reverse=True)
        return threats
    
    def get_distance(self, vid1: int, vid2: int) -> Optional[float]:
        """Get distance between two vehicles"""
        return self.distances.get((vid1, vid2))
    
    def get_network_stats(self) -> dict:
        """Get network statistics"""
        return self.stats.copy()
    
    def enable_bidirectional_sharing(self, vehicle_id: int, 
                                     sensor_data: dict) -> List[int]:
        """
        Share sensor data with neighbors bidirectionally.
        
        Args:
            vehicle_id: Source vehicle ID
            sensor_data: Dictionary of sensor data to share
        
        Returns:
            List of vehicle IDs that received the data
        """
        if not self.enable_coop_perception:
            return []
        
        neighbors = self.neighbors.get(vehicle_id, [])
        recipients = []
        
        for neighbor_id in neighbors:
            distance = self.get_distance(vehicle_id, neighbor_id)
            
            # Share sensor data only with close neighbors
            if distance and distance <= SHARE_SENSOR_DATA_DISTANCE:
                recipients.append(neighbor_id)
                self.stats['cooperative_shares'] += 1
        
        return recipients
    
    def get_one_line_status(self, ego_id: int = 0) -> str:
        """
        Get one-line status string for console output.
        
        Args:
            ego_id: Ego vehicle ID (default 0)
        
        Returns:
            Formatted one-line status string
        """
        ego_bsm = self.bsm_messages.get(ego_id)
        if not ego_bsm:
            return "V2V: No data"
        
        neighbors = self.neighbors.get(ego_id, [])
        threats = self.get_threats(ego_id)
        high_threats = [t for t in threats if t['level'] >= 3]
        
        return (f"V2V: {ego_bsm.speed:5.1f}m/s | "
                f"Heading:{ego_bsm.heading:6.1f}° | "
                f"Neighbors:{len(neighbors):2d} | "
                f"Threats:{len(high_threats):2d} | "
                f"Msgs:{self.msg_counters.get(ego_id, 0):3d}")

    # ── DENM management ─────────────────────────────────────────────────────────

    def publish_denm(self, denm: DENM) -> None:
        """
        Store or update a DENM alert originating from *station_id*.

        A DENM is uniquely identified by (station_id, action_id).
        Calling publish_denm with the same key overwrites the previous entry
        (i.e. this acts as an update / keep-alive).
        """
        self.denm_store[(denm.station_id, denm.action_id)] = denm
        logger.debug(
            "DENM published: station=%d action=%d cause=%s",
            denm.station_id, denm.action_id, denm.cause_code.name,
        )

    def cancel_denm(self, station_id: int, action_id: int) -> bool:
        """
        Mark a DENM as cancelled (ETSI termination = isCancellation) and
        remove it from the active store.

        Returns True if the DENM existed and was removed, False otherwise.
        """
        key = (station_id, action_id)
        if key in self.denm_store:
            del self.denm_store[key]
            logger.debug("DENM cancelled: station=%d action=%d", station_id, action_id)
            return True
        return False

    def get_denm(self, station_id: Optional[int] = None) -> List[DENM]:
        """
        Return active DENMs.

        Args:
            station_id: If given, return only DENMs from that station.
                        If None, return all active DENMs.
        """
        if station_id is None:
            return list(self.denm_store.values())
        return [d for (sid, _), d in self.denm_store.items() if sid == station_id]

    # ── Channel access ──────────────────────────────────────────────────────────

    def get_channel_stats(self) -> dict:
        """
        Get DSRC/WAVE channel model statistics for research analysis.

        Returns:
            Dict containing:
              total_broadcasts   — total (sender, receiver) pair attempts
              total_deliveries   — successful packet deliveries
              total_drops        — packets dropped by channel model
              prr                — rolling packet reception rate [0, 1]
              avg_snr_margin_db  — average SNR margin above sensitivity floor
        """
        return self._dsrc_channel.get_stats()

    def shutdown(self):
        """
        Clean shutdown of the V2V network.

        No external connections to close — the DSRC channel is in-process.
        Call this before destroying CARLA actors to flush any pending state.
        """
        logger.info(
            "V2V Network shutdown. Final channel PRR=%.3f, deliveries=%d, drops=%d",
            self._dsrc_channel.stats.get("prr", 1.0),
            int(self._dsrc_channel.stats.get("total_deliveries", 0)),
            int(self._dsrc_channel.stats.get("total_drops", 0)),
        )
