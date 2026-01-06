"""
Enhanced V2V Network Manager
Implements BSM-based V2V communication with 2 Hz tick rate and cooperative perception.
"""

import carla
from typing import Dict, List, Optional, Tuple
import time
import logging
import math
from collections import deque

from .messages import (
    BSMCore, BSMPartII, V2VEnhancedMessage,
    create_bsm_from_carla, calculate_threat_level,
    PRIORITY_ROUTINE, PRIORITY_HIGH, PRIORITY_EMERGENCY,
    V2V_RANGE_MEDIUM, SHARE_SENSOR_DATA_DISTANCE
)

logger = logging.getLogger(__name__)


class V2VNetworkEnhanced:
    """
    Enhanced V2V Network Manager with BSM protocol support.
    
    Features:
    - 2 Hz tick rate for V2V updates
    - Bidirectional data sharing
    - BSM (Basic Safety Message) protocol
    - Cooperative perception
    - Threat assessment
    - Message prioritization
    """
    
    def __init__(self, 
                 max_range: float = V2V_RANGE_MEDIUM,
                 update_rate_hz: float = 2.0,
                 enable_cooperative_perception: bool = True,
                 world=None):
        """
        Initialize enhanced V2V network.
        
        Args:
            max_range: Maximum communication range in meters
            update_rate_hz: V2V update frequency (default 2 Hz)
            enable_cooperative_perception: Enable sensor data sharing
            world: CARLA World instance (optional, auto-detected from vehicles)
        """
        self.max_range = max_range
        self.update_rate_hz = update_rate_hz
        self.update_interval = 1.0 / update_rate_hz  # 0.5s for 2 Hz
        self.enable_coop_perception = enable_cooperative_perception
        
        # Vehicle registry
        self.vehicles: Dict[int, carla.Actor] = {}
        self.bsm_messages: Dict[int, BSMCore] = {}
        self.enhanced_messages: Dict[int, V2VEnhancedMessage] = {}
        
        # Message counters (0-127, wraps around)
        self.msg_counters: Dict[int, int] = {}
        
        # Network topology
        self.neighbors: Dict[int, List[int]] = {}  # vehicle_id -> [neighbor_ids]
        self.distances: Dict[Tuple[int, int], float] = {}  # (id1, id2) -> distance
        
        # Threat assessment
        self.threats: Dict[Tuple[int, int], dict] = {}  # (ego, other) -> threat_info
        
        # Previous velocities for acceleration calculation
        self.prev_speeds: Dict[int, float] = {}
        
        # Timing
        self.last_update_time = 0.0
        self.last_tick_time = 0.0
        self.world = world
        
        # Statistics
        self.stats = {
            'total_messages_sent': 0,
            'total_messages_received': 0,
            'average_neighbors': 0.0,
            'max_neighbors': 0,
            'cooperative_shares': 0
        }
        
        logger.info(f"V2V Network initialized: {update_rate_hz} Hz, range {max_range}m")
    
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
        # Check update timing (2 Hz)
        if not force and not self.should_update():
            return False
        
        current_time = time.time()
        delta_time = current_time - self.last_update_time
        self.last_update_time = current_time
        
        if snapshot is None and self.world:
            snapshot = self.world.get_snapshot()
        
        if snapshot is None:
            logger.warning("No snapshot available for V2V update")
            return False
        
        # Update BSM messages for all vehicles
        for vehicle_id, vehicle in self.vehicles.items():
            bsm = self._create_bsm(vehicle, vehicle_id, snapshot, delta_time)
            self.bsm_messages[vehicle_id] = bsm
            
            # Update message counter
            self.msg_counters[vehicle_id] = (self.msg_counters[vehicle_id] + 1) % 128
            
            # Update speed history
            self.prev_speeds[vehicle_id] = bsm.speed
        
        # Discover neighbors and calculate distances
        self._discover_neighbors()
        
        # Assess threats
        self._assess_threats()
        
        # Update statistics
        self._update_stats()
        
        logger.debug(f"V2V update completed: {len(self.vehicles)} vehicles, "
                    f"avg {self.stats['average_neighbors']:.1f} neighbors")
        
        return True
    
    def _create_bsm(self, vehicle: carla.Actor, vehicle_id: int, 
                    snapshot, delta_time: float) -> BSMCore:
        """Create BSM message from CARLA vehicle"""
        prev_speed = self.prev_speeds.get(vehicle_id, 0.0)
        msg_count = self.msg_counters.get(vehicle_id, 0)
        
        return create_bsm_from_carla(
            vehicle, vehicle_id, msg_count,
            prev_velocity=prev_speed,
            delta_time=delta_time
        )
    
    def _discover_neighbors(self):
        """Discover neighboring vehicles within communication range"""
        vehicle_ids = list(self.vehicles.keys())
        
        # Reset neighbors for all vehicles
        for vid in vehicle_ids:
            self.neighbors[vid] = []
        
        # Check all vehicle pairs
        for i, vid1 in enumerate(vehicle_ids):
            bsm1 = self.bsm_messages.get(vid1)
            if not bsm1:
                continue
            
            # Check against all OTHER vehicles (not just forward in list)
            for vid2 in vehicle_ids:
                if vid1 == vid2:  # Skip self
                    continue
                    
                bsm2 = self.bsm_messages.get(vid2)
                if not bsm2:
                    continue
                
                # Calculate distance
                dx = bsm2.latitude - bsm1.latitude
                dy = bsm2.longitude - bsm1.longitude
                distance = math.sqrt(dx**2 + dy**2)
                
                # Store distance (only once per pair)
                if (vid2, vid1) not in self.distances:
                    self.distances[(vid1, vid2)] = distance
                    self.distances[(vid2, vid1)] = distance
                
                # Check if within range and add to neighbors
                if distance <= self.max_range:
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
                    'timestamp': time.time()
                }
    
    def _update_stats(self):
        """Update network statistics"""
        if self.neighbors:
            neighbor_counts = [len(n) for n in self.neighbors.values()]
            self.stats['average_neighbors'] = sum(neighbor_counts) / len(neighbor_counts)
            self.stats['max_neighbors'] = max(neighbor_counts)
        
        self.stats['total_messages_sent'] += len(self.bsm_messages)
    
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
