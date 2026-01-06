"""
V2V Communication Network for CARLA
Efficient neighbor discovery and state sharing implementation.
"""

import carla
from typing import Dict, List, Optional
import time
import logging
import math

from .protocol import V2VState


class V2VNetwork:
    """Manages V2V communication network and neighbor discovery."""
    
    def __init__(self, max_range: float = 50.0, update_interval: float = 0.1):
        """Initialize V2V network.
        
        Args:
            max_range: Maximum communication range in meters
            update_interval: Minimum time between state updates in seconds
        """
        self.max_range = max_range
        self.update_interval = update_interval
        self.vehicles: Dict[int, carla.Actor] = {}
        self.states: Dict[int, V2VState] = {}
        self.neighbors: Dict[int, List[int]] = {}
        self.last_update_time = 0.0
        self.logger = logging.getLogger(__name__)
        self.world = None
    
    def register(self, vehicle_id: int, vehicle: carla.Actor):
        """Register a vehicle in the V2V network.
        
        Args:
            vehicle_id: Unique identifier for the vehicle
            vehicle: CARLA vehicle actor
        """
        self.vehicles[vehicle_id] = vehicle
        self.neighbors[vehicle_id] = []
        
        # Store world reference from first vehicle
        if self.world is None:
            self.world = vehicle.get_world()
        
        self.logger.debug(f"Registered vehicle {vehicle_id} to V2V network")
    
    def unregister(self, vehicle_id: int):
        """Remove a vehicle from the V2V network.
        
        Args:
            vehicle_id: Vehicle identifier to remove
        """
        self.vehicles.pop(vehicle_id, None)
        self.states.pop(vehicle_id, None)
        self.neighbors.pop(vehicle_id, None)
        self.logger.debug(f"Unregistered vehicle {vehicle_id} from V2V network")
    
    def update(self, force: bool = False, snapshot=None):
        """Update all vehicle states and discover neighbors.
        
        CRITICAL: Uses world snapshot to get FRESH velocity data, not cached.
        
        Args:
            force: Force update even if interval hasn't elapsed
            snapshot: Optional WorldSnapshot from world.tick(). If None, will call get_snapshot()
        """
        current_time = time.time()
        
        if not force and (current_time - self.last_update_time) < self.update_interval:
            return
        
        if self.world is None:
            self.logger.error("World not initialized - call register() first")
            return
        
        # CRITICAL: Use provided snapshot or get current one
        # Snapshot from world.tick() is fresher than get_snapshot()
        if snapshot is None:
            snapshot = self.world.get_snapshot()
        sim_timestamp = snapshot.timestamp.elapsed_seconds
        
        # Update states for all registered vehicles using SNAPSHOT data
        for vehicle_id, vehicle in self.vehicles.items():
            try:
                # Get actor snapshot - this has FRESH data from current tick
                actor_snapshot = snapshot.find(vehicle.id)
                
                if actor_snapshot is None:
                    self.logger.warning(f"Vehicle {vehicle_id} (actor {vehicle.id}) not found in snapshot")
                    continue
                
                # Extract data from snapshot (guaranteed fresh!)
                transform = actor_snapshot.get_transform()
                vel_vec = actor_snapshot.get_velocity()  # FRESH velocity from snapshot
                
                # Calculate speed magnitude (m/s)
                speed_ms = math.sqrt(vel_vec.x**2 + vel_vec.y**2 + vel_vec.z**2)
                
                # Create V2VState with fresh data
                self.states[vehicle_id] = V2VState(
                    vehicle_id=vehicle_id,
                    timestamp=sim_timestamp,
                    location=(transform.location.x, transform.location.y, transform.location.z),
                    velocity=(vel_vec.x, vel_vec.y, vel_vec.z),
                    speed=speed_ms,  # m/s
                    yaw=transform.rotation.yaw
                )
                
            except Exception as e:
                self.logger.error(f"Failed to update state for vehicle {vehicle_id}: {e}")
                continue
        
        # Update neighbor relationships based on distance
        self._update_neighbors()
        
        self.last_update_time = current_time
    
    def _update_neighbors(self):
        """Internal method to update neighbor relationships based on max_range."""
        for vehicle_id in self.vehicles.keys():
            self.neighbors[vehicle_id] = []
            
            if vehicle_id not in self.states:
                continue
            
            my_state = self.states[vehicle_id]
            
            # Find all vehicles within communication range
            for other_id, other_state in self.states.items():
                if other_id == vehicle_id:
                    continue
                
                distance = my_state.distance_to(other_state)
                if distance <= self.max_range:
                    self.neighbors[vehicle_id].append(other_id)
    
    def get_neighbors(self, vehicle_id: int) -> List[V2VState]:
        """Get neighboring vehicles within communication range.
        
        Args:
            vehicle_id: ID of the vehicle to get neighbors for
            
        Returns:
            List of V2VState objects for neighboring vehicles
        """
        if vehicle_id not in self.neighbors:
            self.logger.warning(f"Vehicle {vehicle_id} not registered in V2V network")
            return []
        
        neighbor_states = []
        for neighbor_id in self.neighbors[vehicle_id]:
            if neighbor_id in self.states:
                neighbor_states.append(self.states[neighbor_id])
        
        return neighbor_states
    
    def get_state(self, vehicle_id: int) -> Optional[V2VState]:
        """Get current state of a vehicle.
        
        Args:
            vehicle_id: Vehicle identifier
            
        Returns:
            V2VState if vehicle exists, None otherwise
        """
        return self.states.get(vehicle_id)
    
    def broadcast(self, vehicle_id: int, message: dict):
        """Broadcast a message to all neighbors (placeholder for future protocols).
        
        Args:
            vehicle_id: Sender vehicle ID
            message: Message dictionary to broadcast
        """
        if vehicle_id not in self.neighbors:
            return
        
        # Future: Implement message queuing, delay, packet loss simulation
        self.logger.debug(f"Vehicle {vehicle_id} broadcasting to {len(self.neighbors[vehicle_id])} neighbors")
