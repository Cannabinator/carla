"""
V2V (Vehicle-to-Vehicle) Communication for CARLA
Lightweight implementation for cooperative perception and vehicle coordination.
"""

import carla
import math
import numpy as np
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple
import time


@dataclass
class V2VState:
    """Vehicle state for V2V communication."""
    vehicle_id: int
    timestamp: float
    location: tuple  # (x, y, z)
    velocity: tuple  # (vx, vy, vz) in m/s
    speed: float  # Speed in m/s
    yaw: float  # Heading in degrees
    
    @classmethod
    def from_vehicle(cls, vehicle_id: int, vehicle: carla.Actor, timestamp: float) -> 'V2VState':
        """Create V2VState from CARLA vehicle actor.
        
        ⚠️  WARNING: This method uses actor.get_velocity() which returns CACHED data.
        It should NOT be used in the main simulation loop.
        Use V2VNetwork.update() which uses snapshot data instead.
        
        For fresh velocity data, always use:
            world.tick()
            snapshot = world.get_snapshot()
            velocity = snapshot.find(vehicle.id).get_velocity()
        
        Args:
            vehicle_id: Unique vehicle identifier
            vehicle: CARLA vehicle actor
            timestamp: Current simulation timestamp
            
        Returns:
            V2VState instance with current vehicle data (⚠️  velocity may be stale!)
        """
        transform = vehicle.get_transform()
        velocity_vec = vehicle.get_velocity()  # ⚠️  CACHED - one tick behind!
        
        # Convert Vector3D to tuple
        velocity = (velocity_vec.x, velocity_vec.y, velocity_vec.z)
        
        # Calculate speed as magnitude of velocity vector (m/s)
        speed = math.sqrt(velocity_vec.x**2 + velocity_vec.y**2 + velocity_vec.z**2)
        
        return cls(
            vehicle_id=vehicle_id,
            timestamp=timestamp,
            location=(transform.location.x, transform.location.y, transform.location.z),
            velocity=velocity,
            speed=speed,
            yaw=transform.rotation.yaw
        )
    
    def distance_to(self, other: 'V2VState') -> float:
        """Calculate Euclidean distance to another vehicle.
        
        Args:
            other: Another V2VState instance
            
        Returns:
            Distance in meters
        """
        dx = self.location[0] - other.location[0]
        dy = self.location[1] - other.location[1]
        dz = self.location[2] - other.location[2]
        return math.sqrt(dx*dx + dy*dy + dz*dz)
