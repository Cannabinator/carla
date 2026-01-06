#!/usr/bin/env python3
"""
CARLA session management with context managers for guaranteed cleanup.
"""

import carla
import logging
from typing import Optional, List
from dataclasses import dataclass

from .carla_utils import setup_synchronous_mode, restore_world_settings, destroy_actors

logger = logging.getLogger(__name__)


class CARLASession:
    """
    Context manager for CARLA connections with automatic cleanup.
    
    Guarantees proper cleanup even on exceptions:
    - Restores world settings
    - Destroys all spawned actors
    - Handles errors gracefully
    
    Example:
        >>> with CARLASession('192.168.1.110', 2000, config) as session:
        ...     ego = session.world.spawn_actor(bp, spawn_point)
        ...     session.actors.append(ego)
        ...     # ... scenario code ...
        ... # Automatic cleanup on exit!
    """
    
    def __init__(self, host: str, port: int, config):
        """
        Initialize CARLA session.
        
        Args:
            host: CARLA server IP
            port: CARLA server port
            config: SimulationConfig instance
        """
        self.host = host
        self.port = port
        self.config = config
        self.client: Optional[carla.Client] = None
        self.world: Optional[carla.World] = None
        self.original_settings: Optional[carla.WorldSettings] = None
        self.actors: List[carla.Actor] = []
        self.bp_lib: Optional[carla.BlueprintLibrary] = None
        self.spawn_points: List[carla.Transform] = []
    
    def __enter__(self):
        """Connect to CARLA and setup synchronous mode."""
        logger.info(f"Connecting to CARLA server at {self.host}:{self.port}")
        
        self.client = carla.Client(self.host, self.port)
        self.client.set_timeout(self.config.timeout)
        self.world = self.client.get_world()
        
        logger.info(f"Connected to map: {self.world.get_map().name}")
        
        # Setup synchronous mode
        self.original_settings = setup_synchronous_mode(
            self.world, 
            self.config.fixed_delta_seconds
        )
        logger.info(f"Synchronous mode enabled: delta={self.config.fixed_delta_seconds}s")
        
        # Prepare common resources
        self.bp_lib = self.world.get_blueprint_library()
        self.spawn_points = self.world.get_map().get_spawn_points()
        
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        """Cleanup resources on exit."""
        logger.info("Starting CARLA session cleanup...")
        
        # Restore world settings
        if self.world and self.original_settings:
            restore_world_settings(self.world, self.original_settings)
            logger.info("World settings restored")
        
        # Destroy actors
        if self.client and self.actors:
            destroy_actors(self.client, self.actors)
            logger.info(f"Destroyed {len(self.actors)} actors")
        
        logger.info("✓ CARLA session cleanup complete")
        
        # Don't suppress exceptions
        return False
    
    def add_actor(self, actor: carla.Actor) -> carla.Actor:
        """
        Register an actor for automatic cleanup.
        
        Args:
            actor: CARLA actor to track
            
        Returns:
            The same actor (for chaining)
        """
        self.actors.append(actor)
        return actor


@dataclass
class VehicleState:
    """
    Complete vehicle state snapshot from CARLA.
    
    Makes code more readable and type-safe compared to raw tuples.
    """
    frame: int
    position: tuple[float, float, float]  # (x, y, z) in meters
    velocity: tuple[float, float, float]  # (vx, vy, vz) in m/s
    orientation: tuple[float, float, float]  # (yaw, pitch, roll) in degrees
    angular_velocity: tuple[float, float, float]  # (wx, wy, wz) in deg/s
    speed_ms: float  # Speed magnitude in m/s
    speed_kmh: float  # Speed magnitude in km/h
    control: Optional[carla.VehicleControl] = None
    
    @classmethod
    def from_snapshot(
        cls, 
        frame: int, 
        actor_snapshot, 
        control: Optional[carla.VehicleControl] = None
    ) -> 'VehicleState':
        """
        Create VehicleState from CARLA actor snapshot.
        
        Args:
            frame: Current simulation frame
            actor_snapshot: CARLA ActorSnapshot from world.get_snapshot()
            control: Optional vehicle control state
            
        Returns:
            VehicleState instance
        """
        import numpy as np
        
        transform = actor_snapshot.get_transform()
        velocity = actor_snapshot.get_velocity()
        angular_velocity = actor_snapshot.get_angular_velocity()
        
        # Calculate speed magnitude
        speed_ms = np.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2)
        speed_kmh = speed_ms * 3.6
        
        return cls(
            frame=frame,
            position=(transform.location.x, transform.location.y, transform.location.z),
            velocity=(velocity.x, velocity.y, velocity.z),
            orientation=(transform.rotation.yaw, transform.rotation.pitch, transform.rotation.roll),
            angular_velocity=(
                np.degrees(angular_velocity.x),
                np.degrees(angular_velocity.y),
                np.degrees(angular_velocity.z)
            ),
            speed_ms=speed_ms,
            speed_kmh=speed_kmh,
            control=control
        )
    
    def __str__(self) -> str:
        """Human-readable state representation."""
        return (
            f"VehicleState(frame={self.frame}, "
            f"pos=({self.position[0]:.1f}, {self.position[1]:.1f}, {self.position[2]:.1f}), "
            f"speed={self.speed_kmh:.1f}km/h)"
        )
