"""
Utility functions for CARLA operations.
Reduces code duplication and improves reusability.
"""

import carla
import numpy as np
import math
import logging
import random
from typing import Optional, Tuple, Dict, List

logger = logging.getLogger(__name__)


def calculate_speed(velocity: carla.Vector3D) -> Tuple[float, float]:
    """Calculate speed from velocity vector.
    
    Args:
        velocity: CARLA Vector3D velocity
        
    Returns:
        Tuple of (speed_ms, speed_kmh) - speed in m/s and km/h
    """
    speed_ms = math.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2)
    speed_kmh = speed_ms * 3.6
    return speed_ms, speed_kmh


def calculate_distance_2d(loc1: tuple, loc2: tuple) -> float:
    """Calculate 2D Euclidean distance between two locations.
    
    Args:
        loc1: (x, y, z) tuple for first location
        loc2: (x, y, z) tuple for second location
        
    Returns:
        Distance in meters (ignoring z)
    """
    dx = loc1[0] - loc2[0]
    dy = loc1[1] - loc2[1]
    return math.sqrt(dx*dx + dy*dy)


def calculate_distance_3d(loc1: tuple, loc2: tuple) -> float:
    """Calculate 3D Euclidean distance between two locations.
    
    Args:
        loc1: (x, y, z) tuple for first location
        loc2: (x, y, z) tuple for second location
        
    Returns:
        Distance in meters
    """
    return np.linalg.norm([loc1[0] - loc2[0],
                           loc1[1] - loc2[1],
                           loc1[2] - loc2[2]])


def setup_synchronous_mode(world: carla.World, delta_seconds: float = 0.05) -> carla.WorldSettings:
    """Configure synchronous mode for deterministic simulation.
    
    Args:
        world: CARLA world instance
        delta_seconds: Fixed time step (default 0.05 = 20 FPS)
        
    Returns:
        Original world settings (for restoration later)
    """
    original_settings = world.get_settings()
    settings = world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = delta_seconds
    settings.no_rendering_mode = False
    world.apply_settings(settings)
    return original_settings


def restore_world_settings(world: carla.World, original_settings: carla.WorldSettings):
    """Restore original world settings.
    
    Args:
        world: CARLA world instance
        original_settings: Original settings to restore
    """
    if world and original_settings:
        world.apply_settings(original_settings)


def setup_traffic_manager(client: carla.Client, port: int = 8000, 
                         seed: int = 42, use_hybrid: bool = False,
                         hybrid_radius: float = 70.0) -> carla.TrafficManager:
    """Setup and configure Traffic Manager.
    
    Args:
        client: CARLA client instance
        port: Traffic Manager port
        seed: Random seed for determinism
        use_hybrid: Enable hybrid physics mode (NOT recommended - causes zero velocity)
        hybrid_radius: Radius for full physics in hybrid mode
        
    Returns:
        Configured TrafficManager instance
    """
    try:
        tm = client.get_trafficmanager(port)
    except RuntimeError as e:
        # Port might be in use, try to reuse existing TM
        logger.warning(f"Traffic Manager port {port} in use, attempting to reuse existing instance")
        import time
        time.sleep(2)  # Wait for port cleanup
        try:
            tm = client.get_trafficmanager(port)
        except RuntimeError:
            # Try different port as fallback
            logger.warning(f"Retrying with port {port + 1}")
            tm = client.get_trafficmanager(port + 1)
    
    tm.set_synchronous_mode(True)
    tm.set_random_device_seed(seed)
    
    if use_hybrid:
        tm.set_hybrid_physics_mode(True)
        tm.set_hybrid_physics_radius(hybrid_radius)
    
    return tm


def get_fresh_velocity(snapshot: carla.WorldSnapshot, actor_id: int) -> Optional[carla.Vector3D]:
    """Get fresh velocity data from world snapshot.
    
    CRITICAL: Always use this instead of actor.get_velocity() to avoid stale data.
    
    Args:
        snapshot: World snapshot from world.tick() or world.get_snapshot()
        actor_id: Actor ID to query
        
    Returns:
        Fresh velocity Vector3D or None if actor not found
    """
    actor_snapshot = snapshot.find(actor_id)
    if actor_snapshot:
        return actor_snapshot.get_velocity()
    return None


def spawn_vehicle(world: carla.World, blueprint: carla.ActorBlueprint, 
                 transform: carla.Transform) -> Optional[carla.Actor]:
    """Safely spawn a vehicle.
    
    Args:
        world: CARLA world instance
        blueprint: Vehicle blueprint
        transform: Spawn transform
        
    Returns:
        Spawned vehicle actor or None if failed
    """
    try:
        vehicle = world.spawn_actor(blueprint, transform)
        return vehicle
    except RuntimeError as e:
        return None


def destroy_actors(client: carla.Client, actors: list):
    """Safely destroy multiple actors.
    
    Args:
        client: CARLA client instance
        actors: List of actors to destroy
    """
    if client and actors:
        client.apply_batch([carla.command.DestroyActor(x) for x in actors])


class StuckVehicleTracker:
    """
    Tracks and manages stuck vehicles in the simulation.
    
    A vehicle is considered "stuck" if it maintains a velocity below the threshold
    for a specified number of consecutive frames, indicating it may have crashed
    or gotten wedged in geometry.
    
    Features:
    - Velocity-based stuck detection
    - Configurable thresholds (velocity, frame count)
    - Per-vehicle tracking with frame counters
    - Recovery via physics reset
    
    Example:
        >>> tracker = StuckVehicleTracker(velocity_threshold=0.5, frames_threshold=100)
        >>> if tracker.check_and_update(vehicle, snapshot):
        ...     tracker.recover_vehicle(vehicle, spawn_points)
    """
    
    def __init__(
        self,
        velocity_threshold: float = 0.5,  # m/s
        frames_threshold: int = 100  # frames
    ):
        """
        Initialize stuck vehicle tracker.
        
        Args:
            velocity_threshold: Speed below which vehicle is considered stuck (m/s)
            frames_threshold: Consecutive frames below threshold to trigger stuck state
        """
        self.velocity_threshold = velocity_threshold
        self.frames_threshold = frames_threshold
        self.stuck_counters: Dict[int, int] = {}  # actor_id -> stuck_frame_count
        self.recovered_vehicles: List[int] = []  # List of recovered vehicle IDs
        
    def check_and_update(
        self,
        vehicle: carla.Actor,
        snapshot: carla.WorldSnapshot
    ) -> bool:
        """
        Check if vehicle is stuck and update tracking state.
        
        Args:
            vehicle: Vehicle actor to check
            snapshot: Current world snapshot
            
        Returns:
            True if vehicle is stuck and needs recovery, False otherwise
        """
        actor_id = vehicle.id
        
        # Get fresh velocity from snapshot
        velocity = get_fresh_velocity(snapshot, actor_id)
        if velocity is None:
            return False
        
        # Calculate speed
        speed = math.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2)
        
        # Check if below threshold
        if speed < self.velocity_threshold:
            # Increment stuck counter
            self.stuck_counters[actor_id] = self.stuck_counters.get(actor_id, 0) + 1
            
            # Check if reached threshold
            if self.stuck_counters[actor_id] >= self.frames_threshold:
                return True
        else:
            # Vehicle is moving, reset counter
            if actor_id in self.stuck_counters:
                self.stuck_counters[actor_id] = 0
        
        return False
    
    def recover_vehicle(
        self,
        vehicle: carla.Actor,
        spawn_points: Optional[List[carla.Transform]] = None,
        world: Optional[carla.World] = None
    ) -> bool:
        """
        Recover a stuck vehicle by resetting its physics state.
        
        Strategy:
        1. First try: Reset physics (angular/linear velocity)
        2. If spawn_points provided: Teleport to new location
        
        Args:
            vehicle: Stuck vehicle to recover
            spawn_points: Optional list of available spawn points
            world: Optional world instance for teleport
            
        Returns:
            True if recovery attempted, False otherwise
        """
        actor_id = vehicle.id
        
        try:
            # Strategy 1: Reset physics state (least disruptive)
            vehicle.set_target_velocity(carla.Vector3D(0, 0, 0))
            vehicle.set_target_angular_velocity(carla.Vector3D(0, 0, 0))
            
            # Strategy 2: If spawn points available, teleport to new location
            if spawn_points and world and len(spawn_points) > 0:
                new_spawn = random.choice(spawn_points)
                vehicle.set_transform(new_spawn)
                logger.info(f"Recovered stuck vehicle {actor_id} - teleported to new location")
            else:
                logger.info(f"Recovered stuck vehicle {actor_id} - physics reset")
            
            # Reset counter
            self.stuck_counters[actor_id] = 0
            self.recovered_vehicles.append(actor_id)
            
            return True
            
        except Exception as e:
            logger.error(f"Failed to recover vehicle {actor_id}: {e}")
            return False
    
    def get_stats(self) -> Dict[str, int]:
        """
        Get statistics about stuck vehicle tracking.
        
        Returns:
            Dictionary with tracking statistics
        """
        currently_stuck = sum(1 for count in self.stuck_counters.values() 
                            if count > self.frames_threshold // 2)
        
        return {
            'total_recovered': len(self.recovered_vehicles),
            'currently_tracked': len(self.stuck_counters),
            'currently_stuck': currently_stuck,
            'velocity_threshold': self.velocity_threshold,
            'frames_threshold': self.frames_threshold
        }
    
    def reset(self):
        """Reset all tracking state."""
        self.stuck_counters.clear()
        self.recovered_vehicles.clear()
