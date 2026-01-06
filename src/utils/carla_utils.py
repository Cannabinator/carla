"""
Utility functions for CARLA operations.
Reduces code duplication and improves reusability.
"""

import carla
import numpy as np
import math
from typing import Optional, Tuple


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
    tm = client.get_trafficmanager(port)
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
