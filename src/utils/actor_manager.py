#!/usr/bin/env python3
"""
Actor management for CARLA scenarios.
Centralizes spawning, tracking, and cleanup of vehicles and other actors.
"""

import carla
import random
import logging
from typing import List, Dict, Optional

logger = logging.getLogger(__name__)


class ActorManager:
    """
    Manages spawning, tracking, and cleanup of CARLA actors.
    
    Benefits:
    - Single responsibility for actor lifecycle
    - Automatic tracking of all spawned actors
    - Consistent error handling
    - Easy testing with mocks
    
    Example:
        >>> manager = ActorManager(world, bp_lib)
        >>> ego = manager.spawn_ego('vehicle.tesla.model3', spawn_point, '255,0,0')
        >>> traffic = manager.spawn_traffic(10, spawn_points[1:])
        >>> all_actors = manager.get_all()
    """
    
    def __init__(self, world: carla.World, blueprint_library: carla.BlueprintLibrary):
        """
        Initialize actor manager.
        
        Args:
            world: CARLA world instance
            blueprint_library: CARLA blueprint library
        """
        self.world = world
        self.bp_lib = blueprint_library
        self.actors: List[carla.Actor] = []
        self.actor_map: Dict[int, carla.Actor] = {}  # vehicle_id -> actor
    
    def spawn_ego(
        self, 
        blueprint_id: str, 
        spawn_point: carla.Transform, 
        color: str
    ) -> carla.Actor:
        """
        Spawn ego (leading) vehicle with specific configuration.
        
        Args:
            blueprint_id: Vehicle blueprint ID (e.g., 'vehicle.tesla.model3')
            spawn_point: Spawn transform
            color: RGB color string (e.g., '255,0,0' for red)
            
        Returns:
            Spawned vehicle actor
            
        Raises:
            RuntimeError: If spawn fails
        """
        bp = self.bp_lib.filter(blueprint_id)[0]
        bp.set_attribute('color', color)
        
        ego = self.world.spawn_actor(bp, spawn_point)
        self.actors.append(ego)
        self.actor_map[0] = ego  # Ego is always ID 0
        
        logger.info(f"Ego vehicle spawned - ID: {ego.id}, Type: {blueprint_id}, Location: {spawn_point.location}")
        
        return ego
    
    def spawn_traffic(
        self, 
        num_vehicles: int, 
        spawn_points: List[carla.Transform],
        min_wheels: int = 4
    ) -> List[carla.Actor]:
        """
        Spawn traffic vehicles with variety.
        
        Args:
            num_vehicles: Number of traffic vehicles to spawn
            spawn_points: Available spawn points
            min_wheels: Minimum number of wheels (default: 4, filters out bikes)
            
        Returns:
            List of successfully spawned vehicle actors
        """
        # Get valid vehicle blueprints
        vehicle_bps = [
            x for x in self.bp_lib.filter('vehicle.*') 
            if int(x.get_attribute('number_of_wheels')) >= min_wheels
        ]
        
        traffic = []
        spawn_failures = 0
        
        for i, spawn_point in enumerate(spawn_points[:num_vehicles]):
            try:
                # Random vehicle type
                bp = random.choice(vehicle_bps)
                
                # Random color if available
                if bp.has_attribute('color'):
                    color = random.choice(bp.get_attribute('color').recommended_values)
                    bp.set_attribute('color', color)
                
                # Spawn
                vehicle = self.world.spawn_actor(bp, spawn_point)
                self.actors.append(vehicle)
                self.actor_map[i + 1] = vehicle  # Traffic starts at ID 1
                traffic.append(vehicle)
                
            except RuntimeError as e:
                # Spawn point collision - skip
                spawn_failures += 1
                logger.debug(f"Failed to spawn traffic vehicle at point {i}: {e}")
                continue
        
        logger.info(f"Spawned {len(traffic)} traffic vehicles ({spawn_failures} failures)")
        
        return traffic
    
    def get_all(self) -> List[carla.Actor]:
        """Get all managed actors."""
        return self.actors.copy()
    
    def get_by_id(self, vehicle_id: int) -> Optional[carla.Actor]:
        """
        Get actor by vehicle ID.
        
        Args:
            vehicle_id: Vehicle ID (0 = ego, 1+ = traffic)
            
        Returns:
            Actor or None if not found
        """
        return self.actor_map.get(vehicle_id)
    
    def get_ego(self) -> Optional[carla.Actor]:
        """Get ego vehicle (convenience method)."""
        return self.actor_map.get(0)
    
    def count(self) -> int:
        """Get total number of managed actors."""
        return len(self.actors)
    
    def cleanup(self, client: carla.Client):
        """
        Destroy all managed actors.
        
        Args:
            client: CARLA client instance
        """
        if self.actors:
            client.apply_batch([carla.command.DestroyActor(x) for x in self.actors])
            logger.info(f"Destroyed {len(self.actors)} actors")
        
        self.actors.clear()
        self.actor_map.clear()
