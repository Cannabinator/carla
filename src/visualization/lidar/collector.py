#!/usr/bin/env python3
"""
LiDAR Data Collector for V2V Communication
Collects and processes semantic LiDAR data from multiple vehicles.
"""

import numpy as np
import carla
import logging
from typing import Dict, Optional

logger = logging.getLogger(__name__)


class LiDARDataCollector:
    """Collects and processes semantic LiDAR data from multiple vehicles."""
    
    def __init__(self, world: carla.World, downsample_factor: int = 1):
        """
        Initialize LiDAR data collector.
        
        Args:
            world: CARLA world instance
            downsample_factor: Keep every Nth point (1=no downsampling, higher=more downsampling)
        """
        self.world = world
        self.downsample_factor = downsample_factor
        self.vehicles: Dict[int, carla.Actor] = {}
        self.lidar_sensors: Dict[int, carla.Sensor] = {}
        self.latest_data: Dict[int, Optional[np.ndarray]] = {}
        self.vehicle_transforms: Dict[int, carla.Transform] = {}
        self.actor_ids: Dict[int, int] = {}  # Map vehicle_id -> actor_id
        
    def register_vehicle(self, vehicle_id: int, vehicle: carla.Actor):
        """Register a vehicle and attach semantic LiDAR sensor.
        
        Args:
            vehicle_id: Unique vehicle identifier
            vehicle: CARLA vehicle actor
        """
        self.vehicles[vehicle_id] = vehicle
        self.actor_ids[vehicle_id] = vehicle.id  # Store actor ID
        self.latest_data[vehicle_id] = None
        
        # Create semantic LiDAR blueprint
        bp_lib = self.world.get_blueprint_library()
        lidar_bp = bp_lib.find('sensor.lidar.ray_cast_semantic')
        
        # Configure high-quality LiDAR parameters for dense point cloud
        lidar_bp.set_attribute('channels', '64')  # More laser beams (32 → 64)
        lidar_bp.set_attribute('range', '100.0')  # Longer range (50 → 100m)
        lidar_bp.set_attribute('points_per_second', '1000000')  # 10x more points (100k → 1M)
        lidar_bp.set_attribute('rotation_frequency', '20.0')  # Faster rotation (10 → 20 Hz)
        lidar_bp.set_attribute('upper_fov', '15.0')  # Wider vertical FOV (10 → 15)
        lidar_bp.set_attribute('lower_fov', '-25.0')  # Better ground coverage
        lidar_bp.set_attribute('horizontal_fov', '360.0')  # Full 360° coverage
        
        # Attach LiDAR to vehicle roof
        lidar_transform = carla.Transform(carla.Location(x=0.0, z=2.4))
        lidar_sensor = self.world.spawn_actor(lidar_bp, lidar_transform, attach_to=vehicle)
        
        # Setup callback
        lidar_sensor.listen(lambda data: self._on_lidar_data(vehicle_id, data))
        
        self.lidar_sensors[vehicle_id] = lidar_sensor
        logger.info(f"Registered vehicle {vehicle_id} with semantic LiDAR")
        
    def _on_lidar_data(self, vehicle_id: int, data):
        """Callback for LiDAR data reception.
        
        Args:
            vehicle_id: Vehicle that generated the data
            data: CARLA SemanticLidarMeasurement
        """
        try:
            # Convert to numpy array: [x, y, z, cos_angle, object_tag, object_idx]
            points = np.frombuffer(data.raw_data, dtype=np.dtype([
                ('x', np.float32), ('y', np.float32), ('z', np.float32),
                ('cos_inc_angle', np.float32), ('object_tag', np.uint32), ('object_idx', np.uint32)
            ]))
            
            # Downsample only if factor > 1
            if self.downsample_factor > 1:
                points = points[::self.downsample_factor]
            
            # Get sensor transform directly from measurement (always available, even if vehicle destroyed)
            # Data contains sensor transform at measurement time - no need to query vehicle
            self.vehicle_transforms[vehicle_id] = data.transform
            
            self.latest_data[vehicle_id] = points
        except Exception as e:
            logger.error(f"Error processing LiDAR data for vehicle {vehicle_id}: {e}")
        
    def transform_to_world_coords(self, vehicle_id: int, local_points: np.ndarray) -> np.ndarray:
        """Transform local LiDAR coordinates to world coordinates.
        
        Args:
            vehicle_id: Vehicle ID
            local_points: Nx6 array of local points [x, y, z, cos, tag, idx]
            
        Returns:
            Nx6 array with world coordinates [x, y, z, cos, tag, idx]
        """
        if vehicle_id not in self.vehicle_transforms:
            return local_points
            
        transform = self.vehicle_transforms[vehicle_id]
        
        # Extract position and rotation
        location = transform.location
        rotation = transform.rotation
        
        # Convert rotation to radians
        yaw = np.radians(rotation.yaw)
        pitch = np.radians(rotation.pitch)
        roll = np.radians(rotation.roll)
        
        # Create rotation matrix (Unreal Engine coordinate system: X-forward, Y-right, Z-up)
        # Rotation order: Roll -> Pitch -> Yaw (ZYX Euler angles)
        cos_yaw, sin_yaw = np.cos(yaw), np.sin(yaw)
        cos_pitch, sin_pitch = np.cos(pitch), np.sin(pitch)
        cos_roll, sin_roll = np.cos(roll), np.sin(roll)
        
        # Combined rotation matrix
        rotation_matrix = np.array([
            [cos_yaw * cos_pitch, 
             cos_yaw * sin_pitch * sin_roll - sin_yaw * cos_roll,
             cos_yaw * sin_pitch * cos_roll + sin_yaw * sin_roll],
            [sin_yaw * cos_pitch,
             sin_yaw * sin_pitch * sin_roll + cos_yaw * cos_roll,
             sin_yaw * sin_pitch * cos_roll - cos_yaw * sin_roll],
            [-sin_pitch,
             cos_pitch * sin_roll,
             cos_pitch * cos_roll]
        ])
        
        # Extract XYZ coordinates
        local_xyz = np.column_stack([local_points['x'], local_points['y'], local_points['z']])
        
        # Apply rotation and translation
        world_xyz = (rotation_matrix @ local_xyz.T).T
        world_xyz[:, 0] += location.x
        world_xyz[:, 1] += location.y
        world_xyz[:, 2] += location.z
        
        # Create output array with world coordinates
        world_points = local_points.copy()
        world_points['x'] = world_xyz[:, 0]
        world_points['y'] = world_xyz[:, 1]
        world_points['z'] = world_xyz[:, 2]
        
        return world_points
        
    def get_combined_pointcloud(self) -> Optional[Dict]:
        """Get combined point cloud from all vehicles in world coordinates.
        
        Returns:
            Dictionary with point cloud data or None if no data available
        """
        all_points = []
        vehicle_ids = []
        
        for vehicle_id, points in self.latest_data.items():
            if points is not None and len(points) > 0:
                # Transform to world coordinates
                world_points = self.transform_to_world_coords(vehicle_id, points)
                all_points.append(world_points)
                vehicle_ids.extend([vehicle_id] * len(world_points))
        
        if not all_points:
            return None
            
        # Combine all point clouds
        combined = np.concatenate(all_points)
        
        # Prepare data for JSON serialization
        data = {
            'num_points': int(len(combined)),
            'points': {
                'x': combined['x'].astype(float).tolist(),
                'y': combined['y'].astype(float).tolist(),
                'z': combined['z'].astype(float).tolist(),
                'tag': combined['object_tag'].astype(int).tolist(),
            },
            'vehicle_ids': vehicle_ids,
            'num_vehicles': len(self.vehicles)
        }
        
        return data
    
    def cleanup(self):
        """Cleanup sensors - MUST be called before destroying vehicles."""
        logger.info(f"Cleaning up {len(self.lidar_sensors)} LiDAR sensors...")
        
        # First, stop all sensors to prevent callbacks
        for vehicle_id, sensor in list(self.lidar_sensors.items()):
            try:
                if sensor is not None:
                    sensor.stop()  # Stop listening before destroy
                    logger.debug(f"Stopped sensor for vehicle {vehicle_id}")
            except Exception as e:
                logger.warning(f"Error stopping sensor {vehicle_id}: {e}")
        
        # Small delay to ensure callbacks finish
        import time
        time.sleep(0.1)
        
        # Then destroy sensors
        for vehicle_id, sensor in list(self.lidar_sensors.items()):
            try:
                if sensor is not None:
                    sensor.destroy()
                    logger.debug(f"Destroyed sensor for vehicle {vehicle_id}")
            except Exception as e:
                logger.warning(f"Error destroying sensor {vehicle_id}: {e}")
        
        self.lidar_sensors.clear()
        self.latest_data.clear()
        self.vehicle_transforms.clear()
        self.vehicles.clear()
        self.actor_ids.clear()
        
        logger.info("✓ All LiDAR sensors cleaned up")
