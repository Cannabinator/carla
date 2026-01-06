#!/usr/bin/env python3
"""
Data Collection Extension for Scientific Scenario
Saves sensor data to disk for analysis
"""

import carla
import numpy as np
import os
from pathlib import Path
from datetime import datetime

class DataCollector:
    """Handles saving sensor data to disk."""
    
    def __init__(self, output_dir='./data'):
        """Initialize data collector with output directory."""
        self.output_dir = Path(output_dir)
        self.frame_count = 0
        
        # Create timestamped run directory
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.run_dir = self.output_dir / f'run_{timestamp}'
        self.run_dir.mkdir(parents=True, exist_ok=True)
        
        # Create subdirectories for different sensors
        (self.run_dir / 'rgb').mkdir(exist_ok=True)
        (self.run_dir / 'semantic').mkdir(exist_ok=True)
        (self.run_dir / 'lidar').mkdir(exist_ok=True)
        (self.run_dir / 'logs').mkdir(exist_ok=True)
        
        print(f"📁 Data will be saved to: {self.run_dir}")
        
        # Log file for metadata
        self.log_file = open(self.run_dir / 'logs' / 'metadata.csv', 'w')
        self.log_file.write('frame,timestamp,location_x,location_y,location_z,rotation_pitch,rotation_yaw,rotation_roll,velocity_x,velocity_y,velocity_z\n')
        
    def save_rgb_image(self, image, frame):
        """Save RGB camera image."""
        array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
        array = np.reshape(array, (image.height, image.width, 4))
        array = array[:, :, :3]  # Remove alpha channel
        
        filename = self.run_dir / 'rgb' / f'{frame:06d}.npy'
        np.save(filename, array)
        
    def save_semantic_image(self, image, frame):
        """Save semantic segmentation image."""
        array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
        array = np.reshape(array, (image.height, image.width, 4))
        array = array[:, :, 2]  # Red channel contains labels
        
        filename = self.run_dir / 'semantic' / f'{frame:06d}.npy'
        np.save(filename, array)
        
    def save_lidar_data(self, lidar, frame):
        """Save LIDAR point cloud."""
        points = np.frombuffer(lidar.raw_data, dtype=np.dtype('f4'))
        points = np.reshape(points, (int(points.shape[0] / 4), 4))
        
        filename = self.run_dir / 'lidar' / f'{frame:06d}.npy'
        np.save(filename, points)
        
    def log_vehicle_state(self, vehicle, frame, timestamp):
        """Log vehicle state (position, rotation, velocity)."""
        transform = vehicle.get_transform()
        velocity = vehicle.get_velocity()
        
        self.log_file.write(f'{frame},{timestamp:.3f},'
                          f'{transform.location.x:.3f},{transform.location.y:.3f},{transform.location.z:.3f},'
                          f'{transform.rotation.pitch:.3f},{transform.rotation.yaw:.3f},{transform.rotation.roll:.3f},'
                          f'{velocity.x:.3f},{velocity.y:.3f},{velocity.z:.3f}\n')
        
    def close(self):
        """Close log files."""
        self.log_file.close()
        print(f"✓ Data saved to: {self.run_dir}")
        print(f"✓ Total frames collected: {self.frame_count}")


# Example usage callback functions
def save_rgb_callback(image, data_collector, frame):
    """Callback to save RGB images."""
    data_collector.save_rgb_image(image, frame)
    
def save_semantic_callback(image, data_collector, frame):
    """Callback to save semantic segmentation."""
    data_collector.save_semantic_image(image, frame)
    
def save_lidar_callback(lidar, data_collector, frame):
    """Callback to save LIDAR data."""
    data_collector.save_lidar_data(lidar, frame)
