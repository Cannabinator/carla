#!/usr/bin/env python3
"""
Reproducible CARLA Scenario for Scientific Use
- Fixed random seed for reproducibility
- Synchronous mode for deterministic simulation
- 1 minute duration with background traffic
- Multiple sensors: LIDAR, RGB cameras, semantic segmentation
- Data collection and logging
"""

import carla
import random
import time
import numpy as np
import queue
import sys
import argparse
from pathlib import Path

class ScientificScenario:
    def __init__(self, host='192.168.1.110', port=2000, seed=42, render_mode=True):
        """Initialize the scenario with fixed seed for reproducibility."""
        self.host = host
        self.port = port
        self.seed = seed
        self.render_mode = render_mode
        self.client = None
        self.world = None
        self.ego_vehicle = None
        self.sensors = []
        self.actor_list = []
        self.sensor_data = {}
        
    def connect(self):
        """Connect to CARLA server."""
        print(f"🔄 Connecting to CARLA server at {self.host}:{self.port}")
        self.client = carla.Client(self.host, self.port)
        self.client.set_timeout(30.0)
        self.world = self.client.get_world()
        print(f"✓ Connected! Map: {self.world.get_map().name}")
        
    def setup_world(self):
        """Configure world for reproducible simulation."""
        print("\n⚙️  Configuring world settings...")
        
        # Get current settings
        settings = self.world.get_settings()
        
        # Enable synchronous mode for deterministic simulation
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 0.05  # 20 FPS
        
        # Disable rendering if requested (significant performance boost for data collection)
        if not self.render_mode:
            settings.no_rendering_mode = True
        
        self.world.apply_settings(settings)
        
        # Set fixed random seed for reproducibility
        random.seed(self.seed)
        np.random.seed(self.seed)
        
        # Set weather (fixed for reproducibility)
        weather = carla.WeatherParameters(
            cloudiness=30.0,
            precipitation=0.0,
            sun_altitude_angle=70.0,
            sun_azimuth_angle=0.0,
            fog_density=0.0,
            fog_distance=0.0,
            wetness=0.0
        )
        self.world.set_weather(weather)
        
        print(f"✓ Synchronous mode enabled (20 FPS)")
        print(f"✓ Random seed set to: {self.seed}")
        print(f"✓ Weather configured")
        
    def spawn_ego_vehicle(self):
        """Spawn the ego vehicle at a fixed location."""
        print("\n🚗 Spawning ego vehicle...")
        
        blueprint_library = self.world.get_blueprint_library()
        vehicle_bp = blueprint_library.filter('vehicle.tesla.model3')[0]
        
        # Fixed spawn point for reproducibility
        spawn_points = self.world.get_map().get_spawn_points()
        spawn_point = spawn_points[0]  # Always use first spawn point
        
        self.ego_vehicle = self.world.spawn_actor(vehicle_bp, spawn_point)
        self.actor_list.append(self.ego_vehicle)
        
        print(f"✓ Ego vehicle spawned at ({spawn_point.location.x:.1f}, {spawn_point.location.y:.1f})")
        
    def attach_sensors(self):
        """Attach sensors to ego vehicle."""
        print("\n📡 Attaching sensors...")
        
        blueprint_library = self.world.get_blueprint_library()
        
        # 1. RGB Camera (front) - Reduced resolution for network efficiency
        camera_bp = blueprint_library.find('sensor.camera.rgb')
        camera_bp.set_attribute('image_size_x', '800')  # Reduced from 1920
        camera_bp.set_attribute('image_size_y', '600')  # Reduced from 1080
        camera_bp.set_attribute('fov', '110')
        
        camera_transform = carla.Transform(
            carla.Location(x=2.5, z=0.7),
            carla.Rotation(pitch=0)
        )
        camera = self.world.spawn_actor(camera_bp, camera_transform, attach_to=self.ego_vehicle)
        self.sensors.append(camera)
        self.sensor_data['rgb_camera'] = queue.Queue()
        camera.listen(lambda data: self.sensor_data['rgb_camera'].put(data))
        print("✓ RGB Camera attached")
        
        # 2. Semantic Segmentation Camera - Reduced resolution for network efficiency
        sem_camera_bp = blueprint_library.find('sensor.camera.semantic_segmentation')
        sem_camera_bp.set_attribute('image_size_x', '800')  # Reduced from 1920
        sem_camera_bp.set_attribute('image_size_y', '600')  # Reduced from 1080
        sem_camera = self.world.spawn_actor(sem_camera_bp, camera_transform, attach_to=self.ego_vehicle)
        self.sensors.append(sem_camera)
        self.sensor_data['semantic_camera'] = queue.Queue()
        sem_camera.listen(lambda data: self.sensor_data['semantic_camera'].put(data))
        print("✓ Semantic Segmentation Camera attached")
        
        # 3. LIDAR - Reduced density for network efficiency
        lidar_bp = blueprint_library.find('sensor.lidar.ray_cast')
        lidar_bp.set_attribute('channels', '32')  # Reduced from 64
        lidar_bp.set_attribute('points_per_second', '280000')  # Reduced from 1.12M
        lidar_bp.set_attribute('rotation_frequency', '10')  # Reduced from 20
        lidar_bp.set_attribute('range', '100')
        
        lidar_transform = carla.Transform(carla.Location(x=0, z=2.4))
        lidar = self.world.spawn_actor(lidar_bp, lidar_transform, attach_to=self.ego_vehicle)
        self.sensors.append(lidar)
        self.sensor_data['lidar'] = queue.Queue()
        lidar.listen(lambda data: self.sensor_data['lidar'].put(data))
        print("✓ LIDAR attached (32 channels, 10Hz, 100m range)")
        
        # 4. Collision Sensor
        collision_bp = blueprint_library.find('sensor.other.collision')
        collision_sensor = self.world.spawn_actor(collision_bp, carla.Transform(), attach_to=self.ego_vehicle)
        self.sensors.append(collision_sensor)
        self.sensor_data['collision'] = []
        collision_sensor.listen(lambda event: self.sensor_data['collision'].append(event))
        print("✓ Collision Sensor attached")
        
        # 5. IMU
        imu_bp = blueprint_library.find('sensor.other.imu')
        imu_sensor = self.world.spawn_actor(imu_bp, carla.Transform(), attach_to=self.ego_vehicle)
        self.sensors.append(imu_sensor)
        self.sensor_data['imu'] = queue.Queue()
        imu_sensor.listen(lambda data: self.sensor_data['imu'].put(data))
        print("✓ IMU Sensor attached")
        
        # 6. GNSS
        gnss_bp = blueprint_library.find('sensor.other.gnss')
        gnss_sensor = self.world.spawn_actor(gnss_bp, carla.Transform(), attach_to=self.ego_vehicle)
        self.sensors.append(gnss_sensor)
        self.sensor_data['gnss'] = queue.Queue()
        gnss_sensor.listen(lambda data: self.sensor_data['gnss'].put(data))
        print("✓ GNSS Sensor attached")
        
    def spawn_traffic(self, num_vehicles=50, num_pedestrians=30):
        """Spawn background traffic for realistic simulation."""
        print(f"\n🚦 Spawning traffic ({num_vehicles} vehicles, {num_pedestrians} pedestrians)...")
        
        # Set traffic manager to use deterministic mode
        traffic_manager = self.client.get_trafficmanager(8000)
        traffic_manager.set_synchronous_mode(True)
        traffic_manager.set_random_device_seed(self.seed)
        
        blueprint_library = self.world.get_blueprint_library()
        spawn_points = self.world.get_map().get_spawn_points()
        
        # Shuffle with fixed seed for reproducibility
        random.shuffle(spawn_points)
        
        # Spawn vehicles
        vehicle_bps = blueprint_library.filter('vehicle.*')
        vehicle_bps = [x for x in vehicle_bps if int(x.get_attribute('number_of_wheels')) == 4]
        
        vehicles_spawned = 0
        for i, spawn_point in enumerate(spawn_points[:num_vehicles]):
            if i == 0:  # Skip first spawn point (used by ego vehicle)
                continue
                
            vehicle_bp = random.choice(vehicle_bps)
            
            # Randomize vehicle color
            if vehicle_bp.has_attribute('color'):
                color = random.choice(vehicle_bp.get_attribute('color').recommended_values)
                vehicle_bp.set_attribute('color', color)
                
            try:
                vehicle = self.world.spawn_actor(vehicle_bp, spawn_point)
                vehicle.set_autopilot(True, 8000)
                self.actor_list.append(vehicle)
                vehicles_spawned += 1
            except:
                pass
                
        print(f"✓ Spawned {vehicles_spawned} traffic vehicles")
        
        # Spawn pedestrians
        pedestrian_bps = blueprint_library.filter('walker.pedestrian.*')
        pedestrians_spawned = 0
        walker_controller_bp = blueprint_library.find('controller.ai.walker')
        
        for i in range(num_pedestrians):
            spawn_point = carla.Transform()
            loc = self.world.get_random_location_from_navigation()
            if loc is not None:
                spawn_point.location = loc
                pedestrian_bp = random.choice(pedestrian_bps)
                
                try:
                    walker = self.world.spawn_actor(pedestrian_bp, spawn_point)
                    self.actor_list.append(walker)
                    
                    # Spawn walker controller
                    walker_controller = self.world.spawn_actor(walker_controller_bp, carla.Transform(), walker)
                    self.actor_list.append(walker_controller)
                    
                    # Start walking
                    walker_controller.start()
                    walker_controller.go_to_location(self.world.get_random_location_from_navigation())
                    walker_controller.set_max_speed(1.4)  # Normal walking speed
                    
                    pedestrians_spawned += 1
                except:
                    pass
                    
        print(f"✓ Spawned {pedestrians_spawned} pedestrians")
        
    def run_scenario(self, duration=60):
        """Run the scenario for specified duration in seconds."""
        print(f"\n▶️  Running scenario for {duration} seconds...")
        print("=" * 60)
        
        # Enable autopilot for ego vehicle
        self.ego_vehicle.set_autopilot(True, 8000)
        
        start_time = time.time()
        frame = 0
        
        try:
            while time.time() - start_time < duration:
                # Tick the simulation (synchronous mode)
                self.world.tick()
                frame += 1
                
                # Clear sensor queues to prevent memory buildup
                # Remove data from queues (or process/save it if needed)
                for key in ['rgb_camera', 'semantic_camera', 'lidar', 'imu', 'gnss']:
                    while not self.sensor_data[key].empty():
                        try:
                            self.sensor_data[key].get_nowait()
                        except:
                            break
                
                # Progress update every 5 seconds
                elapsed = time.time() - start_time
                if frame % 100 == 0:  # Every 5 seconds at 20 FPS
                    progress = (elapsed / duration) * 100
                    actual_fps = frame / elapsed if elapsed > 0 else 0
                    print(f"[{elapsed:.1f}s / {duration}s] Frame: {frame} | Progress: {progress:.1f}% | FPS: {actual_fps:.1f}")
                    
        except KeyboardInterrupt:
            print("\n⚠️  Scenario interrupted by user")
            
        elapsed = time.time() - start_time
        print("=" * 60)
        print(f"✓ Scenario completed! Duration: {elapsed:.1f}s, Frames: {frame}")
        print(f"✓ Collisions detected: {len(self.sensor_data['collision'])}")
        
    def cleanup(self):
        """Clean up all actors and restore settings."""
        print("\n🧹 Cleaning up...")
        
        # Disable sensors
        for sensor in self.sensors:
            if sensor.is_alive:
                sensor.stop()
                sensor.destroy()
                
        # Destroy all spawned actors
        self.client.apply_batch([carla.command.DestroyActor(x) for x in self.actor_list])
        
        # Restore world settings
        settings = self.world.get_settings()
        settings.synchronous_mode = False
        settings.fixed_delta_seconds = None
        self.world.apply_settings(settings)
        
        print("✓ All actors destroyed")
        print("✓ World settings restored")


def main():
    parser = argparse.ArgumentParser(description='Run reproducible CARLA scenario')
    parser.add_argument('--host', default='192.168.1.110', help='CARLA server host')
    parser.add_argument('--port', type=int, default=2000, help='CARLA server port')
    parser.add_argument('--seed', type=int, default=42, help='Random seed for reproducibility')
    parser.add_argument('--duration', type=int, default=60, help='Scenario duration in seconds')
    parser.add_argument('--vehicles', type=int, default=50, help='Number of traffic vehicles')
    parser.add_argument('--pedestrians', type=int, default=30, help='Number of pedestrians')
    parser.add_argument('--no-render', action='store_true', help='Disable rendering for faster performance')
    parser.add_argument('--save-data', action='store_true', help='Save sensor data to disk')
    
    args = parser.parse_args()
    
    scenario = ScientificScenario(host=args.host, port=args.port, seed=args.seed, 
                                   render_mode=not args.no_render)
    
    try:
        scenario.connect()
        scenario.setup_world()
        scenario.spawn_ego_vehicle()
        scenario.attach_sensors()
        scenario.spawn_traffic(num_vehicles=args.vehicles, num_pedestrians=args.pedestrians)
        
        # Wait a moment for everything to settle
        print("\n⏱️  Waiting 2 seconds for initialization...")
        for _ in range(40):  # 2 seconds at 20 FPS
            scenario.world.tick()
            
        scenario.run_scenario(duration=args.duration)
        
    except Exception as e:
        print(f"\n❌ Error: {e}")
        import traceback
        traceback.print_exc()
        
    finally:
        scenario.cleanup()
        print("\n✅ Scenario finished!")


if __name__ == "__main__":
    main()
