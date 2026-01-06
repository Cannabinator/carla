#!/usr/bin/env python3
"""
Lightweight CARLA Scenario - Optimized for Network/Remote Use
- Minimal sensor data transmission
- Lower resolution cameras
- Reduced LIDAR density
- Optional sensor disabling
"""

import carla
import random
import time
import numpy as np
import queue
import sys
import argparse

class LightweightScenario:
    def __init__(self, host='192.168.1.110', port=2000, seed=42, 
                 enable_cameras=False, enable_lidar=False):
        """Initialize lightweight scenario."""
        self.host = host
        self.port = port
        self.seed = seed
        self.enable_cameras = enable_cameras
        self.enable_lidar = enable_lidar
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
        
        settings = self.world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 0.05  # 20 FPS
        self.world.apply_settings(settings)
        
        random.seed(self.seed)
        np.random.seed(self.seed)
        
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
        
    def spawn_ego_vehicle(self):
        """Spawn the ego vehicle at a fixed location."""
        print("\n🚗 Spawning ego vehicle...")
        
        blueprint_library = self.world.get_blueprint_library()
        vehicle_bp = blueprint_library.filter('vehicle.tesla.model3')[0]
        
        spawn_points = self.world.get_map().get_spawn_points()
        spawn_point = spawn_points[0]
        
        self.ego_vehicle = self.world.spawn_actor(vehicle_bp, spawn_point)
        self.actor_list.append(self.ego_vehicle)
        
        print(f"✓ Ego vehicle spawned")
        
    def setup_spectator_camera(self):
        """Setup spectator camera to follow ego vehicle."""
        print("\n📹 Setting up spectator camera to follow ego vehicle...")
        
        spectator = self.world.get_spectator()
        
        # Position camera behind and above the vehicle
        def update_spectator():
            transform = self.ego_vehicle.get_transform()
            spectator_transform = carla.Transform(
                transform.location + carla.Location(x=-8, z=4),
                carla.Rotation(pitch=-15, yaw=transform.rotation.yaw)
            )
            spectator.set_transform(spectator_transform)
            
        # Initial position
        update_spectator()
        print("✓ Spectator camera following ego vehicle")
        
        return update_spectator
        
    def attach_sensors(self):
        """Attach minimal sensors to ego vehicle."""
        print("\n📡 Attaching sensors...")
        
        blueprint_library = self.world.get_blueprint_library()
        sensor_count = 0
        
        # Only attach cameras if enabled
        if self.enable_cameras:
            # Small RGB Camera
            camera_bp = blueprint_library.find('sensor.camera.rgb')
            camera_bp.set_attribute('image_size_x', '640')
            camera_bp.set_attribute('image_size_y', '480')
            camera_bp.set_attribute('fov', '90')
            
            camera_transform = carla.Transform(
                carla.Location(x=2.5, z=0.7),
                carla.Rotation(pitch=0)
            )
            camera = self.world.spawn_actor(camera_bp, camera_transform, attach_to=self.ego_vehicle)
            self.sensors.append(camera)
            self.sensor_data['camera'] = queue.Queue()
            camera.listen(lambda data: self.sensor_data['camera'].put(data))
            print("✓ RGB Camera (640x480)")
            sensor_count += 1
        
        # Only attach LIDAR if enabled
        if self.enable_lidar:
            lidar_bp = blueprint_library.find('sensor.lidar.ray_cast')
            lidar_bp.set_attribute('channels', '16')  # Very low
            lidar_bp.set_attribute('points_per_second', '100000')  # Very low
            lidar_bp.set_attribute('rotation_frequency', '10')
            lidar_bp.set_attribute('range', '50')  # Shorter range
            
            lidar_transform = carla.Transform(carla.Location(x=0, z=2.4))
            lidar = self.world.spawn_actor(lidar_bp, lidar_transform, attach_to=self.ego_vehicle)
            self.sensors.append(lidar)
            self.sensor_data['lidar'] = queue.Queue()
            lidar.listen(lambda data: self.sensor_data['lidar'].put(data))
            print("✓ LIDAR (16 channels, 10Hz)")
            sensor_count += 1
        
        # Always add collision sensor (minimal data)
        collision_bp = blueprint_library.find('sensor.other.collision')
        collision_sensor = self.world.spawn_actor(collision_bp, carla.Transform(), attach_to=self.ego_vehicle)
        self.sensors.append(collision_sensor)
        self.sensor_data['collision'] = []
        collision_sensor.listen(lambda event: self.sensor_data['collision'].append(event))
        print("✓ Collision Sensor")
        sensor_count += 1
        
        # IMU (minimal data)
        imu_bp = blueprint_library.find('sensor.other.imu')
        imu_sensor = self.world.spawn_actor(imu_bp, carla.Transform(), attach_to=self.ego_vehicle)
        self.sensors.append(imu_sensor)
        self.sensor_data['imu'] = queue.Queue()
        imu_sensor.listen(lambda data: self.sensor_data['imu'].put(data))
        print("✓ IMU Sensor")
        sensor_count += 1
        
        if sensor_count == 2:
            print(f"⚠️  Running with minimal sensors (no cameras/LIDAR) for best network performance")
        
    def spawn_traffic(self, num_vehicles=30, num_pedestrians=20):
        """Spawn background traffic."""
        print(f"\n🚦 Spawning traffic ({num_vehicles} vehicles, {num_pedestrians} pedestrians)...")
        
        traffic_manager = self.client.get_trafficmanager(8000)
        traffic_manager.set_synchronous_mode(True)
        traffic_manager.set_random_device_seed(self.seed)
        
        blueprint_library = self.world.get_blueprint_library()
        spawn_points = self.world.get_map().get_spawn_points()
        random.shuffle(spawn_points)
        
        vehicle_bps = blueprint_library.filter('vehicle.*')
        vehicle_bps = [x for x in vehicle_bps if int(x.get_attribute('number_of_wheels')) == 4]
        
        vehicles_spawned = 0
        for i, spawn_point in enumerate(spawn_points[:num_vehicles]):
            if i == 0:
                continue
                
            vehicle_bp = random.choice(vehicle_bps)
            
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
                    
                    walker_controller = self.world.spawn_actor(walker_controller_bp, carla.Transform(), walker)
                    self.actor_list.append(walker_controller)
                    
                    walker_controller.start()
                    walker_controller.go_to_location(self.world.get_random_location_from_navigation())
                    walker_controller.set_max_speed(1.4)
                    
                    pedestrians_spawned += 1
                except:
                    pass
                    
        print(f"✓ Spawned {pedestrians_spawned} pedestrians")
        
    def run_scenario(self, duration=60, follow_camera=True):
        """Run the scenario for specified duration in seconds."""
        print(f"\n▶️  Running scenario for {duration} seconds...")
        print("=" * 60)
        
        self.ego_vehicle.set_autopilot(True, 8000)
        
        # Setup spectator camera if requested
        update_spectator = None
        if follow_camera:
            update_spectator = self.setup_spectator_camera()
        
        start_time = time.time()
        frame = 0
        
        try:
            while time.time() - start_time < duration:
                self.world.tick()
                frame += 1
                
                # Update spectator camera to follow vehicle
                if update_spectator and frame % 2 == 0:  # Update every other frame
                    update_spectator()
                
                # Clear sensor queues to prevent memory buildup
                for key in self.sensor_data:
                    if isinstance(self.sensor_data[key], queue.Queue):
                        while not self.sensor_data[key].empty():
                            try:
                                self.sensor_data[key].get_nowait()
                            except:
                                break
                
                elapsed = time.time() - start_time
                if frame % 100 == 0:
                    progress = (elapsed / duration) * 100
                    actual_fps = frame / elapsed if elapsed > 0 else 0
                    print(f"[{elapsed:.1f}s / {duration}s] Frame: {frame} | Progress: {progress:.1f}% | FPS: {actual_fps:.1f}")
                    
        except KeyboardInterrupt:
            print("\n⚠️  Scenario interrupted by user")
            
        elapsed = time.time() - start_time
        print("=" * 60)
        print(f"✓ Scenario completed! Duration: {elapsed:.1f}s, Frames: {frame}")
        print(f"✓ Average FPS: {frame/elapsed:.1f}")
        print(f"✓ Collisions detected: {len(self.sensor_data['collision'])}")
        
    def cleanup(self):
        """Clean up all actors and restore settings."""
        print("\n🧹 Cleaning up...")
        
        for sensor in self.sensors:
            if sensor.is_alive:
                sensor.stop()
                sensor.destroy()
                
        self.client.apply_batch([carla.command.DestroyActor(x) for x in self.actor_list])
        
        settings = self.world.get_settings()
        settings.synchronous_mode = False
        settings.fixed_delta_seconds = None
        self.world.apply_settings(settings)
        
        print("✓ Cleanup complete")


def main():
    parser = argparse.ArgumentParser(description='Run lightweight CARLA scenario (network optimized)')
    parser.add_argument('--host', default='192.168.1.110', help='CARLA server host')
    parser.add_argument('--port', type=int, default=2000, help='CARLA server port')
    parser.add_argument('--seed', type=int, default=42, help='Random seed for reproducibility')
    parser.add_argument('--duration', type=int, default=60, help='Scenario duration in seconds')
    parser.add_argument('--vehicles', type=int, default=30, help='Number of traffic vehicles')
    parser.add_argument('--pedestrians', type=int, default=20, help='Number of pedestrians')
    parser.add_argument('--enable-cameras', action='store_true', help='Enable cameras (increases network load)')
    parser.add_argument('--enable-lidar', action='store_true', help='Enable LIDAR (increases network load)')
    parser.add_argument('--no-follow', action='store_true', help='Disable spectator camera following ego vehicle')
    
    args = parser.parse_args()
    
    scenario = LightweightScenario(host=args.host, port=args.port, seed=args.seed,
                                    enable_cameras=args.enable_cameras, 
                                    enable_lidar=args.enable_lidar)
    
    try:
        scenario.connect()
        scenario.setup_world()
        scenario.spawn_ego_vehicle()
        scenario.attach_sensors()
        scenario.spawn_traffic(num_vehicles=args.vehicles, num_pedestrians=args.pedestrians)
        
        print("\n⏱️  Waiting 2 seconds for initialization...")
        for _ in range(40):
            scenario.world.tick()
            
        scenario.run_scenario(duration=args.duration, follow_camera=not args.no_follow)
        
    except Exception as e:
        print(f"\n❌ Error: {e}")
        import traceback
        traceback.print_exc()
        
    finally:
        scenario.cleanup()
        print("\n✅ Scenario finished!")


if __name__ == "__main__":
    main()
