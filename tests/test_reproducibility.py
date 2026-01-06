#!/usr/bin/env python3
"""
Reproducibility Test for CARLA Scenarios
Verifies that scenarios with the same seed produce identical results.

This test runs the scenario multiple times with the same seed and compares:
- Vehicle positions at specific frames
- Collision events
- Sensor data checksums
"""

import carla
import random
import numpy as np
import queue
import hashlib
import json
from pathlib import Path
import argparse
import time


class ReproducibilityTest:
    def __init__(self, host='192.168.1.110', port=2000):
        self.host = host
        self.port = port
        self.client = None
        self.world = None
        
    def connect(self):
        """Connect to CARLA server."""
        print(f"🔄 Connecting to CARLA server at {self.host}:{self.port}")
        self.client = carla.Client(self.host, self.port)
        self.client.set_timeout(30.0)
        self.world = self.client.get_world()
        print(f"✓ Connected! Map: {self.world.get_map().name}\n")
        
    def run_scenario_and_collect_data(self, seed=42, duration=20, sample_frames=None):
        """Run a scenario and collect data for comparison."""
        print(f"Running scenario with seed={seed}, duration={duration}s...")
        
        actor_list = []
        sensor_data = {
            'positions': [],
            'collisions': [],
            'imu_data': [],
            'frame_checksums': []
        }
        
        if sample_frames is None:
            # Sample at these specific frames
            sample_frames = [20, 50, 100, 200, 300, 380]
        
        try:
            # Setup world
            settings = self.world.get_settings()
            settings.synchronous_mode = True
            settings.fixed_delta_seconds = 0.05  # 20 FPS
            self.world.apply_settings(settings)
            
            # Set seed
            random.seed(seed)
            np.random.seed(seed)
            
            # Fixed weather
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
            
            # Spawn ego vehicle
            blueprint_library = self.world.get_blueprint_library()
            vehicle_bp = blueprint_library.filter('vehicle.tesla.model3')[0]
            spawn_points = self.world.get_map().get_spawn_points()
            ego_vehicle = self.world.spawn_actor(vehicle_bp, spawn_points[0])
            actor_list.append(ego_vehicle)
            
            # Attach collision sensor
            collision_bp = blueprint_library.find('sensor.other.collision')
            collision_sensor = self.world.spawn_actor(collision_bp, carla.Transform(), attach_to=ego_vehicle)
            actor_list.append(collision_sensor)
            collision_sensor.listen(lambda event: sensor_data['collisions'].append({
                'frame': event.frame,
                'other_actor': event.other_actor.type_id if event.other_actor else None
            }))
            
            # Attach IMU
            imu_bp = blueprint_library.find('sensor.other.imu')
            imu_sensor = self.world.spawn_actor(imu_bp, carla.Transform(), attach_to=ego_vehicle)
            actor_list.append(imu_sensor)
            imu_queue = queue.Queue()
            imu_sensor.listen(lambda data: imu_queue.put(data))
            
            # Spawn traffic
            traffic_manager = self.client.get_trafficmanager(8000)
            traffic_manager.set_synchronous_mode(True)
            traffic_manager.set_random_device_seed(seed)
            
            spawn_points_shuffled = spawn_points.copy()
            random.shuffle(spawn_points_shuffled)
            
            vehicle_bps = blueprint_library.filter('vehicle.*')
            vehicle_bps = [x for x in vehicle_bps if int(x.get_attribute('number_of_wheels')) == 4]
            
            # Spawn 20 vehicles
            vehicles_spawned = 0
            for i, spawn_point in enumerate(spawn_points_shuffled[:21]):
                if i == 0:
                    continue
                    
                vehicle_bp_traffic = random.choice(vehicle_bps)
                
                try:
                    vehicle = self.world.spawn_actor(vehicle_bp_traffic, spawn_point)
                    vehicle.set_autopilot(True, 8000)
                    actor_list.append(vehicle)
                    vehicles_spawned += 1
                except:
                    pass
            
            print(f"  Spawned ego vehicle + {vehicles_spawned} traffic vehicles")
            
            # Enable autopilot
            ego_vehicle.set_autopilot(True, 8000)
            
            # Wait for initialization
            for _ in range(40):
                self.world.tick()
            
            # Run scenario and collect data
            frame = 0
            max_frames = int(duration * 20)  # 20 FPS
            
            print(f"  Collecting data at frames: {sample_frames}")
            
            while frame < max_frames:
                self.world.tick()
                frame += 1
                
                # Collect data at specific frames
                if frame in sample_frames:
                    transform = ego_vehicle.get_transform()
                    velocity = ego_vehicle.get_velocity()
                    
                    # Get IMU data if available
                    imu_accel = None
                    if not imu_queue.empty():
                        imu = imu_queue.get_nowait()
                        imu_accel = [imu.accelerometer.x, imu.accelerometer.y, imu.accelerometer.z]
                        sensor_data['imu_data'].append({
                            'frame': frame,
                            'accel': imu_accel
                        })
                    
                    # Clear queue
                    while not imu_queue.empty():
                        imu_queue.get_nowait()
                    
                    position_data = {
                        'frame': frame,
                        'location': {
                            'x': round(transform.location.x, 3),
                            'y': round(transform.location.y, 3),
                            'z': round(transform.location.z, 3)
                        },
                        'rotation': {
                            'pitch': round(transform.rotation.pitch, 3),
                            'yaw': round(transform.rotation.yaw, 3),
                            'roll': round(transform.rotation.roll, 3)
                        },
                        'velocity': {
                            'x': round(velocity.x, 3),
                            'y': round(velocity.y, 3),
                            'z': round(velocity.z, 3)
                        }
                    }
                    sensor_data['positions'].append(position_data)
                    
                    # Create checksum of position data
                    checksum = hashlib.md5(json.dumps(position_data, sort_keys=True).encode()).hexdigest()
                    sensor_data['frame_checksums'].append({
                        'frame': frame,
                        'checksum': checksum
                    })
                    
                if frame % 100 == 0:
                    print(f"    Frame {frame}/{max_frames}")
            
            print(f"  ✓ Scenario completed - {len(sensor_data['positions'])} samples collected")
            print(f"  ✓ Collisions: {len(sensor_data['collisions'])}")
            
        finally:
            # Cleanup
            self.client.apply_batch([carla.command.DestroyActor(x) for x in actor_list])
            
            settings = self.world.get_settings()
            settings.synchronous_mode = False
            settings.fixed_delta_seconds = None
            self.world.apply_settings(settings)
            
        return sensor_data
        
    def compare_data(self, data1, data2, run1_name="Run 1", run2_name="Run 2"):
        """Compare two data sets for reproducibility."""
        print(f"\n{'='*60}")
        print(f"COMPARING: {run1_name} vs {run2_name}")
        print(f"{'='*60}\n")
        
        all_match = True
        
        # Compare positions
        print("📍 Comparing vehicle positions...")
        if len(data1['positions']) != len(data2['positions']):
            print(f"  ❌ Different number of position samples: {len(data1['positions'])} vs {len(data2['positions'])}")
            all_match = False
        else:
            position_matches = 0
            for i, (pos1, pos2) in enumerate(zip(data1['positions'], data2['positions'])):
                if pos1 == pos2:
                    position_matches += 1
                else:
                    print(f"  ❌ Mismatch at frame {pos1['frame']}:")
                    print(f"     {run1_name}: {pos1['location']}")
                    print(f"     {run2_name}: {pos2['location']}")
                    all_match = False
                    
            if position_matches == len(data1['positions']):
                print(f"  ✅ All {position_matches} position samples match perfectly!")
            else:
                print(f"  ⚠️  {position_matches}/{len(data1['positions'])} positions match")
        
        # Compare checksums
        print("\n🔐 Comparing frame checksums...")
        if len(data1['frame_checksums']) != len(data2['frame_checksums']):
            print(f"  ❌ Different number of checksums")
            all_match = False
        else:
            checksum_matches = sum(1 for c1, c2 in zip(data1['frame_checksums'], data2['frame_checksums']) 
                                  if c1['checksum'] == c2['checksum'])
            if checksum_matches == len(data1['frame_checksums']):
                print(f"  ✅ All {checksum_matches} checksums match perfectly!")
            else:
                print(f"  ❌ Only {checksum_matches}/{len(data1['frame_checksums'])} checksums match")
                all_match = False
        
        # Compare collisions
        print("\n💥 Comparing collisions...")
        if len(data1['collisions']) != len(data2['collisions']):
            print(f"  ❌ Different number of collisions: {len(data1['collisions'])} vs {len(data2['collisions'])}")
            all_match = False
        else:
            if len(data1['collisions']) == 0:
                print(f"  ✅ No collisions in either run (both clean)")
            else:
                collision_matches = sum(1 for c1, c2 in zip(data1['collisions'], data2['collisions']) 
                                       if c1['frame'] == c2['frame'])
                if collision_matches == len(data1['collisions']):
                    print(f"  ✅ All {collision_matches} collisions match!")
                else:
                    print(f"  ❌ Only {collision_matches}/{len(data1['collisions'])} collisions match")
                    all_match = False
        
        # Compare IMU data
        print("\n📊 Comparing IMU data...")
        if len(data1['imu_data']) != len(data2['imu_data']):
            print(f"  ⚠️  Different number of IMU samples (non-critical)")
        elif len(data1['imu_data']) > 0:
            imu_matches = sum(1 for i1, i2 in zip(data1['imu_data'], data2['imu_data']) 
                             if i1['frame'] == i2['frame'])
            print(f"  ✅ {imu_matches}/{len(data1['imu_data'])} IMU samples at same frames")
        
        # Final verdict
        print(f"\n{'='*60}")
        if all_match:
            print("✅ REPRODUCIBILITY TEST PASSED!")
            print("Scenarios are perfectly reproducible with the same seed.")
        else:
            print("❌ REPRODUCIBILITY TEST FAILED!")
            print("Scenarios differ - check seed handling or non-deterministic code.")
        print(f"{'='*60}\n")
        
        return all_match


def main():
    parser = argparse.ArgumentParser(description='Test CARLA scenario reproducibility')
    parser.add_argument('--host', default='192.168.1.110', help='CARLA server host')
    parser.add_argument('--port', type=int, default=2000, help='CARLA server port')
    parser.add_argument('--seed', type=int, default=42, help='Random seed for both runs')
    parser.add_argument('--duration', type=int, default=20, help='Test duration in seconds')
    parser.add_argument('--runs', type=int, default=2, help='Number of test runs')
    
    args = parser.parse_args()
    
    test = ReproducibilityTest(host=args.host, port=args.port)
    
    try:
        test.connect()
        
        # Define sample frames
        sample_frames = [20, 50, 100, 150, 200, 250, 300, 350, 380]
        
        # Run multiple scenarios with same seed
        print(f"Running {args.runs} scenarios with seed={args.seed}\n")
        
        results = []
        for i in range(args.runs):
            print(f"\n{'='*60}")
            print(f"RUN {i+1}/{args.runs}")
            print(f"{'='*60}")
            
            data = test.run_scenario_and_collect_data(
                seed=args.seed, 
                duration=args.duration,
                sample_frames=sample_frames
            )
            results.append(data)
            
            # Wait a bit between runs
            if i < args.runs - 1:
                print("\nWaiting 3 seconds before next run...")
                time.sleep(3)
        
        # Compare all consecutive pairs
        all_tests_passed = True
        for i in range(len(results) - 1):
            passed = test.compare_data(
                results[i], 
                results[i+1], 
                run1_name=f"Run {i+1}",
                run2_name=f"Run {i+2}"
            )
            all_tests_passed = all_tests_passed and passed
        
        # Final summary
        print("\n" + "="*60)
        print("FINAL SUMMARY")
        print("="*60)
        if all_tests_passed:
            print("✅ ALL REPRODUCIBILITY TESTS PASSED!")
            print(f"All {args.runs} runs produced identical results.")
        else:
            print("❌ SOME TESTS FAILED")
            print("Scenarios are not perfectly reproducible.")
        print("="*60)
        
    except Exception as e:
        print(f"\n❌ Error: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main()
