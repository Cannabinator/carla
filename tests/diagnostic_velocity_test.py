#!/usr/bin/env python3
"""
Diagnostic test to investigate CARLA velocity data retrieval.
Tests multiple methods to find which actually returns non-zero velocity.
"""

import carla
import time
import argparse
import math

def test_velocity_methods(host='192.168.1.110', port=2000):
    """Test different methods of getting velocity from CARLA."""
    
    client = None
    world = None
    vehicle = None
    
    try:
        print(f"🔄 Connecting to {host}:{port}")
        client = carla.Client(host, port)
        client.set_timeout(30.0)
        world = client.get_world()
        print(f"✓ Connected to {world.get_map().name}\n")
        
        # Enable synchronous mode
        original_settings = world.get_settings()
        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 0.05
        world.apply_settings(settings)
        print("✓ Synchronous mode enabled\n")
        
        # Spawn a vehicle
        bp_lib = world.get_blueprint_library()
        vehicle_bp = bp_lib.filter('vehicle.tesla.model3')[0]
        spawn_points = world.get_map().get_spawn_points()
        vehicle = world.spawn_actor(vehicle_bp, spawn_points[0])
        print(f"✓ Vehicle spawned at {spawn_points[0].location}\n")
        
        # Enable autopilot
        tm = client.get_trafficmanager(8000)
        tm.set_synchronous_mode(True)
        vehicle.set_autopilot(True, 8000)
        print("✓ Autopilot enabled\n")
        
        # Warmup
        print("⏱️  Warming up (100 frames)...")
        for i in range(100):
            world.tick()
        print("✓ Warmup complete\n")
        
        print("="*80)
        print("VELOCITY RETRIEVAL DIAGNOSTIC TEST")
        print("="*80)
        
        # Test for 20 frames
        for frame in range(1, 21):
            world.tick()
            
            # Get snapshot
            snapshot = world.get_snapshot()
            actor_snapshot = snapshot.find(vehicle.id)
            
            # METHOD 1: actor.get_velocity()
            vel_actor = vehicle.get_velocity()
            speed_actor = math.sqrt(vel_actor.x**2 + vel_actor.y**2 + vel_actor.z**2)
            
            # METHOD 2: snapshot.find(id).get_velocity()
            vel_snapshot = actor_snapshot.get_velocity()
            speed_snapshot = math.sqrt(vel_snapshot.x**2 + vel_snapshot.y**2 + vel_snapshot.z**2)
            
            # METHOD 3: Calculate from position change
            if frame == 1:
                last_pos = actor_snapshot.get_transform().location
                speed_calculated = 0.0
            else:
                current_pos = actor_snapshot.get_transform().location
                dx = current_pos.x - last_pos.x
                dy = current_pos.y - last_pos.y
                dz = current_pos.z - last_pos.z
                dist = math.sqrt(dx*dx + dy*dy + dz*dz)
                speed_calculated = dist / 0.05  # delta_seconds = 0.05
                last_pos = current_pos
            
            # Get other data
            transform = actor_snapshot.get_transform()
            control = vehicle.get_control()
            
            print(f"\nFrame {frame:03d}:")
            print(f"  Position:     ({transform.location.x:.2f}, {transform.location.y:.2f}, {transform.location.z:.2f})")
            print(f"  Control:      T={control.throttle:.3f} B={control.brake:.3f} S={control.steer:.3f}")
            print(f"  Method 1 (actor.get_velocity()):           vel=({vel_actor.x:.3f}, {vel_actor.y:.3f}, {vel_actor.z:.3f}) speed={speed_actor:.3f} m/s = {speed_actor*3.6:.2f} km/h")
            print(f"  Method 2 (snapshot.get_velocity()):        vel=({vel_snapshot.x:.3f}, {vel_snapshot.y:.3f}, {vel_snapshot.z:.3f}) speed={speed_snapshot:.3f} m/s = {speed_snapshot*3.6:.2f} km/h")
            print(f"  Method 3 (calculated from Δposition):      speed={speed_calculated:.3f} m/s = {speed_calculated*3.6:.2f} km/h")
            
            # Check if any method gives non-zero
            if speed_actor > 0.01:
                print(f"  ✓ Method 1 shows movement!")
            if speed_snapshot > 0.01:
                print(f"  ✓ Method 2 shows movement!")
            if speed_calculated > 0.01:
                print(f"  ✓ Method 3 shows movement!")
            
            time.sleep(0.1)
        
        print("\n" + "="*80)
        print("DIAGNOSTIC COMPLETE")
        print("="*80)
        
    except Exception as e:
        print(f"\n❌ Error: {e}")
        import traceback
        traceback.print_exc()
    
    finally:
        if vehicle:
            vehicle.destroy()
            print("\n✓ Vehicle destroyed")
        
        if world and original_settings:
            world.apply_settings(original_settings)
            print("✓ Settings restored")

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Diagnostic: Test velocity retrieval methods')
    parser.add_argument('--host', default='192.168.1.110', help='CARLA server IP')
    parser.add_argument('--port', type=int, default=2000, help='CARLA server port')
    
    args = parser.parse_args()
    test_velocity_methods(host=args.host, port=args.port)
