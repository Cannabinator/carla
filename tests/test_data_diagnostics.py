#!/usr/bin/env python3
"""
Comprehensive CARLA Data Handling Diagnostics
Tests synchronous mode, physics updates, and data retrieval timing.
"""

import sys
import time
import argparse
import numpy as np
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent))

try:
    import carla
except ImportError:
    print("❌ CARLA module not found. Install with: pip install carla==0.9.16")
    sys.exit(1)

from src.v2v import V2VNetwork, V2VState


class DataDiagnostics:
    """Diagnostic tests for CARLA data handling."""
    
    def __init__(self, host='192.168.1.110', port=2000):
        self.host = host
        self.port = port
        self.client = None
        self.world = None
        self.original_settings = None
        self.test_actors = []
        
    def setup(self):
        """Setup CARLA connection."""
        print(f"\n{'='*70}")
        print("🔧 TEST SETUP")
        print(f"{'='*70}")
        
        print(f"Connecting to {self.host}:{self.port}...")
        self.client = carla.Client(self.host, self.port)
        self.client.set_timeout(30.0)
        self.world = self.client.get_world()
        print(f"✓ Connected to {self.world.get_map().name}")
        
        # Save original settings
        self.original_settings = self.world.get_settings()
        
        # Configure synchronous mode
        settings = self.world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 0.05
        self.world.apply_settings(settings)
        print(f"✓ Synchronous mode: {settings.fixed_delta_seconds}s delta (20 FPS)")
        
    def cleanup(self):
        """Cleanup test actors and restore settings."""
        print(f"\n{'='*70}")
        print("🧹 CLEANUP")
        print(f"{'='*70}")
        
        if self.test_actors and self.client:
            print(f"Destroying {len(self.test_actors)} test actors...")
            self.client.apply_batch([carla.command.DestroyActor(x) for x in self.test_actors])
            print("✓ Actors destroyed")
        
        if self.world and self.original_settings:
            print("Restoring original world settings...")
            self.world.apply_settings(self.original_settings)
            print("✓ Settings restored")
    
    def test_1_basic_connection(self):
        """Test 1: Basic connection and world info."""
        print(f"\n{'='*70}")
        print("TEST 1: Basic Connection & World Info")
        print(f"{'='*70}")
        
        settings = self.world.get_settings()
        print(f"Map: {self.world.get_map().name}")
        print(f"Synchronous mode: {settings.synchronous_mode}")
        print(f"Fixed delta: {settings.fixed_delta_seconds}")
        print(f"No rendering: {settings.no_rendering_mode}")
        
        # Get initial snapshot
        snapshot = self.world.get_snapshot()
        print(f"World frame: {snapshot.frame}")
        print(f"Timestamp: {snapshot.timestamp.elapsed_seconds:.3f}s")
        
        print("✅ PASS: Connection and world info verified")
    
    def test_2_tick_progression(self):
        """Test 2: Verify world.tick() advances simulation."""
        print(f"\n{'='*70}")
        print("TEST 2: Tick Progression")
        print(f"{'='*70}")
        
        frame_before = self.world.get_snapshot().frame
        print(f"Frame before tick: {frame_before}")
        
        # Tick and measure
        start = time.time()
        self.world.tick()
        elapsed = time.time() - start
        
        frame_after = self.world.get_snapshot().frame
        print(f"Frame after tick:  {frame_after}")
        print(f"Tick latency: {elapsed*1000:.2f}ms")
        
        if frame_after == frame_before + 1:
            print("✅ PASS: Tick advances simulation by 1 frame")
        else:
            print(f"❌ FAIL: Expected frame {frame_before + 1}, got {frame_after}")
    
    def test_3_vehicle_spawn_and_physics(self):
        """Test 3: Spawn vehicle and verify physics initialization."""
        print(f"\n{'='*70}")
        print("TEST 3: Vehicle Spawn & Physics")
        print(f"{'='*70}")
        
        bp_lib = self.world.get_blueprint_library()
        vehicle_bp = bp_lib.find('vehicle.tesla.model3')
        spawn_points = self.world.get_map().get_spawn_points()
        
        if not spawn_points:
            print("❌ FAIL: No spawn points available")
            return
        
        spawn_transform = spawn_points[0]
        print(f"Spawning at: {spawn_transform.location}")
        
        vehicle = self.world.spawn_actor(vehicle_bp, spawn_transform)
        self.test_actors.append(vehicle.id)
        
        # Wait for physics to initialize
        for _ in range(5):
            self.world.tick()
        
        # Check initial state
        transform = vehicle.get_transform()
        velocity = vehicle.get_velocity()
        speed = 3.6 * np.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2)
        
        print(f"Vehicle ID: {vehicle.id}")
        print(f"Location: ({transform.location.x:.2f}, {transform.location.y:.2f}, {transform.location.z:.2f})")
        print(f"Velocity: ({velocity.x:.3f}, {velocity.y:.3f}, {velocity.z:.3f}) m/s")
        print(f"Speed: {speed:.2f} km/h")
        
        if abs(speed) < 0.1:
            print("✅ PASS: Vehicle spawned with zero velocity (as expected)")
        else:
            print(f"⚠️  WARNING: Vehicle spawned with non-zero velocity: {speed:.2f} km/h")
        
        return vehicle
    
    def test_4_autopilot_activation(self, vehicle):
        """Test 4: Traffic Manager autopilot and control application."""
        print(f"\n{'='*70}")
        print("TEST 4: Autopilot & Traffic Manager")
        print(f"{'='*70}")
        
        # Configure Traffic Manager
        tm = self.client.get_trafficmanager(8000)
        tm.set_synchronous_mode(True)
        tm.set_random_device_seed(42)
        
        print("✓ Traffic Manager configured (sync mode, seed 42)")
        
        # Enable autopilot
        vehicle.set_autopilot(True, 8000)
        print("✓ Autopilot enabled on port 8000")
        
        # Wait several ticks for TM to start controlling
        print("\nWaiting for TM to take control (20 ticks)...")
        for i in range(20):
            self.world.tick()
            
            if i % 5 == 0:
                control = vehicle.get_control()
                vel = vehicle.get_velocity()
                speed = 3.6 * np.sqrt(vel.x**2 + vel.y**2 + vel.z**2)
                print(f"  Tick {i:2d}: throttle={control.throttle:.3f}, brake={control.brake:.3f}, "
                      f"steer={control.steer:.3f}, speed={speed:.2f} km/h")
        
        # Check final state
        control = vehicle.get_control()
        vel = vehicle.get_velocity()
        speed = 3.6 * np.sqrt(vel.x**2 + vel.y**2 + vel.z**2)
        
        print(f"\nFinal state after 20 ticks:")
        print(f"  Throttle: {control.throttle:.3f}")
        print(f"  Brake: {control.brake:.3f}")
        print(f"  Speed: {speed:.2f} km/h")
        
        if control.throttle > 0 or control.brake > 0:
            print("✅ PASS: Traffic Manager is applying controls")
        else:
            print("❌ FAIL: No controls being applied by Traffic Manager")
        
        if speed > 1.0:
            print("✅ PASS: Vehicle is moving")
        else:
            print("⚠️  WARNING: Vehicle speed still low or zero")
    
    def test_5_data_retrieval_timing(self, vehicle):
        """Test 5: Data retrieval timing after tick."""
        print(f"\n{'='*70}")
        print("TEST 5: Data Retrieval Timing")
        print(f"{'='*70}")
        
        print("Testing data freshness at different intervals after tick...\n")
        
        delays = [0, 0.001, 0.005, 0.010, 0.020]
        
        for delay in delays:
            self.world.tick()
            
            if delay > 0:
                time.sleep(delay)
            
            vel = vehicle.get_velocity()
            speed = 3.6 * np.sqrt(vel.x**2 + vel.y**2 + vel.z**2)
            control = vehicle.get_control()
            transform = vehicle.get_transform()
            
            print(f"Delay {delay*1000:5.1f}ms: "
                  f"speed={speed:6.2f} km/h, "
                  f"throttle={control.throttle:.3f}, "
                  f"loc=({transform.location.x:.1f},{transform.location.y:.1f})")
        
        print("\n✅ PASS: Data retrieval timing tested")
    
    def test_6_v2v_state_extraction(self, vehicle):
        """Test 6: V2V state extraction accuracy."""
        print(f"\n{'='*70}")
        print("TEST 6: V2V State Extraction")
        print(f"{'='*70}")
        
        # Get data directly from CARLA
        self.world.tick()
        time.sleep(0.001)
        
        vel = vehicle.get_velocity()
        speed_direct = 3.6 * np.sqrt(vel.x**2 + vel.y**2 + vel.z**2)
        transform = vehicle.get_transform()
        
        # Get data via V2VState
        v2v_state = V2VState.from_vehicle(vehicle, vehicle_id=999)
        
        print(f"Direct CARLA data:")
        print(f"  Location: ({transform.location.x:.3f}, {transform.location.y:.3f}, {transform.location.z:.3f})")
        print(f"  Velocity: ({vel.x:.3f}, {vel.y:.3f}, {vel.z:.3f}) m/s")
        print(f"  Speed: {speed_direct:.3f} km/h")
        
        print(f"\nV2VState data:")
        print(f"  Location: {v2v_state.location}")
        print(f"  Velocity: {v2v_state.velocity}")
        print(f"  Speed: {v2v_state.speed:.3f} km/h")
        
        # Compare
        loc_error = abs(v2v_state.location[0] - transform.location.x)
        speed_error = abs(v2v_state.speed - speed_direct)
        
        print(f"\nErrors:")
        print(f"  Location: {loc_error:.6f}m")
        print(f"  Speed: {speed_error:.6f} km/h")
        
        if loc_error < 0.01 and speed_error < 0.01:
            print("✅ PASS: V2VState extraction is accurate")
        else:
            print("❌ FAIL: V2VState extraction has errors")
    
    def test_7_v2v_network_update(self, vehicle):
        """Test 7: V2V network update and neighbor discovery."""
        print(f"\n{'='*70}")
        print("TEST 7: V2V Network Update")
        print(f"{'='*70}")
        
        # Spawn second vehicle nearby
        bp_lib = self.world.get_blueprint_library()
        vehicle_bp = bp_lib.find('vehicle.audi.a2')
        spawn_points = self.world.get_map().get_spawn_points()
        
        vehicle2 = None
        for sp in spawn_points[1:10]:  # Try nearby spawn points
            try:
                vehicle2 = self.world.spawn_actor(vehicle_bp, sp)
                self.test_actors.append(vehicle2.id)
                print(f"✓ Spawned second vehicle at {sp.location}")
                break
            except:
                continue
        
        if not vehicle2:
            print("⚠️  WARNING: Could not spawn second vehicle, skipping neighbor test")
            return
        
        # Enable autopilot on second vehicle
        vehicle2.set_autopilot(True, 8000)
        
        # Let vehicles initialize
        for _ in range(10):
            self.world.tick()
        
        # Create V2V network
        v2v = V2VNetwork(max_range=500.0)  # Large range to ensure detection
        v2v.register(0, vehicle)
        v2v.register(1, vehicle2)
        
        # Update network
        v2v.update()
        
        # Check states
        state0 = v2v.get_state(0)
        state1 = v2v.get_state(1)
        
        print(f"\nVehicle 0 state:")
        print(f"  Location: {state0.location}")
        print(f"  Speed: {state0.speed:.2f} km/h")
        
        print(f"\nVehicle 1 state:")
        print(f"  Location: {state1.location}")
        print(f"  Speed: {state1.speed:.2f} km/h")
        
        # Check neighbors
        neighbors = v2v.get_neighbors(0)
        print(f"\nVehicle 0 neighbors: {len(neighbors)}")
        
        if len(neighbors) > 0:
            dist = np.linalg.norm(np.array(state0.location) - np.array(state1.location))
            print(f"  Distance to vehicle 1: {dist:.2f}m")
            print("✅ PASS: V2V network detects neighbors")
        else:
            print("⚠️  WARNING: No neighbors detected (may be out of range)")
    
    def test_8_extended_movement(self, vehicle):
        """Test 8: Extended movement test to verify continuous physics."""
        print(f"\n{'='*70}")
        print("TEST 8: Extended Movement (100 ticks)")
        print(f"{'='*70}")
        
        print("\nTracking vehicle movement over 5 seconds (100 ticks @ 20 FPS):\n")
        print(f"{'Tick':>5} {'Time':>6} {'Speed':>8} {'Throttle':>9} {'Brake':>7} {'Position':>30}")
        print(f"{'-'*70}")
        
        positions = []
        speeds = []
        
        for i in range(100):
            self.world.tick()
            
            if i % 10 == 0:  # Every 0.5s
                vel = vehicle.get_velocity()
                speed = 3.6 * np.sqrt(vel.x**2 + vel.y**2 + vel.z**2)
                control = vehicle.get_control()
                trans = vehicle.get_transform()
                
                positions.append((trans.location.x, trans.location.y))
                speeds.append(speed)
                
                print(f"{i:5d} {i*0.05:6.2f}s {speed:7.2f} {control.throttle:8.3f} "
                      f"{control.brake:7.3f} ({trans.location.x:8.2f}, {trans.location.y:8.2f})")
        
        # Analyze movement
        print(f"\n{'='*70}")
        print("Movement Analysis:")
        print(f"{'='*70}")
        
        total_distance = 0
        for i in range(1, len(positions)):
            dx = positions[i][0] - positions[i-1][0]
            dy = positions[i][1] - positions[i-1][1]
            dist = np.sqrt(dx**2 + dy**2)
            total_distance += dist
        
        avg_speed = np.mean(speeds)
        max_speed = np.max(speeds)
        min_speed = np.min(speeds)
        
        print(f"Total distance traveled: {total_distance:.2f}m")
        print(f"Average speed: {avg_speed:.2f} km/h")
        print(f"Max speed: {max_speed:.2f} km/h")
        print(f"Min speed: {min_speed:.2f} km/h")
        
        if total_distance > 1.0:
            print("✅ PASS: Vehicle is moving continuously")
        else:
            print("❌ FAIL: Vehicle not moving (total distance < 1m)")
        
        if max_speed > 5.0:
            print("✅ PASS: Vehicle reaches measurable speed")
        else:
            print("❌ FAIL: Vehicle speed remains very low")
    
    def run_all_tests(self):
        """Run all diagnostic tests."""
        try:
            self.setup()
            
            self.test_1_basic_connection()
            self.test_2_tick_progression()
            
            vehicle = self.test_3_vehicle_spawn_and_physics()
            if not vehicle:
                print("\n❌ Cannot continue tests without vehicle")
                return
            
            self.test_4_autopilot_activation(vehicle)
            self.test_5_data_retrieval_timing(vehicle)
            self.test_6_v2v_state_extraction(vehicle)
            self.test_7_v2v_network_update(vehicle)
            self.test_8_extended_movement(vehicle)
            
            print(f"\n{'='*70}")
            print("🎉 ALL TESTS COMPLETED")
            print(f"{'='*70}")
            
        except Exception as e:
            print(f"\n❌ ERROR: {e}")
            import traceback
            traceback.print_exc()
        finally:
            self.cleanup()


def main():
    parser = argparse.ArgumentParser(description='CARLA Data Handling Diagnostics')
    parser.add_argument('--host', default='192.168.1.110', help='CARLA server IP')
    parser.add_argument('--port', type=int, default=2000, help='CARLA server port')
    
    args = parser.parse_args()
    
    diagnostics = DataDiagnostics(host=args.host, port=args.port)
    diagnostics.run_all_tests()


if __name__ == '__main__':
    main()
