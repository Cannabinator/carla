#!/usr/bin/env python3
"""
Demonstration of Stuck Vehicle Detection Feature

This script shows how the stuck vehicle detection system works
without requiring a CARLA server connection.
"""

import time
import math
from typing import Dict, List, Optional


class MockVector3D:
    """Mock CARLA Vector3D for demonstration"""
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x = x
        self.y = y
        self.z = z


class MockActorSnapshot:
    """Mock CARLA ActorSnapshot for demonstration"""
    def __init__(self, actor_id, velocity):
        self.id = actor_id
        self._velocity = velocity
    
    def get_velocity(self):
        return self._velocity


class MockWorldSnapshot:
    """Mock CARLA WorldSnapshot for demonstration"""
    def __init__(self):
        self.actors = {}
    
    def add_actor(self, actor_id, velocity):
        self.actors[actor_id] = MockActorSnapshot(actor_id, velocity)
    
    def find(self, actor_id):
        return self.actors.get(actor_id)


class MockVehicle:
    """Mock CARLA Vehicle for demonstration"""
    def __init__(self, actor_id, name):
        self.id = actor_id
        self.name = name


class StuckVehicleTracker:
    """
    Simplified version of StuckVehicleTracker for demonstration
    """
    
    def __init__(self, velocity_threshold: float = 0.5, frames_threshold: int = 100):
        self.velocity_threshold = velocity_threshold
        self.frames_threshold = frames_threshold
        self.stuck_counters: Dict[int, int] = {}
        self.recovered_vehicles: List[int] = []
        
    def check_and_update(self, vehicle, snapshot) -> bool:
        actor_id = vehicle.id
        actor_snapshot = snapshot.find(actor_id)
        if actor_snapshot is None:
            return False
        
        vel = actor_snapshot.get_velocity()
        speed = math.sqrt(vel.x**2 + vel.y**2 + vel.z**2)
        
        if speed < self.velocity_threshold:
            self.stuck_counters[actor_id] = self.stuck_counters.get(actor_id, 0) + 1
            if self.stuck_counters[actor_id] >= self.frames_threshold:
                return True
        else:
            if actor_id in self.stuck_counters:
                self.stuck_counters[actor_id] = 0
        
        return False
    
    def recover_vehicle(self, vehicle) -> bool:
        actor_id = vehicle.id
        self.stuck_counters[actor_id] = 0
        self.recovered_vehicles.append(actor_id)
        print(f"   🔧 Recovered vehicle {vehicle.name} (ID: {actor_id})")
        return True
    
    def get_stats(self) -> Dict:
        currently_stuck = sum(1 for count in self.stuck_counters.values() 
                            if count > self.frames_threshold // 2)
        return {
            'total_recovered': len(self.recovered_vehicles),
            'currently_tracked': len(self.stuck_counters),
            'currently_stuck': currently_stuck,
            'velocity_threshold': self.velocity_threshold,
            'frames_threshold': self.frames_threshold
        }


def demonstrate_stuck_vehicle_detection():
    """
    Demonstrate the stuck vehicle detection feature with a simulated scenario
    """
    print("=" * 80)
    print("STUCK VEHICLE DETECTION DEMONSTRATION")
    print("=" * 80)
    print()
    
    # Initialize tracker
    print("Initializing StuckVehicleTracker...")
    tracker = StuckVehicleTracker(
        velocity_threshold=0.5,    # 0.5 m/s = 1.8 km/h
        frames_threshold=10        # 10 frames for faster demo (would be 100 in real scenario)
    )
    print(f"  Velocity threshold: {tracker.velocity_threshold} m/s")
    print(f"  Frames threshold: {tracker.frames_threshold} frames")
    print()
    
    # Create mock vehicles
    vehicles = [
        MockVehicle(100, "Vehicle A (Normal)"),
        MockVehicle(101, "Vehicle B (Crashes)"),
        MockVehicle(102, "Vehicle C (Normal)"),
    ]
    
    print("Simulating traffic scenario with 3 vehicles...")
    print()
    
    # Simulate frames
    frame = 0
    max_frames = 30
    
    while frame < max_frames:
        frame += 1
        snapshot = MockWorldSnapshot()
        
        # Vehicle A: Always moving normally
        snapshot.add_actor(100, MockVector3D(10.0, 0.0, 0.0))  # 10 m/s = 36 km/h
        
        # Vehicle B: Crashes at frame 5 and gets stuck
        if frame < 5:
            snapshot.add_actor(101, MockVector3D(12.0, 0.0, 0.0))  # 12 m/s = 43.2 km/h
        else:
            snapshot.add_actor(101, MockVector3D(0.0, 0.0, 0.0))  # 0 m/s (STUCK)
        
        # Vehicle C: Always moving normally  
        snapshot.add_actor(102, MockVector3D(8.0, 0.0, 0.0))  # 8 m/s = 28.8 km/h
        
        # Check all vehicles
        for vehicle in vehicles:
            is_stuck = tracker.check_and_update(vehicle, snapshot)
            
            if is_stuck:
                print(f"Frame {frame:3d}: ⚠️  {vehicle.name} detected as STUCK!")
                tracker.recover_vehicle(vehicle)
        
        # Print status every 5 frames
        if frame % 5 == 0:
            print(f"\nFrame {frame:3d} Status:")
            for vehicle in vehicles:
                counter = tracker.stuck_counters.get(vehicle.id, 0)
                actor_snap = snapshot.find(vehicle.id)
                if actor_snap:
                    vel = actor_snap.get_velocity()
                    speed = math.sqrt(vel.x**2 + vel.y**2 + vel.z**2)
                    speed_kmh = speed * 3.6
                    
                    status = "🟢 Moving" if speed > tracker.velocity_threshold else "🔴 Slow"
                    print(f"  {vehicle.name:20s}: {status:10s} | Speed: {speed_kmh:5.1f} km/h | Stuck Counter: {counter:2d}")
            print()
    
    # Final statistics
    print("=" * 80)
    print("FINAL STATISTICS")
    print("=" * 80)
    stats = tracker.get_stats()
    print(f"Total recovered:       {stats['total_recovered']}")
    print(f"Currently tracked:     {stats['currently_tracked']}")
    print(f"Currently stuck:       {stats['currently_stuck']}")
    print(f"Velocity threshold:    {stats['velocity_threshold']} m/s")
    print(f"Frames threshold:      {stats['frames_threshold']} frames")
    print()
    print("✅ Demonstration completed!")
    print()


if __name__ == '__main__':
    demonstrate_stuck_vehicle_detection()
