#!/usr/bin/env python3
"""
Unit tests for StuckVehicleTracker.
Tests stuck vehicle detection and recovery without requiring CARLA server.
"""

import unittest
import sys
from pathlib import Path
from unittest.mock import MagicMock
import math

# Mock carla module before imports
sys.modules['carla'] = MagicMock()

# Add project root to path
project_root = Path(__file__).parent.parent
sys.path.insert(0, str(project_root))

# Import only what we need to avoid dependency issues
from src.utils.carla_utils import StuckVehicleTracker


# Mock objects for testing without CARLA
class MockVector3D:
    """Mock CARLA Vector3D"""
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x = x
        self.y = y
        self.z = z


class MockActorSnapshot:
    """Mock CARLA ActorSnapshot"""
    def __init__(self, actor_id, velocity):
        self.id = actor_id
        self._velocity = velocity
    
    def get_velocity(self):
        return self._velocity


class MockWorldSnapshot:
    """Mock CARLA WorldSnapshot"""
    def __init__(self):
        self.actors = {}
    
    def add_actor(self, actor_id, velocity):
        self.actors[actor_id] = MockActorSnapshot(actor_id, velocity)
    
    def find(self, actor_id):
        return self.actors.get(actor_id)


class MockVehicle:
    """Mock CARLA Vehicle"""
    def __init__(self, actor_id):
        self.id = actor_id
        self._velocity = MockVector3D(0, 0, 0)
        self._angular_velocity = MockVector3D(0, 0, 0)
    
    def set_target_velocity(self, velocity):
        self._velocity = velocity
    
    def set_target_angular_velocity(self, angular_velocity):
        self._angular_velocity = angular_velocity
    
    def set_transform(self, transform):
        pass


class TestStuckVehicleTracker(unittest.TestCase):
    """Test cases for StuckVehicleTracker"""
    
    def setUp(self):
        """Set up test fixtures"""
        self.tracker = StuckVehicleTracker(
            velocity_threshold=0.5,
            frames_threshold=10
        )
        self.vehicle = MockVehicle(actor_id=100)
    
    def test_initialization(self):
        """Test tracker initialization"""
        self.assertEqual(self.tracker.velocity_threshold, 0.5)
        self.assertEqual(self.tracker.frames_threshold, 10)
        self.assertEqual(len(self.tracker.stuck_counters), 0)
        self.assertEqual(len(self.tracker.recovered_vehicles), 0)
    
    def test_moving_vehicle_not_stuck(self):
        """Test that moving vehicle is not detected as stuck"""
        snapshot = MockWorldSnapshot()
        snapshot.add_actor(100, MockVector3D(5.0, 0.0, 0.0))  # 5 m/s
        
        is_stuck = self.tracker.check_and_update(self.vehicle, snapshot)
        self.assertFalse(is_stuck)
        self.assertEqual(self.tracker.stuck_counters.get(100, 0), 0)
    
    def test_slow_vehicle_increments_counter(self):
        """Test that slow vehicle increments stuck counter"""
        snapshot = MockWorldSnapshot()
        snapshot.add_actor(100, MockVector3D(0.1, 0.1, 0.0))  # 0.14 m/s (below threshold)
        
        # First check
        is_stuck = self.tracker.check_and_update(self.vehicle, snapshot)
        self.assertFalse(is_stuck)  # Not stuck yet
        self.assertEqual(self.tracker.stuck_counters[100], 1)
        
        # Second check
        is_stuck = self.tracker.check_and_update(self.vehicle, snapshot)
        self.assertFalse(is_stuck)  # Still not stuck
        self.assertEqual(self.tracker.stuck_counters[100], 2)
    
    def test_stuck_vehicle_detected_after_threshold(self):
        """Test that vehicle is detected as stuck after threshold frames"""
        snapshot = MockWorldSnapshot()
        snapshot.add_actor(100, MockVector3D(0.0, 0.0, 0.0))  # 0 m/s
        
        # Check multiple times until threshold
        for i in range(self.tracker.frames_threshold - 1):
            is_stuck = self.tracker.check_and_update(self.vehicle, snapshot)
            self.assertFalse(is_stuck, f"Should not be stuck at frame {i+1}")
        
        # Check one more time - should now be stuck
        is_stuck = self.tracker.check_and_update(self.vehicle, snapshot)
        self.assertTrue(is_stuck, "Should be stuck after threshold")
        self.assertEqual(self.tracker.stuck_counters[100], self.tracker.frames_threshold)
    
    def test_vehicle_starts_moving_resets_counter(self):
        """Test that counter resets when vehicle starts moving"""
        snapshot = MockWorldSnapshot()
        
        # Vehicle slow for a few frames
        snapshot.add_actor(100, MockVector3D(0.0, 0.0, 0.0))
        for _ in range(5):
            self.tracker.check_and_update(self.vehicle, snapshot)
        self.assertEqual(self.tracker.stuck_counters[100], 5)
        
        # Vehicle starts moving
        snapshot.add_actor(100, MockVector3D(5.0, 0.0, 0.0))
        is_stuck = self.tracker.check_and_update(self.vehicle, snapshot)
        self.assertFalse(is_stuck)
        self.assertEqual(self.tracker.stuck_counters[100], 0)
    
    def test_multiple_vehicles_tracked_independently(self):
        """Test that multiple vehicles are tracked independently"""
        snapshot = MockWorldSnapshot()
        vehicle1 = MockVehicle(actor_id=100)
        vehicle2 = MockVehicle(actor_id=200)
        
        # Vehicle 1 stuck, vehicle 2 moving
        snapshot.add_actor(100, MockVector3D(0.0, 0.0, 0.0))
        snapshot.add_actor(200, MockVector3D(5.0, 0.0, 0.0))
        
        for _ in range(5):
            stuck1 = self.tracker.check_and_update(vehicle1, snapshot)
            stuck2 = self.tracker.check_and_update(vehicle2, snapshot)
        
        self.assertEqual(self.tracker.stuck_counters[100], 5)
        self.assertEqual(self.tracker.stuck_counters[200], 0)
        self.assertFalse(stuck1)
        self.assertFalse(stuck2)
    
    def test_recovery_resets_counter(self):
        """Test that recovery resets stuck counter"""
        snapshot = MockWorldSnapshot()
        snapshot.add_actor(100, MockVector3D(0.0, 0.0, 0.0))
        
        # Make vehicle stuck
        for _ in range(self.tracker.frames_threshold):
            self.tracker.check_and_update(self.vehicle, snapshot)
        
        # Recover vehicle
        success = self.tracker.recover_vehicle(self.vehicle)
        self.assertTrue(success)
        self.assertEqual(self.tracker.stuck_counters[100], 0)
        self.assertIn(100, self.tracker.recovered_vehicles)
    
    def test_get_stats(self):
        """Test statistics retrieval"""
        snapshot = MockWorldSnapshot()
        snapshot.add_actor(100, MockVector3D(0.0, 0.0, 0.0))
        
        # Make one vehicle stuck
        for _ in range(self.tracker.frames_threshold):
            self.tracker.check_and_update(self.vehicle, snapshot)
        
        # Recover it
        self.tracker.recover_vehicle(self.vehicle)
        
        # Get stats
        stats = self.tracker.get_stats()
        self.assertEqual(stats['total_recovered'], 1)
        self.assertEqual(stats['currently_tracked'], 1)
        self.assertEqual(stats['velocity_threshold'], 0.5)
        self.assertEqual(stats['frames_threshold'], 10)
    
    def test_reset(self):
        """Test tracker reset"""
        snapshot = MockWorldSnapshot()
        snapshot.add_actor(100, MockVector3D(0.0, 0.0, 0.0))
        
        # Track some vehicles
        for _ in range(5):
            self.tracker.check_and_update(self.vehicle, snapshot)
        
        self.tracker.recover_vehicle(self.vehicle)
        
        # Reset
        self.tracker.reset()
        
        self.assertEqual(len(self.tracker.stuck_counters), 0)
        self.assertEqual(len(self.tracker.recovered_vehicles), 0)
    
    def test_missing_actor_in_snapshot(self):
        """Test handling of missing actor in snapshot"""
        snapshot = MockWorldSnapshot()
        # Don't add actor to snapshot
        
        is_stuck = self.tracker.check_and_update(self.vehicle, snapshot)
        self.assertFalse(is_stuck)
        self.assertEqual(len(self.tracker.stuck_counters), 0)


if __name__ == '__main__':
    unittest.main()
