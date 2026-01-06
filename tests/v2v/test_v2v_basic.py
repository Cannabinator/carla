#!/usr/bin/env python3
"""
Simplified V2V Network Tests
Tests core V2V functionality with mock objects.
"""

import unittest
import sys
import time

sys.path.insert(0, '/home/workstation/carla')

from src.v2v import V2VNetworkEnhanced, BSMCore


# Simple mock objects
class MockVector:
    def __init__(self, x, y, z):
        self.x, self.y, self.z = x, y, z


class MockRotation:
    def __init__(self, pitch, yaw, roll):
        self.pitch = pitch
        self.yaw = yaw
        self.roll = roll


class MockTransform:
    def __init__(self, x, y, z, yaw=0):
        self.location = MockVector(x, y, z)
        self.rotation = MockRotation(0, yaw, 0)


class MockControl:
    def __init__(self):
        self.throttle = 0.0
        self.brake = 0.0
        self.steer = 0.0
        self.reverse = False
        self.hand_brake = False
        self.manual_gear_shift = False
        self.gear = 1


class MockVehicle:
    def __init__(self, vid, x=0, y=0):
        self.id = vid
        self._transform = MockTransform(x, y, 0)
        self._velocity = MockVector(0, 0, 0)
        self._control = MockControl()
        self.bounding_box = type('obj', (object,), {
            'extent': MockVector(2.25, 0.9, 0.75)
        })()
    
    def get_transform(self):
        return self._transform
    
    def get_velocity(self):
        return self._velocity
    
    def get_control(self):
        return self._control
    
    def get_acceleration(self):
        return MockVector(0, 0, 0)
    
    def get_angular_velocity(self):
        return MockVector(0, 0, 0)


class MockWorld:
    def __init__(self):
        self.vehicles = []
    
    def get_snapshot(self):
        snapshot = type('obj', (object,), {'find': lambda vid: next((v for v in self.vehicles if v.id == vid), None)})()
        return snapshot
    
    def add_vehicle(self, v):
        self.vehicles.append(v)


class TestV2VBasics(unittest.TestCase):
    """Test basic V2V network functionality"""
    
    def test_network_creation(self):
        """Test creating V2V network"""
        v2v = V2VNetworkEnhanced(max_range=100.0, update_rate_hz=2.0)
        self.assertIsNotNone(v2v)
        self.assertEqual(v2v.max_range, 100.0)
        self.assertEqual(v2v.update_rate_hz, 2.0)
    
    def test_vehicle_registration(self):
        """Test registering vehicles"""
        world = MockWorld()
        v2v = V2VNetworkEnhanced(max_range=100.0, world=world)
        v1 = MockVehicle(1, 0, 0)
        world.add_vehicle(v1)
        
        v2v.register(1, v1)
        self.assertIn(1, v2v.vehicles)
        
        v2v.unregister(1)
        self.assertNotIn(1, v2v.vehicles)
    
    def test_bsm_creation(self):
        """Test BSM message creation"""
        world = MockWorld()
        v2v = V2VNetworkEnhanced(max_range=100.0, world=world)
        v1 = MockVehicle(1, 100, 50)
        world.add_vehicle(v1)
        
        v2v.register(1, v1)
        v2v.update(force=True)
        
        bsm = v2v.get_bsm(1)
        self.assertIsNotNone(bsm)
        self.assertEqual(bsm.vehicle_id, 1)
        self.assertEqual(bsm.latitude, 100)
        self.assertEqual(bsm.longitude, 50)
    
    def test_neighbor_discovery(self):
        """Test neighbor discovery within range"""
        world = MockWorld()
        v2v = V2VNetworkEnhanced(max_range=100.0, world=world)
        
        v1 = MockVehicle(1, 0, 0)
        v2 = MockVehicle(2, 50, 0)  # 50m away
        v3 = MockVehicle(3, 150, 0)  # 150m away (out of range)
        
        for v in [v1, v2, v3]:
            world.add_vehicle(v)
            v2v.register(v.id, v)
        
        v2v.update(force=True)
        
        neighbors = v2v.get_neighbors(1)
        neighbor_ids = [n.vehicle_id for n in neighbors]
        
        self.assertIn(2, neighbor_ids)  # Should be in range
        self.assertNotIn(3, neighbor_ids)  # Should be out of range
    
    def test_network_stats(self):
        """Test network statistics"""
        world = MockWorld()
        v2v = V2VNetworkEnhanced(max_range=100.0, world=world)
        
        v1 = MockVehicle(1, 0, 0)
        world.add_vehicle(v1)
        v2v.register(1, v1)
        v2v.update(force=True)
        
        stats = v2v.get_network_stats()
        
        self.assertIn('total_messages_sent', stats)
        self.assertIn('average_neighbors', stats)
        self.assertGreater(stats['total_messages_sent'], 0)
    
    def test_one_line_status(self):
        """Test one-line status output"""
        world = MockWorld()
        v2v = V2VNetworkEnhanced(max_range=100.0, world=world)
        
        v1 = MockVehicle(1, 0, 0)
        v1._velocity = MockVector(15, 0, 0)
        world.add_vehicle(v1)
        v2v.register(1, v1)
        v2v.update(force=True)
        
        status = v2v.get_one_line_status(1)
        
        self.assertIn('V2V:', status)
        self.assertIn('m/s', status)
        self.assertIn('Neighbors:', status)


def run_tests():
    """Run all tests"""
    loader = unittest.TestLoader()
    suite = loader.loadTestsFromTestCase(TestV2VBasics)
    
    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(suite)
    
    print("\n" + "="*70)
    print("TEST SUMMARY")
    print("="*70)
    print(f"Tests run: {result.testsRun}")
    print(f"Successes: {result.testsRun - len(result.failures) - len(result.errors)}")
    print(f"Failures: {len(result.failures)}")
    print(f"Errors: {len(result.errors)}")
    print("="*70)
    
    return result.wasSuccessful()


if __name__ == '__main__':
    success = run_tests()
    sys.exit(0 if success else 1)
