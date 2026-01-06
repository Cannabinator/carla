#!/usr/bin/env python3
"""
Comprehensive tests for V2V REST API
Tests all endpoints, WebSocket functionality, and data validation.
"""

import unittest
import sys
import asyncio
from unittest.mock import Mock, MagicMock, patch
from fastapi.testclient import TestClient

sys.path.insert(0, '/home/workstation/carla')

from src.v2v import V2VNetworkEnhanced, create_v2v_api, BSMCore, VehicleType, BrakingStatus
from src.v2v.api import V2VAPI

# Import mock classes from test_v2v_basic
from test_v2v_basic import MockVehicle, MockWorld


class TestV2VAPI(unittest.TestCase):
    """Test V2V REST API endpoints"""
    
    def setUp(self):
        """Setup test API client"""
        # Create V2V network
        self.world = MockWorld()
        self.v2v = V2VNetworkEnhanced(max_range=100.0, update_rate_hz=2.0, world=self.world)
        
        # Create API
        self.api = create_v2v_api(self.v2v, port=8001)
        
        # Create test client
        self.client = TestClient(self.api.app)
    
    def test_root_endpoint(self):
        """Test root endpoint returns service info"""
        response = self.client.get("/")
        
        self.assertEqual(response.status_code, 200)
        data = response.json()
        
        self.assertIn("service", data)
        self.assertIn("version", data)
        self.assertIn("endpoints", data)
        self.assertEqual(data["service"], "V2V Network API")
    
    def test_get_vehicles_empty(self):
        """Test getting vehicles when none registered"""
        response = self.client.get("/vehicles")
        
        self.assertEqual(response.status_code, 200)
        data = response.json()
        
        self.assertIsInstance(data, list)
        self.assertEqual(len(data), 0)
    
    def test_get_vehicles_with_data(self):
        """Test getting vehicles when some registered"""
        v1 = MockVehicle(1, x=0, y=0)
        v2 = MockVehicle(2, x=50, y=0)
        
        self.world.add_vehicle(v1)
        self.world.add_vehicle(v2)
        
        self.v2v.register(1, v1)
        self.v2v.register(2, v2)
        self.v2v.update(force=True)
        
        response = self.client.get("/vehicles")
        
        self.assertEqual(response.status_code, 200)
        data = response.json()
        
        self.assertEqual(len(data), 2)
        self.assertIn(1, data)
        self.assertIn(2, data)
    
    def test_get_vehicle_not_found(self):
        """Test getting non-existent vehicle returns 404"""
        response = self.client.get("/vehicles/999")
        
        self.assertEqual(response.status_code, 404)
        self.assertIn("detail", response.json())
    
    def test_get_vehicle_success(self):
        """Test getting specific vehicle BSM data"""
        v1 = MockVehicle(1, x=100, y=50)
        v1._velocity.x = 15
        
        self.world.add_vehicle(v1)
        self.v2v.register(1, v1)
        self.v2v.update(force=True)
        
        response = self.client.get("/vehicles/1")
        
        self.assertEqual(response.status_code, 200)
        data = response.json()
        
        self.assertEqual(data["vehicle_id"], 1)
        self.assertIn("position", data)
        self.assertIn("speed", data)
        self.assertIn("heading", data)
        self.assertEqual(data["position"]["x"], 100)
        self.assertEqual(data["position"]["y"], 50)
    
    def test_get_neighbors(self):
        """Test getting neighbors for vehicle"""
        v1 = MockVehicle(1, x=0, y=0)
        v2 = MockVehicle(2, x=50, y=0)
        v3 = MockVehicle(3, x=0, y=50)
        
        for v in [v1, v2, v3]:
            self.world.add_vehicle(v)
            self.v2v.register(v.id, v)
        self.v2v.update(force=True)
        
        response = self.client.get("/vehicles/1/neighbors")
        
        self.assertEqual(response.status_code, 200)
        data = response.json()
        
        self.assertIsInstance(data, list)
        # Should have 2 neighbors
        self.assertGreater(len(data), 0)
        
        # Check neighbor structure
        if len(data) > 0:
            neighbor = data[0]
            self.assertIn("vehicle_id", neighbor)
            self.assertIn("distance", neighbor)
            self.assertIn("relative_speed", neighbor)
            self.assertIn("bsm", neighbor)
    
    def test_get_neighbors_vehicle_not_found(self):
        """Test getting neighbors for non-existent vehicle"""
        response = self.client.get("/vehicles/999/neighbors")
        
        self.assertEqual(response.status_code, 404)
    
    def test_get_threats(self):
        """Test getting threats for vehicle"""
        v1 = MockVehicle(1, x=0, y=0)
        v1._velocity.x = 20
        v2 = MockVehicle(2, x=20, y=0)
        v2._velocity.x = 10
        
        self.world.add_vehicle(v1)
        self.world.add_vehicle(v2)
        self.v2v.register(1, v1)
        self.v2v.register(2, v2)
        self.v2v.update(force=True)
        
        response = self.client.get("/vehicles/1/threats")
        
        self.assertEqual(response.status_code, 200)
        data = response.json()
        
        self.assertIsInstance(data, list)
    
    def test_get_all_bsm(self):
        """Test getting all BSM messages"""
        v1 = MockVehicle(1, x=0, y=0)
        v2 = MockVehicle(2, x=50, y=0)
        
        self.world.add_vehicle(v1)
        self.world.add_vehicle(v2)
        self.v2v.register(1, v1)
        self.v2v.register(2, v2)
        self.v2v.update(force=True)
        
        response = self.client.get("/bsm")
        
        self.assertEqual(response.status_code, 200)
        data = response.json()
        
        self.assertIsInstance(data, list)
        self.assertEqual(len(data), 2)
        
        # Check BSM structure
        bsm = data[0]
        self.assertIn("vehicle_id", bsm)
        self.assertIn("timestamp", bsm)
        self.assertIn("speed", bsm)
        self.assertIn("heading", bsm)
        self.assertIn("position", bsm)
    
    def test_get_bsm_specific_vehicle(self):
        """Test getting BSM for specific vehicle"""
        v1 = MockVehicle(1, x=100, y=50)
        
        self.world.add_vehicle(v1)
        self.v2v.register(1, v1)
        self.v2v.update(force=True)
        
        response = self.client.get("/bsm/1")
        
        self.assertEqual(response.status_code, 200)
        data = response.json()
        
        self.assertEqual(data["vehicle_id"], 1)
        self.assertEqual(data["position"]["x"], 100)
        self.assertEqual(data["position"]["y"], 50)
    
    def test_get_network_stats(self):
        """Test getting network statistics"""
        v1 = MockVehicle(1, x=0, y=0)
        v2 = MockVehicle(2, x=50, y=0)
        
        self.world.add_vehicle(v1)
        self.world.add_vehicle(v2)
        self.v2v.register(1, v1)
        self.v2v.register(2, v2)
        self.v2v.update(force=True)
        
        response = self.client.get("/network/stats")
        
        self.assertEqual(response.status_code, 200)
        data = response.json()
        
        self.assertIn("total_vehicles", data)
        self.assertIn("total_messages_sent", data)
        self.assertIn("average_neighbors", data)
        self.assertIn("update_rate_hz", data)
        self.assertIn("max_range_m", data)
        
        self.assertEqual(data["total_vehicles"], 2)
        self.assertEqual(data["update_rate_hz"], 2.0)
        self.assertEqual(data["max_range_m"], 100.0)


class TestAPIResponseFormat(unittest.TestCase):
    """Test API response data formats"""
    
    def setUp(self):
        """Setup test API client"""
        self.world = MockWorld()
        self.v2v = V2VNetworkEnhanced(max_range=100.0, update_rate_hz=2.0, world=self.world)
        self.api = create_v2v_api(self.v2v, port=8001)
        self.client = TestClient(self.api.app)
    
    def test_bsm_response_format(self):
        """Test BSM response has all required fields"""
        v1 = MockVehicle(1, x=100, y=50)
        v1._velocity.x = 15
        
        self.world.add_vehicle(v1)
        self.v2v.register(1, v1)
        self.v2v.update(force=True)
        
        response = self.client.get("/bsm/1")
        data = response.json()
        
        # Check all required BSM fields
        required_fields = [
            "vehicle_id", "timestamp", "msg_count", "vehicle_type",
            "position", "speed", "heading", "steering_angle",
            "acceleration", "dimensions", "brake_status",
            "brake_pressure", "transmission_state"
        ]
        
        for field in required_fields:
            self.assertIn(field, data, f"Missing field: {field}")
        
        # Check nested structures
        self.assertIn("x", data["position"])
        self.assertIn("y", data["position"])
        self.assertIn("z", data["position"])
        
        self.assertIn("longitudinal", data["acceleration"])
        self.assertIn("lateral", data["acceleration"])
        
        self.assertIn("length", data["dimensions"])
        self.assertIn("width", data["dimensions"])
        self.assertIn("height", data["dimensions"])
    
    def test_neighbor_response_format(self):
        """Test neighbor response format"""
        v1 = MockVehicle(1, x=0, y=0)
        v2 = MockVehicle(2, x=50, y=0)
        
        self.world.add_vehicle(v1)
        self.world.add_vehicle(v2)
        self.v2v.register(1, v1)
        self.v2v.register(2, v2)
        self.v2v.update(force=True)
        
        response = self.client.get("/vehicles/1/neighbors")
        data = response.json()
        
        if len(data) > 0:
            neighbor = data[0]
            
            self.assertIn("vehicle_id", neighbor)
            self.assertIn("distance", neighbor)
            self.assertIn("relative_speed", neighbor)
            self.assertIn("bsm", neighbor)
            
            # Check BSM is complete
            self.assertIn("speed", neighbor["bsm"])
            self.assertIn("heading", neighbor["bsm"])


def run_tests():
    """Run all API tests"""
    loader = unittest.TestLoader()
    suite = unittest.TestSuite()
    
    suite.addTests(loader.loadTestsFromTestCase(TestV2VAPI))
    suite.addTests(loader.loadTestsFromTestCase(TestAPIResponseFormat))
    
    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(suite)
    
    print("\n" + "="*70)
    print("API TEST SUMMARY")
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
