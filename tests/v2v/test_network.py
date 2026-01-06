#!/usr/bin/env python3
"""
Unit Tests for V2V Network
Tests the lightweight V2V communication implementation.
"""

import unittest
import sys
from pathlib import Path
from unittest.mock import Mock, MagicMock
import numpy as np
import carla
import math

sys.path.insert(0, str(Path(__file__).parent.parent.parent))

from src.v2v import V2VNetwork, V2VState


class MockVehicle:
    """Mock CARLA vehicle for testing."""
    
    def __init__(self, x, y, z, vx=0, vy=0, vz=0):
        self.x, self.y, self.z = x, y, z
        self.vx, self.vy, self.vz = vx, vy, vz
        
    def get_transform(self):
        mock_transform = Mock()
        mock_transform.location = Mock(x=self.x, y=self.y, z=self.z)
        return mock_transform
    
    def get_velocity(self):
        return Mock(x=self.vx, y=self.vy, z=self.vz)


class TestV2VState(unittest.TestCase):
    """Test V2VState data structure."""
    
    def test_create_state(self):
        """Test creating V2VState directly."""
        state = V2VState(
            id=1,
            location=(10.0, 20.0, 0.5),
            velocity=(5.0, 0.0, 0.0),
            speed=18.0,
            timestamp=123.456
        )
        
        self.assertEqual(state.id, 1)
        self.assertEqual(state.location[0], 10.0)
        self.assertEqual(state.speed, 18.0)
    
    def test_from_vehicle(self):
        """Test creating V2VState from mock vehicle."""
        vehicle = MockVehicle(10.0, 20.0, 0.5, vx=5.0)
        state = V2VState.from_vehicle(vehicle, vehicle_id=42)
        
        self.assertEqual(state.id, 42)
        self.assertEqual(state.location[0], 10.0)
        self.assertEqual(state.location[1], 20.0)
        self.assertGreater(state.speed, 0)  # Should calculate speed from velocity


class TestV2VNetwork(unittest.TestCase):
    """Test V2V network functionality."""
    
    def setUp(self):
        """Set up test network."""
        self.network = V2VNetwork(max_range=50.0)
        
        # Create mock vehicles at different positions
        self.v1 = MockVehicle(0, 0, 0)      # Origin
        self.v2 = MockVehicle(30, 0, 0)     # 30m away (in range)
        self.v3 = MockVehicle(100, 0, 0)    # 100m away (out of range)
        
    def test_register_vehicle(self):
        """Test registering vehicles."""
        self.network.register(1, self.v1)
        self.network.register(2, self.v2)
        
        self.assertEqual(len(self.network.vehicles), 2)
        self.assertIn(1, self.network.vehicles)
        self.assertIn(2, self.network.vehicles)
    
    def test_update_states(self):
        """Test updating vehicle states."""
        self.network.register(1, self.v1)
        self.network.register(2, self.v2)
        
        self.network.update()
        
        self.assertEqual(len(self.network.states), 2)
        self.assertIn(1, self.network.states)
        self.assertIn(2, self.network.states)
    
    def test_neighbor_discovery_in_range(self):
        """Test finding neighbors within range."""
        self.network.register(1, self.v1)
        self.network.register(2, self.v2)  # 30m away
        
        self.network.update()
        
        # v1 and v2 should be neighbors (30m < 50m)
        self.assertIn(2, self.network.neighbors[1])
        self.assertIn(1, self.network.neighbors[2])
    
    def test_neighbor_discovery_out_of_range(self):
        """Test that far vehicles are not neighbors."""
        self.network.register(1, self.v1)
        self.network.register(3, self.v3)  # 100m away
        
        self.network.update()
        
        # v1 and v3 should NOT be neighbors (100m > 50m)
        self.assertNotIn(3, self.network.neighbors[1])
        self.assertNotIn(1, self.network.neighbors[3])
    
    def test_get_neighbors(self):
        """Test getting neighbor states."""
        self.network.register(1, self.v1)
        self.network.register(2, self.v2)
        self.network.register(3, self.v3)
        
        self.network.update()
        
        neighbors = self.network.get_neighbors(1)
        
        # v1 should have v2 as neighbor, but not v3
        self.assertEqual(len(neighbors), 1)
        self.assertEqual(neighbors[0].id, 2)
    
    def test_get_state(self):
        """Test getting individual vehicle state."""
        self.network.register(1, self.v1)
        self.network.update()
        
        state = self.network.get_state(1)
        
        self.assertIsNotNone(state)
        self.assertEqual(state.id, 1)
        self.assertEqual(state.location[0], 0)
    
    def test_empty_network(self):
        """Test behavior with no vehicles."""
        self.network.update()
        
        state = self.network.get_state(999)
        neighbors = self.network.get_neighbors(999)
        
        self.assertIsNone(state)
        self.assertEqual(len(neighbors), 0)
    
    def test_velocity_calculation(self):
        """Test that velocity is properly converted from Vector3D."""
        mock_vehicle = MagicMock(spec=carla.Vehicle)
        mock_transform = MagicMock()
        mock_transform.location.x = 100.0
        mock_transform.location.y = 200.0
        mock_transform.location.z = 0.5
        mock_transform.rotation.yaw = 45.0
        mock_vehicle.get_transform.return_value = mock_transform
        
        # Mock velocity as Vector3D with actual values
        mock_velocity = MagicMock(spec=carla.Vector3D)
        mock_velocity.x = 10.0  # 10 m/s in x direction
        mock_velocity.y = 5.0   # 5 m/s in y direction
        mock_velocity.z = 0.0
        mock_vehicle.get_velocity.return_value = mock_velocity
        
        state = V2VState.from_vehicle(1, mock_vehicle, 1.0)
        
        # Check velocity tuple
        self.assertEqual(state.velocity, (10.0, 5.0, 0.0))
        
        # Check speed calculation (sqrt(10^2 + 5^2) = sqrt(125) ≈ 11.18 m/s)
        expected_speed = math.sqrt(10.0**2 + 5.0**2)
        self.assertAlmostEqual(state.speed, expected_speed, places=2)
        
    def test_speed_zero_when_stationary(self):
        """Test that speed is zero for stationary vehicles."""
        mock_vehicle = MagicMock(spec=carla.Vehicle)
        mock_transform = MagicMock()
        mock_transform.location.x = 0.0
        mock_transform.location.y = 0.0
        mock_transform.location.z = 0.0
        mock_transform.rotation.yaw = 0.0
        mock_vehicle.get_transform.return_value = mock_transform
        
        # Stationary vehicle
        mock_velocity = MagicMock(spec=carla.Vector3D)
        mock_velocity.x = 0.0
        mock_velocity.y = 0.0
        mock_velocity.z = 0.0
        mock_vehicle.get_velocity.return_value = mock_velocity
        
        state = V2VState.from_vehicle(1, mock_vehicle, 1.0)
        
        self.assertEqual(state.velocity, (0.0, 0.0, 0.0))
        self.assertEqual(state.speed, 0.0)
    

if __name__ == '__main__':
    unittest.main()
