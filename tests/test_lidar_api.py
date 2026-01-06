#!/usr/bin/env python3
"""
Unit Tests for LiDAR Streaming API
Tests the flexible LiDAR API for different use cases.
"""

import unittest
from unittest.mock import Mock, MagicMock, patch, call
import sys
from pathlib import Path

# Add parent directory to path
sys.path.insert(0, str(Path(__file__).parent.parent))

# Now import after path is set
import src.visualization.lidar.api as api_module
from src.visualization.lidar.api import LiDARStreamingAPI, create_ego_lidar_stream


class TestLiDARStreamingAPI(unittest.TestCase):
    """Test cases for LiDARStreamingAPI class."""
    
    def setUp(self):
        """Set up test fixtures."""
        self.mock_world = Mock()
        self.mock_vehicle = Mock()
        self.mock_vehicle.id = 42
    
    @patch('src.visualization.lidar.api.LiDARDataCollector')
    def test_initialization_default_config(self, mock_collector_class):
        """Test API initialization with default configuration."""
        api = LiDARStreamingAPI(self.mock_world)
        
        # Verify collector was created with defaults
        mock_collector_class.assert_called_once()
        call_args = mock_collector_class.call_args
        
        self.assertEqual(call_args[1]['world'], self.mock_world)
        self.assertEqual(call_args[1]['downsample_factor'], 1)
        self.assertEqual(call_args[1]['channels'], 64)
        self.assertEqual(call_args[1]['points_per_second'], 1000000)
    
    @patch('src.visualization.lidar.api.LiDARDataCollector')
    def test_initialization_custom_config(self, mock_collector_class):
        """Test API initialization with custom configuration."""
        api = LiDARStreamingAPI(
            self.mock_world,
            downsample_factor=2,
            channels=32,
            points_per_second=500000,
            lidar_range=50.0
        )
        
        call_args = mock_collector_class.call_args
        self.assertEqual(call_args[1]['downsample_factor'], 2)
        self.assertEqual(call_args[1]['channels'], 32)
        self.assertEqual(call_args[1]['points_per_second'], 500000)
        # lidar_range is passed as kwarg 'range' to collector
        self.assertEqual(call_args[1]['range'], 50.0)
    
    @patch('src.visualization.lidar.api.LiDARDataCollector')
    def test_register_vehicle(self, mock_collector_class):
        """Test vehicle registration."""
        api = LiDARStreamingAPI(self.mock_world)
        mock_collector = mock_collector_class.return_value
        
        api.register_vehicle(self.mock_vehicle, vehicle_id=0)
        
        mock_collector.register_vehicle.assert_called_once_with(0, self.mock_vehicle)
    
    @patch('src.visualization.lidar.api.LiDARDataCollector')
    def test_register_ego_only(self, mock_collector_class):
        """Test ego-only vehicle registration."""
        api = LiDARStreamingAPI(self.mock_world)
        mock_collector = mock_collector_class.return_value
        
        api.register_ego_only(self.mock_vehicle)
        
        # Should register with vehicle_id=0
        mock_collector.register_vehicle.assert_called_once_with(0, self.mock_vehicle)
    
    @patch('src.visualization.lidar.api.LiDARDataCollector')
    @patch('src.visualization.lidar.api.set_collector')
    @patch('src.visualization.lidar.api.threading.Thread')
    @patch('src.visualization.lidar.api.time.sleep')
    def test_start_server_background(self, mock_sleep, mock_thread, mock_set_collector, mock_collector_class):
        """Test starting server in background mode."""
        api = LiDARStreamingAPI(self.mock_world)
        mock_collector = mock_collector_class.return_value
        
        api.start_server(background=True)
        
        # Verify collector was registered
        mock_set_collector.assert_called_once_with(mock_collector)
        
        # Verify thread was created and started
        mock_thread.assert_called_once()
        mock_thread.return_value.start.assert_called_once()
        
        # Verify server is marked as running
        self.assertTrue(api.is_running)
    
    @patch('src.visualization.lidar.api.LiDARDataCollector')
    def test_stop(self, mock_collector_class):
        """Test stopping the API and cleanup."""
        api = LiDARStreamingAPI(self.mock_world)
        mock_collector = mock_collector_class.return_value
        api.is_running = True
        
        api.stop()
        
        # Verify cleanup was called
        mock_collector.cleanup.assert_called_once()
        
        # Verify running flag cleared
        self.assertFalse(api.is_running)
    
    @patch('src.visualization.lidar.api.LiDARDataCollector')
    def test_get_point_count(self, mock_collector_class):
        """Test getting current point count."""
        api = LiDARStreamingAPI(self.mock_world)
        mock_collector = mock_collector_class.return_value
        
        # Mock point cloud data
        mock_collector.get_combined_pointcloud.return_value = {
            'num_points': 50000,
            'num_vehicles': 1
        }
        
        count = api.get_point_count()
        self.assertEqual(count, 50000)
    
    @patch('src.visualization.lidar.api.LiDARDataCollector')
    def test_get_point_count_no_data(self, mock_collector_class):
        """Test getting point count when no data available."""
        api = LiDARStreamingAPI(self.mock_world)
        mock_collector = mock_collector_class.return_value
        mock_collector.get_combined_pointcloud.return_value = None
        
        count = api.get_point_count()
        self.assertEqual(count, 0)
    
    @patch('src.visualization.lidar.api.LiDARDataCollector')
    def test_get_vehicle_count(self, mock_collector_class):
        """Test getting number of registered vehicles."""
        api = LiDARStreamingAPI(self.mock_world)
        mock_collector = mock_collector_class.return_value
        mock_collector.vehicles = {0: self.mock_vehicle, 1: Mock(), 2: Mock()}
        
        count = api.get_vehicle_count()
        self.assertEqual(count, 3)


class TestCreateEgoLidarStream(unittest.TestCase):
    """Test cases for create_ego_lidar_stream convenience function."""
    
    def setUp(self):
        """Set up test fixtures."""
        self.mock_world = Mock()
        self.mock_vehicle = Mock()
        self.mock_vehicle.id = 42
    
    @patch('src.visualization.lidar.api.LiDARStreamingAPI')
    def test_create_high_quality(self, mock_api_class):
        """Test creating high-quality ego stream."""
        mock_api = mock_api_class.return_value
        
        result = create_ego_lidar_stream(
            self.mock_world,
            self.mock_vehicle,
            web_port=8000,
            high_quality=True
        )
        
        # Verify high quality settings
        call_args = mock_api_class.call_args
        self.assertEqual(call_args[1]['downsample_factor'], 1)
        self.assertEqual(call_args[1]['channels'], 64)
        self.assertEqual(call_args[1]['points_per_second'], 1000000)
        
        # Verify ego vehicle registered
        mock_api.register_ego_only.assert_called_once_with(self.mock_vehicle)
        
        # Verify server started
        mock_api.start_server.assert_called_once()
        
        # Verify return value
        self.assertEqual(result, mock_api)
    
    @patch('src.visualization.lidar.api.LiDARStreamingAPI')
    def test_create_fast_mode(self, mock_api_class):
        """Test creating fast mode ego stream."""
        mock_api = mock_api_class.return_value
        
        result = create_ego_lidar_stream(
            self.mock_world,
            self.mock_vehicle,
            web_port=8000,
            high_quality=False
        )
        
        # Verify fast mode settings
        call_args = mock_api_class.call_args
        self.assertEqual(call_args[1]['downsample_factor'], 2)
        self.assertEqual(call_args[1]['channels'], 32)
        self.assertEqual(call_args[1]['points_per_second'], 500000)
        
        # Verify setup completed
        mock_api.register_ego_only.assert_called_once()
        mock_api.start_server.assert_called_once()


class TestAPIConfiguration(unittest.TestCase):
    """Test different configuration scenarios."""
    
    @patch('src.visualization.lidar.api.LiDARDataCollector')
    def test_single_vehicle_mode(self, mock_collector_class):
        """Test API configured for single vehicle (most common use case)."""
        world = Mock()
        vehicle = Mock()
        
        api = LiDARStreamingAPI(world, downsample_factor=1)
        api.register_ego_only(vehicle)
        
        # Should have only 1 vehicle
        mock_collector = mock_collector_class.return_value
        self.assertEqual(mock_collector.register_vehicle.call_count, 1)
    
    @patch('src.visualization.lidar.api.LiDARDataCollector')
    def test_multi_vehicle_mode(self, mock_collector_class):
        """Test API configured for multiple vehicles."""
        world = Mock()
        vehicles = [Mock(), Mock(), Mock()]
        
        api = LiDARStreamingAPI(world)
        for i, vehicle in enumerate(vehicles):
            api.register_vehicle(vehicle, vehicle_id=i)
        
        # Should have 3 vehicles
        mock_collector = mock_collector_class.return_value
        self.assertEqual(mock_collector.register_vehicle.call_count, 3)
    
    @patch('src.visualization.lidar.api.LiDARDataCollector')
    def test_performance_mode_settings(self, mock_collector_class):
        """Test fast performance mode settings."""
        world = Mock()
        
        # Fast mode: lower resolution, higher downsampling
        api = LiDARStreamingAPI(
            world,
            downsample_factor=4,
            channels=16,
            points_per_second=250000,
            lidar_range=50.0
        )
        
        call_args = mock_collector_class.call_args
        self.assertEqual(call_args[1]['downsample_factor'], 4)
        self.assertEqual(call_args[1]['channels'], 16)
        self.assertEqual(call_args[1]['points_per_second'], 250000)


if __name__ == '__main__':
    # Run tests with verbose output
    unittest.main(verbosity=2)
