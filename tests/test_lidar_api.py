#!/usr/bin/env python3
"""
Unit Tests for LiDAR Streaming API
Tests the flexible LiDAR API for different use cases.
"""

import unittest
from unittest.mock import Mock, MagicMock, patch, call
import sys
from pathlib import Path
import numpy as np

# Add parent directory to path
sys.path.insert(0, str(Path(__file__).parent.parent))

# Now import after path is set
import src.lidar.api as api_module
from src.lidar.api import LiDARStreamingAPI, create_ego_lidar_stream
from src.utils.binary_protocol import BinaryProtocol


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


class TestBinaryProtocol(unittest.TestCase):
    """Unit tests for BinaryProtocol V2 encode/decode round-trip."""

    def _make_structured_points(self, n=10):
        dtype = np.dtype([
            ('x', np.float32), ('y', np.float32), ('z', np.float32),
            ('cos_inc_angle', np.float32),
            ('object_tag', np.uint32), ('object_idx', np.uint32),
        ])
        pts = np.zeros(n, dtype=dtype)
        pts['x'] = np.linspace(0, 50, n, dtype=np.float32)
        pts['y'] = np.linspace(0, 10, n, dtype=np.float32)
        pts['z'] = np.linspace(0,  5, n, dtype=np.float32)
        pts['object_tag'] = np.arange(n, dtype=np.uint32) % 23
        pts['object_idx'] = np.arange(100, 100 + n, dtype=np.uint32)
        return pts

    def test_header_size_matches_struct(self):
        """HEADER_SIZE constant must match the actual struct format size."""
        import struct
        computed = struct.calcsize(BinaryProtocol.HEADER_FORMAT)
        self.assertEqual(BinaryProtocol.HEADER_SIZE, computed,
                         "HEADER_SIZE is out of sync with HEADER_FORMAT")

    def test_transform_size_matches_struct(self):
        """TRANSFORM_SIZE constant must match the actual struct format size."""
        import struct
        computed = struct.calcsize(BinaryProtocol.TRANSFORM_FORMAT)
        self.assertEqual(BinaryProtocol.TRANSFORM_SIZE, computed)

    def test_encode_returns_bytes(self):
        pts = self._make_structured_points(20)
        result = BinaryProtocol.encode(pts, ((1.0, 2.0, 3.0), (90.0, 0.0, 0.0)))
        self.assertIsInstance(result, bytes)

    def test_encode_decode_roundtrip(self):
        """Encode then decode must recover all fields exactly."""
        pts = self._make_structured_points(50)
        ego_pos = (10.5, 20.5, 1.0)
        ego_rot = (45.0, 0.0, 0.0)

        payload = BinaryProtocol.encode(
            pts, (ego_pos, ego_rot),
            timestamp=123.456,
            num_vehicles=3,
            ego_actor_id=42,
        )
        result = BinaryProtocol.decode(payload)
        self.assertIsNotNone(result, "decode() returned None — likely a struct/size mismatch")

        points_out, meta = result

        self.assertEqual(meta['num_points'], 50)
        self.assertEqual(meta['num_vehicles'], 3)
        self.assertEqual(meta['ego_actor_id'], 42)
        self.assertAlmostEqual(meta['timestamp'], 123.456, places=3)
        self.assertAlmostEqual(meta['ego_position'][0], 10.5, places=4)
        self.assertAlmostEqual(meta['ego_position'][1], 20.5, places=4)
        self.assertAlmostEqual(meta['ego_rotation'][0], 45.0, places=4)

        self.assertEqual(points_out.shape, (50, 5))
        np.testing.assert_array_almost_equal(points_out[:, 0], pts['x'])
        np.testing.assert_array_almost_equal(points_out[:, 1], pts['y'])
        np.testing.assert_array_almost_equal(points_out[:, 2], pts['z'])
        np.testing.assert_array_almost_equal(points_out[:, 3], pts['object_tag'].astype(np.float32))
        np.testing.assert_array_almost_equal(points_out[:, 4], pts['object_idx'].astype(np.float32))

    def test_encode_decode_compressed(self):
        """Compressed round-trip must recover the same point count."""
        pts = self._make_structured_points(100)
        payload = BinaryProtocol.encode(
            pts, ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0)), compress=True
        )
        result = BinaryProtocol.decode(payload)
        self.assertIsNotNone(result)
        points_out, meta = result
        self.assertTrue(meta['compressed'])
        self.assertEqual(points_out.shape[0], 100)

    def test_decode_rejects_bad_magic(self):
        """decode() must return None when magic bytes are wrong."""
        pts = self._make_structured_points(5)
        payload = bytearray(BinaryProtocol.encode(pts, ((0, 0, 0), (0, 0, 0))))
        payload[0] = 0xFF  # corrupt magic
        self.assertIsNone(BinaryProtocol.decode(bytes(payload)))

    def test_decode_rejects_short_buffer(self):
        """decode() must return None on buffers that are too short."""
        self.assertIsNone(BinaryProtocol.decode(b'\x00' * 10))

    def test_payload_size(self):
        """Binary payload must be exactly HEADER+TRANSFORM + n*POINT_SIZE bytes."""
        n = 200
        pts = self._make_structured_points(n)
        payload = BinaryProtocol.encode(pts, ((0, 0, 0), (0, 0, 0)))
        expected = (BinaryProtocol.HEADER_SIZE
                    + BinaryProtocol.TRANSFORM_SIZE
                    + n * BinaryProtocol.POINT_SIZE)
        self.assertEqual(len(payload), expected)


class TestCollectorPointcloudSchema(unittest.TestCase):
    """Verify get_combined_pointcloud() output schema includes object_idx."""

    def _make_collector_with_data(self):
        """Build a LiDARDataCollector whose internals are stubbed."""
        import numpy as np
        # Import lazily to avoid CARLA import at module level
        sys.modules.setdefault('carla', Mock())
        from src.visualization.lidar.collector import LiDARDataCollector

        mock_world = Mock()
        mock_world.get_blueprint_library.return_value.find.return_value = Mock(
            set_attribute=Mock()
        )

        collector = LiDARDataCollector.__new__(LiDARDataCollector)
        collector.world = mock_world
        collector.downsample_factor = 1
        collector.lidar_config = {}
        collector.vehicles = {0: Mock()}
        collector.lidar_sensors = {}
        collector._lock = __import__('threading').RLock()
        collector._is_active = True
        collector._data_received_count = 0
        collector.actor_ids = {0: 99}

        # Inject synthetic structured point cloud
        dtype = np.dtype([
            ('x', np.float32), ('y', np.float32), ('z', np.float32),
            ('cos_inc_angle', np.float32),
            ('object_tag', np.uint32), ('object_idx', np.uint32),
        ])
        pts = np.zeros(5, dtype=dtype)
        pts['x'] = [1, 2, 3, 4, 5]
        pts['object_tag'] = [7, 7, 10, 10, 9]
        pts['object_idx'] = [99, 99, 200, 201, 0]

        mock_transform = Mock()
        mock_transform.location.x = 0.0
        mock_transform.location.y = 0.0
        mock_transform.location.z = 0.0
        mock_transform.rotation.yaw = 0.0
        mock_transform.rotation.pitch = 0.0
        mock_transform.rotation.roll = 0.0

        collector.latest_data = {0: pts}
        collector.vehicle_transforms = {0: mock_transform}
        return collector

    def test_json_output_includes_idx(self):
        """get_combined_pointcloud() must include 'idx' in points dict."""
        collector = self._make_collector_with_data()
        data = collector.get_combined_pointcloud()
        self.assertIsNotNone(data)
        self.assertIn('idx', data['points'], "object_idx missing from JSON payload")
        self.assertEqual(len(data['points']['idx']), data['num_points'])

    def test_json_output_basic_invariants(self):
        """JSON payload must have consistent lengths across all point arrays."""
        collector = self._make_collector_with_data()
        data = collector.get_combined_pointcloud()
        n = data['num_points']
        self.assertGreater(n, 0)
        for key in ('x', 'y', 'z', 'tag', 'idx'):
            self.assertEqual(len(data['points'][key]), n,
                             f"points['{key}'] length mismatch")

    def test_binary_output_is_decodable(self):
        """get_combined_pointcloud_binary() must produce a valid BinaryProtocol payload."""
        collector = self._make_collector_with_data()
        payload = collector.get_combined_pointcloud_binary()
        self.assertIsNotNone(payload, "binary payload is None (BinaryProtocol import failed?)")
        result = BinaryProtocol.decode(payload)
        self.assertIsNotNone(result, "BinaryProtocol.decode() failed on collector output")
        points_out, meta = result
        self.assertEqual(meta['num_points'], 5)
        self.assertEqual(meta['ego_actor_id'], 99)


if __name__ == '__main__':
    # Run tests with verbose output
    unittest.main(verbosity=2)
