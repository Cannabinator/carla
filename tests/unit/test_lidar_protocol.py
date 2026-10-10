#!/usr/bin/env python3
"""BinaryProtocol and collector point-cloud schema tests (split out of test_lidar_api.py)."""

import sys
import unittest
from pathlib import Path
from unittest.mock import Mock

import numpy as np

sys.path.insert(0, str(Path(__file__).parent.parent.parent))

from src.utils.binary_protocol import BinaryProtocol


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
