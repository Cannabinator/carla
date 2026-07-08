"""
Binary WebSocket protocol for efficient LiDAR data transfer.
Version 2: Fixes V1 header alignment bug, adds object_idx per point,
ego_actor_id and num_vehicles in header for ego self-filtering.

Wire format (all little-endian, no alignment padding):

  Header (28 bytes):
    4s  magic         b'LIDR'
    H   version       2
    H   flags         bit 0 = zlib-compressed point data
    d   timestamp     float64, seconds
    I   num_points    uint32
    I   num_vehicles  uint32
    I   ego_actor_id  uint32  (used by frontend to filter ego self-hits)

  Ego Transform (24 bytes):
    6f  x, y, z, yaw, pitch, roll  (float32 each)

  Point Data (num_points × 20 bytes):
    5f per point: x, y, z, tag, object_idx  (all float32)
    - tag       cast from uint32 (max CARLA tag = 28, exact in f32)
    - object_idx cast from uint32 (CARLA IDs < 2^24, exact in f32)

  Total overhead: 52 bytes + 20 × num_points
  vs JSON:        ~60 bytes/point  →  6× bandwidth reduction

JS parsing (DataView for first 52 bytes, Float32Array for points):
  view.getFloat64(8, true)   → timestamp
  view.getUint32(16, true)   → num_points
  view.getUint32(20, true)   → num_vehicles
  view.getUint32(24, true)   → ego_actor_id
  new Float32Array(buf, 28, 6)          → ego transform
  new Float32Array(buf, 52, n*5)        → points  (offset 52 is 4-byte aligned)
"""

import struct
import numpy as np
from typing import Tuple, Optional, Dict, Any
import zlib


class BinaryProtocol:
    MAGIC = b'LIDR'
    VERSION = 2

    # Explicit little-endian ('<') eliminates native alignment padding.
    # V1 used native format '4sHHIdd' which struct.calcsize() = 32 on 64-bit
    # Linux (8-byte alignment for double), but HEADER_SIZE was hardcoded to 24,
    # making every V1 decode silently return None.
    HEADER_FORMAT = '<4sHHdIII'
    HEADER_SIZE = struct.calcsize(HEADER_FORMAT)    # 28 bytes, verified

    TRANSFORM_FORMAT = '<6f'
    TRANSFORM_SIZE = struct.calcsize(TRANSFORM_FORMAT)  # 24 bytes

    POINT_SIZE = 20   # 5 × float32: x, y, z, tag, object_idx

    FLAG_COMPRESSED = 1 << 0

    @staticmethod
    def encode(
        points: np.ndarray,
        ego_transform: Tuple[Tuple[float, float, float], Tuple[float, float, float]],
        timestamp: float = 0.0,
        num_vehicles: int = 1,
        ego_actor_id: int = 0,
        compress: bool = False,
    ) -> bytes:
        """Encode a structured or Nx5 point cloud to binary.

        Args:
            points: Structured numpy array with fields (x, y, z, object_tag,
                    object_idx) OR plain Nx5 float32 array.
            ego_transform: ((x, y, z), (yaw, pitch, roll))
            timestamp: Frame timestamp in seconds.
            num_vehicles: Number of registered vehicles.
            ego_actor_id: CARLA actor ID of the ego vehicle (vehicle_id=0).
            compress: Apply zlib level-1 compression to point data.

        Returns:
            Binary payload ready to send over WebSocket.
        """
        num_points = len(points)
        flags = BinaryProtocol.FLAG_COMPRESSED if compress else 0

        header = struct.pack(
            BinaryProtocol.HEADER_FORMAT,
            BinaryProtocol.MAGIC,
            BinaryProtocol.VERSION,
            flags,
            float(timestamp),
            num_points,
            num_vehicles,
            ego_actor_id,
        )

        pos, rot = ego_transform
        transform = struct.pack(
            BinaryProtocol.TRANSFORM_FORMAT,
            float(pos[0]), float(pos[1]), float(pos[2]),
            float(rot[0]), float(rot[1]), float(rot[2]),
        )

        # Build Nx5 float32 array from structured or plain array.
        if points.dtype.names:
            pts = np.column_stack([
                points['x'].astype(np.float32),
                points['y'].astype(np.float32),
                points['z'].astype(np.float32),
                points['object_tag'].astype(np.float32),
                points['object_idx'].astype(np.float32),
            ])
        else:
            pts = np.asarray(points[:, :5], dtype=np.float32)

        point_data = pts.tobytes()
        if compress:
            point_data = zlib.compress(point_data, level=1)

        return header + transform + point_data

    @staticmethod
    def decode(data: bytes) -> Optional[Tuple[np.ndarray, Dict[str, Any]]]:
        """Decode a binary payload produced by encode().

        Returns:
            (points Nx5 float32, metadata dict) or None if payload is invalid.
        """
        min_size = BinaryProtocol.HEADER_SIZE + BinaryProtocol.TRANSFORM_SIZE
        if len(data) < min_size:
            return None

        try:
            magic, version, flags, timestamp, num_points, num_vehicles, ego_actor_id = \
                struct.unpack(BinaryProtocol.HEADER_FORMAT,
                              data[:BinaryProtocol.HEADER_SIZE])
        except struct.error:
            return None

        if magic != BinaryProtocol.MAGIC:
            return None

        offset = BinaryProtocol.HEADER_SIZE
        try:
            ex, ey, ez, eyaw, epitch, eroll = struct.unpack(
                BinaryProtocol.TRANSFORM_FORMAT,
                data[offset:offset + BinaryProtocol.TRANSFORM_SIZE],
            )
        except struct.error:
            return None

        offset += BinaryProtocol.TRANSFORM_SIZE
        point_data = data[offset:]

        if flags & BinaryProtocol.FLAG_COMPRESSED:
            try:
                point_data = zlib.decompress(point_data)
            except zlib.error:
                return None

        expected = num_points * BinaryProtocol.POINT_SIZE
        if len(point_data) != expected:
            return None

        points = np.frombuffer(point_data, dtype=np.float32).reshape(-1, 5)

        metadata: Dict[str, Any] = {
            'version': version,
            'flags': flags,
            'timestamp': timestamp,
            'num_points': num_points,
            'num_vehicles': num_vehicles,
            'ego_actor_id': ego_actor_id,
            'ego_position': (ex, ey, ez),
            'ego_rotation': (eyaw, epitch, eroll),
            'compressed': bool(flags & BinaryProtocol.FLAG_COMPRESSED),
        }
        return points, metadata

    @staticmethod
    def estimate_size(num_points: int, compress: bool = False) -> int:
        """Estimate binary payload size in bytes."""
        base = BinaryProtocol.HEADER_SIZE + BinaryProtocol.TRANSFORM_SIZE
        point_bytes = num_points * BinaryProtocol.POINT_SIZE
        if compress:
            point_bytes = int(point_bytes * 0.5)
        return base + point_bytes


def compare_bandwidth(num_points: int) -> dict:
    """Compare bandwidth usage: JSON vs Binary.

    Args:
        num_points: Number of points in point cloud

    Returns:
        Dict with size comparisons
    """
    # JSON estimate: ~70 bytes/point for [x, y, z, tag, idx] arrays
    json_overhead = 100
    json_point_size = 70
    json_size = json_overhead + (num_points * json_point_size)
    
    # Binary sizes
    binary_size = BinaryProtocol.estimate_size(num_points, compress=False)
    binary_compressed_size = BinaryProtocol.estimate_size(num_points, compress=True)
    
    return {
        'num_points': num_points,
        'json_bytes': json_size,
        'binary_bytes': binary_size,
        'binary_compressed_bytes': binary_compressed_size,
        'binary_savings_pct': ((json_size - binary_size) / json_size) * 100,
        'compressed_savings_pct': ((json_size - binary_compressed_size) / json_size) * 100,
        'json_mb_per_sec_10hz': (json_size * 10) / (1024 * 1024),
        'binary_mb_per_sec_10hz': (binary_size * 10) / (1024 * 1024),
        'binary_compressed_mb_per_sec_10hz': (binary_compressed_size * 10) / (1024 * 1024),
    }


if __name__ == '__main__':
    # Bandwidth comparison
    print("LiDAR Binary Protocol - Bandwidth Comparison\n")
    print("=" * 80)
    
    for num_points in [10000, 20000, 44000, 100000]:
        stats = compare_bandwidth(num_points)
        print(f"\n📊 {num_points:,} points:")
        print(f"   JSON:               {stats['json_bytes']:,} bytes ({stats['json_mb_per_sec_10hz']:.2f} MB/s @ 10Hz)")
        print(f"   Binary:             {stats['binary_bytes']:,} bytes ({stats['binary_mb_per_sec_10hz']:.2f} MB/s @ 10Hz)")
        print(f"   Binary Compressed:  {stats['binary_compressed_bytes']:,} bytes ({stats['binary_compressed_mb_per_sec_10hz']:.2f} MB/s @ 10Hz)")
        print(f"   Savings:            {stats['binary_savings_pct']:.1f}% (uncompressed)")
        print(f"   Savings:            {stats['compressed_savings_pct']:.1f}% (compressed)")
    
    print("\n" + "=" * 80)
    print("\n✅ Binary protocol provides 40-70% bandwidth reduction!")
