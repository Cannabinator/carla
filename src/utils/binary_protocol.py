"""
Binary WebSocket protocol for efficient LiDAR data transfer.
Phase 3: Performance Optimization - Reduce bandwidth by 40%.
"""

import struct
import numpy as np
from typing import Tuple, Optional
import zlib


class BinaryProtocol:
    """
    Binary protocol for LiDAR point cloud data.
    
    Format:
    - Header (24 bytes):
      * Magic: 4 bytes ('LIDR')
      * Version: 2 bytes (uint16)
      * Flags: 2 bytes (uint16)
      * Point count: 4 bytes (uint32)
      * Timestamp: 8 bytes (double)
      * Reserved: 4 bytes
    
    - Ego Transform (28 bytes):
      * Position: 12 bytes (3 x float32)
      * Rotation: 12 bytes (3 x float32)
      * Reserved: 4 bytes
    
    - Point Data (variable):
      * Each point: 16 bytes (4 x float32)
        - X, Y, Z: position (float32)
        - Tag: semantic tag (float32 cast to uint8)
    
    Total overhead: 52 bytes + (16 * num_points)
    vs JSON: ~100 bytes header + (50-80 * num_points)
    Savings: ~40-50% bandwidth reduction
    """
    
    MAGIC = b'LIDR'
    VERSION = 1
    HEADER_SIZE = 24
    TRANSFORM_SIZE = 28
    POINT_SIZE = 16  # 4 floats per point
    
    # Flags
    FLAG_COMPRESSED = 1 << 0
    FLAG_HAS_INTENSITY = 1 << 1
    FLAG_HAS_COLOR = 1 << 2
    
    @staticmethod
    def encode(
        points: np.ndarray,
        ego_transform: Tuple[Tuple[float, float, float], Tuple[float, float, float]],
        timestamp: float = 0.0,
        compress: bool = False
    ) -> bytes:
        """
        Encode point cloud to binary format.
        
        Args:
            points: Nx4 array (x, y, z, tag)
            ego_transform: ((x, y, z), (yaw, pitch, roll))
            timestamp: Frame timestamp
            compress: Use zlib compression
        
        Returns:
            Binary encoded data
        """
        num_points = len(points)
        flags = 0
        if compress:
            flags |= BinaryProtocol.FLAG_COMPRESSED
        
        # Build header
        header = struct.pack(
            '4sHHIdd',
            BinaryProtocol.MAGIC,
            BinaryProtocol.VERSION,
            flags,
            num_points,
            timestamp,
            0  # reserved
        )
        
        # Build ego transform
        pos, rot = ego_transform
        transform = struct.pack(
            '6fi',
            pos[0], pos[1], pos[2],
            rot[0], rot[1], rot[2],
            0  # reserved
        )
        
        # Build point data (convert to float32)
        points_f32 = points.astype(np.float32)
        point_data = points_f32.tobytes()
        
        # Combine
        payload = header + transform + point_data
        
        # Optional compression
        if compress:
            point_data_compressed = zlib.compress(point_data, level=1)
            payload = header + transform + point_data_compressed
        
        return payload
    
    @staticmethod
    def decode(data: bytes) -> Optional[Tuple[np.ndarray, dict]]:
        """
        Decode binary point cloud data.
        
        Args:
            data: Binary encoded data
        
        Returns:
            Tuple of (points array, metadata dict) or None if invalid
        """
        if len(data) < BinaryProtocol.HEADER_SIZE + BinaryProtocol.TRANSFORM_SIZE:
            return None
        
        # Parse header
        try:
            magic, version, flags, num_points, timestamp, _ = struct.unpack(
                '4sHHIdd',
                data[:BinaryProtocol.HEADER_SIZE]
            )
        except struct.error:
            return None
        
        if magic != BinaryProtocol.MAGIC:
            return None
        
        # Parse ego transform
        offset = BinaryProtocol.HEADER_SIZE
        try:
            pos_x, pos_y, pos_z, rot_yaw, rot_pitch, rot_roll, _ = struct.unpack(
                '6fi',
                data[offset:offset + BinaryProtocol.TRANSFORM_SIZE]
            )
        except struct.error:
            return None
        
        # Parse point data
        offset += BinaryProtocol.TRANSFORM_SIZE
        point_data = data[offset:]
        
        # Decompress if needed
        if flags & BinaryProtocol.FLAG_COMPRESSED:
            try:
                point_data = zlib.decompress(point_data)
            except zlib.error:
                return None
        
        # Convert to numpy array
        expected_size = num_points * BinaryProtocol.POINT_SIZE
        if len(point_data) != expected_size:
            return None
        
        points = np.frombuffer(point_data, dtype=np.float32).reshape(-1, 4)
        
        # Build metadata
        metadata = {
            'version': version,
            'flags': flags,
            'timestamp': timestamp,
            'num_points': num_points,
            'ego_position': (pos_x, pos_y, pos_z),
            'ego_rotation': (rot_yaw, rot_pitch, rot_roll),
            'compressed': bool(flags & BinaryProtocol.FLAG_COMPRESSED)
        }
        
        return points, metadata
    
    @staticmethod
    def estimate_size(num_points: int, compress: bool = False) -> int:
        """Estimate binary payload size."""
        base_size = BinaryProtocol.HEADER_SIZE + BinaryProtocol.TRANSFORM_SIZE
        point_size = num_points * BinaryProtocol.POINT_SIZE
        
        if compress:
            # Assume 50% compression ratio
            point_size = int(point_size * 0.5)
        
        return base_size + point_size


def compare_bandwidth(num_points: int) -> dict:
    """
    Compare bandwidth usage: JSON vs Binary.
    
    Args:
        num_points: Number of points in point cloud
    
    Returns:
        Dict with size comparisons
    """
    # JSON estimate (conservative)
    json_overhead = 100  # {"points":[], "ego_transform":{...}}
    json_point_size = 60  # [1.234, 5.678, 9.012, 10] with formatting
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
