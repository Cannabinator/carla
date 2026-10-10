"""LiDAR binary protocol v3 (REFACTOR_HANDOFF section 5.4). Little-endian, no padding.

    Header (48 bytes)
      4s  magic 'LIDR'
      H   version = 3
      H   flags        bit0 point data is zlib-compressed, bit1 points in world frame,
                       bit2 points in sensor frame (exactly one of bit1/bit2 is set)
      Q   frame        simulation frame id the measurement belongs to
      d   sim_time     simulation seconds of that frame
      d   wall_time_unix   seconds since the epoch when the frame was encoded (never a model clock)
      I   num_points
      I   ego_actor_id     actor the sensor is attached to (lets a viewer drop its own hits)
      I   sensor_id
      H   point_stride_bytes
      H   field_mask
    Sensor pose in the world (48 bytes): 6 float64 = x, y, z (m), pitch, yaw, roll (deg, CARLA order)
    Points: num_points records of point_stride_bytes bytes, fields in the fixed order below for the
            bits set in field_mask:
      bit0 XYZ         3 x float32
      bit1 TAG_U8      uint8   semantic tag (CARLA 0.9.14+ ids)
      bit2 TAG_U32     uint32  semantic tag (use when a tag does not fit in a byte)
      bit3 OBJECT_IDX  uint32  actor id of the hit object (semantic sensor)
      bit4 INTENSITY   float32 (ray_cast sensor)
      bit5 COS_INC     float32 cosine of the incidence angle (semantic sensor)
    The stream uses the SENSOR frame (raw as CARLA delivers it, pose in the header); the browser applies
    the pose (src/lidar/transforms.py / lidar-decoder.js). Mirrored in src/web/static/js/lidar-decoder.js
    and checked by tests/unit/test_lidar_protocol_v3.py against bytes made here.
"""

from __future__ import annotations

import struct
import time
import zlib
from dataclasses import dataclass
from typing import Dict, Mapping, Optional, Tuple

import numpy as np

MAGIC = b"LIDR"
VERSION = 3

HEADER_FORMAT = "<4sHHQddIIIHH"
HEADER_SIZE = struct.calcsize(HEADER_FORMAT)      # 48
POSE_FORMAT = "<6d"
POSE_SIZE = struct.calcsize(POSE_FORMAT)          # 48
PREAMBLE_SIZE = HEADER_SIZE + POSE_SIZE           # 96

FLAG_ZLIB = 1 << 0
FLAG_WORLD_FRAME = 1 << 1
FLAG_SENSOR_FRAME = 1 << 2

FIELD_XYZ = 1 << 0
FIELD_TAG_U8 = 1 << 1
FIELD_TAG_U32 = 1 << 2
FIELD_OBJECT_IDX = 1 << 3
FIELD_INTENSITY = 1 << 4
FIELD_COS_INC = 1 << 5

# (mask bit, [(name, numpy dtype), ...]) in wire order
_FIELDS = (
    (FIELD_XYZ, (("x", "<f4"), ("y", "<f4"), ("z", "<f4"))),
    (FIELD_TAG_U8, (("tag", "u1"),)),
    (FIELD_TAG_U32, (("tag", "<u4"),)),
    (FIELD_OBJECT_IDX, (("object_idx", "<u4"),)),
    (FIELD_INTENSITY, (("intensity", "<f4"),)),
    (FIELD_COS_INC, (("cos_inc", "<f4"),)),
)


class ProtocolError(ValueError):
    pass


@dataclass(frozen=True)
class FrameHeader:
    frame: int
    sim_time: float
    sensor_id: int
    ego_actor_id: int
    location: Tuple[float, float, float]          # sensor, world frame, metres
    rotation_pyr: Tuple[float, float, float]      # pitch, yaw, roll in degrees
    wall_time_unix: float = 0.0
    sensor_frame: bool = True                     # True: points are in the sensor frame
    num_points: int = 0                           # filled in by decode_frame
    field_mask: int = 0                           # filled in by decode_frame
    compressed: bool = False                      # filled in by decode_frame


def point_dtype(field_mask: int) -> np.dtype:
    """Packed structured dtype for a field mask (itemsize = point_stride_bytes)."""
    if not field_mask & FIELD_XYZ:
        raise ProtocolError("field_mask must include XYZ")
    if field_mask & FIELD_TAG_U8 and field_mask & FIELD_TAG_U32:
        raise ProtocolError("TAG_U8 and TAG_U32 are mutually exclusive")
    spec = []
    for bit, fields in _FIELDS:
        if field_mask & bit:
            spec.extend(fields)
    return np.dtype(spec)


def encode_frame(header: FrameHeader, points: Mapping[str, np.ndarray], *, compress: bool = False,
                 wall_time_unix: Optional[float] = None) -> bytes:
    """Encode one frame. ``points`` maps field names to equal-length arrays: x, y, z (required) and any of
    tag, object_idx, intensity, cos_inc. A tag array is stored as one byte per point when every tag < 256."""
    for name in ("x", "y", "z"):
        if name not in points:
            raise ProtocolError(f"missing field {name!r}")
    n = len(points["x"])
    if any(len(points[k]) != n for k in points):
        raise ProtocolError("all point arrays must have the same length")

    mask = FIELD_XYZ
    if "tag" in points:
        mask |= FIELD_TAG_U8 if (n == 0 or int(np.max(points["tag"])) < 256) else FIELD_TAG_U32
    if "object_idx" in points:
        mask |= FIELD_OBJECT_IDX
    if "intensity" in points:
        mask |= FIELD_INTENSITY
    if "cos_inc" in points:
        mask |= FIELD_COS_INC

    dtype = point_dtype(mask)
    records = np.empty(n, dtype=dtype)
    for name in dtype.names or ():
        records[name] = points[name]
    body = records.tobytes()
    flags = FLAG_SENSOR_FRAME if header.sensor_frame else FLAG_WORLD_FRAME
    if compress:
        body = zlib.compress(body, level=1)
        flags |= FLAG_ZLIB

    wall = time.time() if wall_time_unix is None else wall_time_unix
    head = struct.pack(HEADER_FORMAT, MAGIC, VERSION, flags, header.frame, header.sim_time, wall, n,
                       header.ego_actor_id, header.sensor_id, dtype.itemsize, mask)
    pose = struct.pack(POSE_FORMAT, *header.location, *header.rotation_pyr)
    return head + pose + body


def decode_frame(data: bytes) -> Tuple[FrameHeader, Dict[str, np.ndarray]]:
    """Decode a v3 frame to (header, {field: array}). Raises ProtocolError on anything malformed."""
    if len(data) < PREAMBLE_SIZE:
        raise ProtocolError("buffer shorter than the v3 header")
    (magic, version, flags, frame, sim_time, wall, n, ego, sensor_id, stride, mask) = \
        struct.unpack_from(HEADER_FORMAT, data, 0)
    if magic != MAGIC:
        raise ProtocolError("bad magic")
    if version != VERSION:
        raise ProtocolError(f"unsupported protocol version {version}")
    if bool(flags & FLAG_WORLD_FRAME) == bool(flags & FLAG_SENSOR_FRAME):
        raise ProtocolError("exactly one of the world-frame / sensor-frame flags must be set")
    dtype = point_dtype(mask)
    if dtype.itemsize != stride:
        raise ProtocolError(f"point_stride_bytes {stride} does not match field_mask {mask:#x}")
    pose = struct.unpack_from(POSE_FORMAT, data, HEADER_SIZE)
    body = bytes(data[PREAMBLE_SIZE:])
    if flags & FLAG_ZLIB:
        try:
            body = zlib.decompress(body)
        except zlib.error as exc:
            raise ProtocolError(f"corrupt compressed point data: {exc}") from exc
    if len(body) != n * stride:
        raise ProtocolError(f"point data is {len(body)} bytes, expected {n * stride}")
    records = np.frombuffer(body, dtype=dtype, count=n)
    points = {name: records[name].copy() for name in dtype.names or ()}
    header = FrameHeader(
        frame=frame, sim_time=sim_time, sensor_id=sensor_id, ego_actor_id=ego,
        location=(pose[0], pose[1], pose[2]), rotation_pyr=(pose[3], pose[4], pose[5]),
        wall_time_unix=wall, sensor_frame=bool(flags & FLAG_SENSOR_FRAME),
        num_points=n, field_mask=mask, compressed=bool(flags & FLAG_ZLIB))
    return header, points
