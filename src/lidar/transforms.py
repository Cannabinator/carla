"""Sensor-to-world transforms (pure math; replaces the hand-written rotation of the old collector, L1).

A CARLA transform maps local coordinates to world coordinates with the 4x4 matrix whose rotation
columns are the object's forward, right and up axes (see ``v2x/geometry.py``) and whose last column is
its location. LiDAR points are in the SENSOR frame; ``to_world`` applies the sensor's pose.

TODO(verify): (a) these axes against ``carla.Transform.get_matrix()``
(tests/integration/test_geometry_live.py compares the axes); (b) that CARLA 0.9.16 delivers lidar points
in the plain sensor frame without a y flip (tests/integration/test_lidar_live.py checks it against a prop
at a known position).
"""

from __future__ import annotations

from typing import Sequence, Tuple

import numpy as np

from src.v2x.geometry import basis_vectors


def transform_matrix(location: Sequence[float], rotation_pyr_deg: Sequence[float]) -> np.ndarray:
    """4x4 matrix local -> world for a pose. ``rotation_pyr_deg`` = (pitch, yaw, roll) in degrees,
    CARLA's order."""
    pitch, yaw, roll = rotation_pyr_deg
    forward, right, up = basis_vectors(yaw, pitch, roll)
    m = np.eye(4, dtype=np.float64)
    m[:3, 0] = forward
    m[:3, 1] = right
    m[:3, 2] = up
    m[:3, 3] = location
    return m


def to_world(points_xyz: np.ndarray, location: Sequence[float], rotation_pyr_deg: Sequence[float]) -> np.ndarray:
    """Transform Nx3 sensor-frame points to world coordinates (float64)."""
    m = transform_matrix(location, rotation_pyr_deg)
    p = np.asarray(points_xyz, dtype=np.float64)
    return p @ m[:3, :3].T + m[:3, 3]


def to_sensor(points_xyz: np.ndarray, location: Sequence[float], rotation_pyr_deg: Sequence[float]) -> np.ndarray:
    """Inverse of ``to_world`` (rotation matrices are orthonormal: inverse = transpose)."""
    m = transform_matrix(location, rotation_pyr_deg)
    p = np.asarray(points_xyz, dtype=np.float64)
    return (p - m[:3, 3]) @ m[:3, :3]


def compose(parent_loc, parent_rot_pyr, child_loc, child_rot_pyr) -> Tuple[np.ndarray, np.ndarray]:
    """World location of a child pose given in a parent's frame (for sensor mounts); returns
    (location xyz, 3x3 rotation matrix)."""
    mp = transform_matrix(parent_loc, parent_rot_pyr)
    mc = transform_matrix(child_loc, child_rot_pyr)
    m = mp @ mc
    return m[:3, 3], m[:3, :3]
