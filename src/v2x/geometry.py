"""Vehicle-frame axes in CARLA world coordinates (pure math, no CARLA import).

CARLA (Unreal Engine) convention: left-handed, x forward, y right, z up; angles in degrees;
yaw turns x towards y. The columns of CARLA's Transform matrix are the vehicle's forward,
right and up axes in world coordinates:

    forward = ( cp*cy,                cp*sy,                sp    )
    right   = ( cy*sp*sr - sy*cr,     sy*sp*sr + cy*cr,    -cp*sr )
    up      = (-cy*sp*cr - sy*sr,    -sy*sp*cr + cy*sr,     cp*cr )

(c/s = cos/sin of p = pitch, y = yaw, r = roll.)
TODO(verify): compared against carla.Transform.get_forward_vector()/get_right_vector()/
get_up_vector() by tests/integration/test_geometry_live.py, which needs the carla module.
"""

import math
from typing import Tuple

Vec3 = Tuple[float, float, float]


def basis_vectors(yaw_deg: float, pitch_deg: float = 0.0, roll_deg: float = 0.0) -> Tuple[Vec3, Vec3, Vec3]:
    """Return (forward, right, up) unit vectors in world coordinates."""
    y, p, r = math.radians(yaw_deg), math.radians(pitch_deg), math.radians(roll_deg)
    cy, sy = math.cos(y), math.sin(y)
    cp, sp = math.cos(p), math.sin(p)
    cr, sr = math.cos(r), math.sin(r)
    forward = (cp * cy, cp * sy, sp)
    right = (cy * sp * sr - sy * cr, sy * sp * sr + cy * cr, -cp * sr)
    up = (-cy * sp * cr - sy * sr, -sy * sp * cr + cy * sr, cp * cr)
    return forward, right, up


def dot(a: Vec3, b: Vec3) -> float:
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
