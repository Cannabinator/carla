"""calculate_threat_level: closing speed from velocity vectors (S14)."""

import math
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).parent.parent.parent))

from src.v2x.messages import BSMCore
from src.v2x.threat import calculate_threat_level


def bsm(vid, x, y, vx=None, vy=None, speed=0.0, heading=0.0):
    return BSMCore(timestamp=0.0, msg_count=0, vehicle_id=vid, x_m=x, y_m=y,
                   vx_mps=vx, vy_mps=vy, speed=speed, heading=heading)


def test_head_on_ttc_from_velocity_vectors():
    ego = bsm(1, 0, 0, vx=10, vy=0, speed=10)
    other = bsm(2, 50, 0, vx=-10, vy=0, speed=10, heading=180)
    level, ttc, dist = calculate_threat_level(ego, other)
    assert dist == pytest.approx(50.0) and ttc == pytest.approx(2.5) and level == 3


def test_reversing_vehicle_is_not_a_threat_to_the_car_in_front_of_it():
    """Heading 0 (nose towards +x) but moving backwards at 5 m/s: it is moving AWAY from the
    stationary vehicle 50 m ahead. The old heading-based calculation saw it closing at 5 m/s
    (TTC 10 s, threat level 2)."""
    ego = bsm(1, 0, 0, vx=-5, vy=0, speed=5, heading=0)
    other = bsm(2, 50, 0, vx=0, vy=0, speed=0, heading=0)
    level, ttc, _ = calculate_threat_level(ego, other)
    assert level == 0 and math.isinf(ttc)


def test_sideways_slide_is_detected_although_the_heading_points_elsewhere():
    ego = bsm(1, 0, 0, vx=0, vy=5, speed=5, heading=0)   # points at +x but slides towards +y
    other = bsm(2, 0, 20, vx=0, vy=0, speed=0)
    level, ttc, _ = calculate_threat_level(ego, other)
    assert ttc == pytest.approx(4.0) and level == 3


def test_hand_made_bsm_without_velocity_vector_falls_back_to_heading():
    ego = bsm(1, 0, 0, speed=10, heading=0)
    other = bsm(2, 50, 0, speed=0, heading=0)
    level, ttc, _ = calculate_threat_level(ego, other)
    assert ttc == pytest.approx(5.0) and level == 3


def test_diverging_vehicles_have_no_threat():
    ego = bsm(1, 0, 0, vx=-3, vy=0, speed=3)
    other = bsm(2, 30, 0, vx=3, vy=0, speed=3)
    level, ttc, _ = calculate_threat_level(ego, other)
    assert level == 0 and math.isinf(ttc)
