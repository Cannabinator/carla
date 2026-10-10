"""Behaviour of create_bsm_from_carla (one test group per handoff defect fixed)."""

import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).parent.parent.parent))

from src.v2x.extract import create_bsm_from_carla
from tests.golden.test_bsm_golden import _snapshot, _vec, _vehicle


def _snap_with_yaw_rate(z):
    snap = _snapshot()
    actor = snap.find(7)
    actor.get_angular_velocity = lambda: _vec(0.0, 0.0, z)
    snap.find = lambda i: actor if i == 7 else None
    return snap


@pytest.mark.parametrize("z", [0.0, 0.5, -12.0, 90.0])
def test_yaw_rate_is_passed_through_as_deg_per_s(z):
    """S2: CARLA already reports deg/s; the BSM must not scale it by 180/pi."""
    bsm = create_bsm_from_carla(_vehicle(), 7, 0, snapshot=_snap_with_yaw_rate(z))
    assert bsm.yaw_rate == z
