"""Settle the vehicle-axes convention against CARLA itself (needs the carla module, no server)."""

import random

import pytest

from src.v2x.geometry import basis_vectors

pytestmark = pytest.mark.carla


def test_basis_vectors_match_carla_transform():
    carla = pytest.importorskip("carla")
    rng = random.Random(7)
    for _ in range(100):
        yaw, pitch, roll = rng.uniform(-180, 180), rng.uniform(-80, 80), rng.uniform(-80, 80)
        t = carla.Transform(carla.Location(), carla.Rotation(pitch=pitch, yaw=yaw, roll=roll))
        mine = basis_vectors(yaw, pitch, roll)
        theirs = (t.get_forward_vector(), t.get_right_vector(), t.get_up_vector())
        for m, c in zip(mine, theirs):
            assert (m[0], m[1], m[2]) == pytest.approx((c.x, c.y, c.z), abs=1e-5), (yaw, pitch, roll)
