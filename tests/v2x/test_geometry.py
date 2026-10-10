import math
import random
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).parent.parent.parent))

from src.v2x.geometry import basis_vectors, dot


def _close(a, b, tol=1e-9):
    return all(abs(x - y) < tol for x, y in zip(a, b))


@pytest.mark.parametrize("yaw,fwd,right", [
    (0, (1, 0, 0), (0, 1, 0)),
    (90, (0, 1, 0), (-1, 0, 0)),
    (180, (-1, 0, 0), (0, -1, 0)),
    (-90, (0, -1, 0), (1, 0, 0)),
])
def test_flat_vehicle_axes(yaw, fwd, right):
    f, r, u = basis_vectors(yaw)
    assert _close(f, fwd, 1e-12) and _close(r, right, 1e-12) and _close(u, (0, 0, 1), 1e-12)


def test_pitch_up_tilts_forward_upwards():
    f, _, _ = basis_vectors(0, pitch_deg=30)
    assert f[2] == pytest.approx(0.5)


def test_roll_tilts_right_axis_downwards():
    # positive roll lowers the right-hand side (z of the right axis = -cos(p)*sin(r))
    _, r, _ = basis_vectors(0, 0, roll_deg=30)
    assert r[2] == pytest.approx(-0.5)


def test_axes_are_orthonormal_and_right_handed_for_random_angles():
    rng = random.Random(1)
    for _ in range(200):
        f, r, u = basis_vectors(rng.uniform(-180, 180), rng.uniform(-60, 60), rng.uniform(-60, 60))
        for v in (f, r, u):
            assert dot(v, v) == pytest.approx(1.0)
        assert dot(f, r) == pytest.approx(0.0, abs=1e-12)
        assert dot(f, u) == pytest.approx(0.0, abs=1e-12)
        assert dot(r, u) == pytest.approx(0.0, abs=1e-12)
        cross = (f[1] * r[2] - f[2] * r[1], f[2] * r[0] - f[0] * r[2], f[0] * r[1] - f[1] * r[0])
        assert _close(cross, u, 1e-9)


def test_dot_projection():
    f, r, _ = basis_vectors(45)
    a = (1.0, 1.0, 0.0)
    assert dot(a, f) == pytest.approx(math.sqrt(2))
    assert dot(a, r) == pytest.approx(0.0, abs=1e-12)
