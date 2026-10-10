"""The vectorised NLOS search must equal the scalar reference (`DSRCChannel._is_nlos`) exactly."""

import random
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).parent.parent.parent))

from src.v2x.channel.dsrc_logdistance import DSRCChannel, _nlos_row


def _check(positions):
    ch = DSRCChannel()
    ids = list(positions)
    xs = np.array([positions[i][0] for i in ids], dtype=np.float64)
    ys = np.array([positions[i][1] for i in ids], dtype=np.float64)
    for si, s in enumerate(ids):
        row = _nlos_row(positions[s], xs, ys, si)
        for ri, r in enumerate(ids):
            if r == s:
                continue
            expected = ch._is_nlos(s, r, positions[s], positions[r], positions)
            assert bool(row[ri]) == expected, (s, r, positions)


@pytest.mark.parametrize("seed", range(40))
def test_random_layouts(seed):
    rng = random.Random(seed)
    n = rng.randint(2, 40)
    scale = rng.choice([5.0, 30.0, 200.0, 1000.0])
    _check({i: (rng.uniform(0, scale), rng.uniform(0, scale)) for i in range(n)})


@pytest.mark.parametrize("seed", range(10))
def test_collinear_and_grid_layouts(seed):
    rng = random.Random(100 + seed)
    spacing = rng.choice([1.0, 2.0, 2.5, 5.0, 25.0])
    pts = {i: (i * spacing, 0.0) for i in range(rng.randint(3, 15))}
    pts.update({100 + i: (rng.randint(0, 6) * spacing, rng.randint(0, 3) * spacing) for i in range(10)})
    _check(pts)


def test_blocker_exactly_on_the_radius_boundary_and_duplicates():
    # blocker at 2.5 m perpendicular distance: not < 2.5, so not blocked (same as the scalar code)
    _check({1: (0.0, 0.0), 2: (10.0, 0.0), 3: (5.0, 2.5)})
    _check({1: (0.0, 0.0), 2: (10.0, 0.0), 3: (5.0, 2.4999999)})
    # identical positions and zero-length segments
    _check({1: (3.0, 3.0), 2: (3.0, 3.0), 3: (3.0, 3.0), 4: (8.0, 3.0)})
    _check({1: (0.0, 0.0), 2: (0.0, 0.0)})


def test_two_stations_never_block_each_other():
    _check({1: (0.0, 0.0), 2: (50.0, 0.0)})


def test_scaling_is_no_longer_cubic():
    import time
    from src.v2x.channel.dsrc_logdistance import DSRCConfig
    from tests.golden.test_dsrc_channel_golden import _bsm
    rng = random.Random(1)

    def run(n):
        pos = {i: (rng.uniform(0, 400), rng.uniform(0, 400)) for i in range(n)}
        msgs = {i: _bsm(i, *p) for i, p in pos.items()}
        ch = DSRCChannel(DSRCConfig())
        t0 = time.perf_counter()
        ch.broadcast_all(msgs, pos)
        return time.perf_counter() - t0

    run(20)  # warm up
    t50, t100 = min(run(50) for _ in range(3)), min(run(100) for _ in range(3))
    # doubling the stations: quadratic work -> ~4x (cubic would be ~8x); allow generous slack
    assert t100 < 6.0 * t50, (t50, t100)
    assert t100 < 0.25, f"100 stations take {t100*1000:.0f} ms per update"
