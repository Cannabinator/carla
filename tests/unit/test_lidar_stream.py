"""Stream profiles: quantisation, deterministic non-strided subsampling, encode-once fan-out."""

import numpy as np
import pytest

from src.lidar.config import LidarConfig
from src.lidar.protocol import decode_frame
from src.lidar.sensor import LidarFrame
from src.lidar.stream import (SensorStream, StreamProfile, Subsampler, frame_divisor, quantize_points,
                              quantize_profile)


def make_frame(frame=20, n=60_000, sensor_id=1, seed=0):
    rng = np.random.default_rng(seed)
    pts = {"x": rng.random(n, dtype=np.float32), "y": rng.random(n, dtype=np.float32),
           "z": rng.random(n, dtype=np.float32), "cos_inc": rng.random(n, dtype=np.float32),
           "object_idx": np.arange(n, dtype=np.uint32), "tag": rng.integers(0, 29, n).astype(np.uint32)}
    return LidarFrame(sensor_id, frame, frame * 0.05, 7, (1, 2, 3), (0, 90, 0), pts, LidarConfig())


def test_quantize_points():
    assert quantize_points(None) is None and quantize_points(0) is None
    assert quantize_points(1) == 10_000
    assert quantize_points(10_000) == 10_000
    assert quantize_points(24_999) == 10_000
    assert quantize_points(30_000) == 25_000
    assert quantize_points(10**9) == 50_000


def test_frame_divisor_and_profile_quantisation():
    assert frame_divisor(20, 20) == 1
    assert frame_divisor(10, 20) == 2
    assert frame_divisor(7, 20) == 3            # 20/3 = 6.67 <= 7
    assert frame_divisor(1000, 20) == 1
    with pytest.raises(ValueError):
        frame_divisor(0, 20)
    p = quantize_profile(30_000, 7, True, 20)
    assert p == StreamProfile(25_000, pytest.approx(20 / 3), True)
    # nearby requests collapse onto one profile -> shared bytes
    assert quantize_profile(26_000, 6.9, True, 20) == quantize_profile(30_000, 6.8, True, 20)


def test_subsampler_is_deterministic_sorted_and_not_a_stride():
    a, b = Subsampler(5), Subsampler(5)
    ia, ib = a.indices(60_000, 10_000), b.indices(60_000, 10_000)
    assert np.array_equal(ia, ib) and len(ia) == 10_000 and np.all(np.diff(ia) > 0)
    assert not np.array_equal(ia, Subsampler(6).indices(60_000, 10_000))
    gaps = np.diff(ia)
    assert gaps.std() > 1.0                     # a stride would have zero gap variance
    assert np.array_equal(a.indices(5, 10), np.arange(5))
    # different n later is still fine and independent of call history
    assert np.array_equal(Subsampler(5).indices(30_000, 1_000), a.indices(30_000, 1_000))


def test_subsample_is_unbiased_over_the_index_range():
    idx = Subsampler(1).indices(100_000, 10_000)
    halves = np.histogram(idx, bins=10, range=(0, 100_000))[0]
    assert halves.min() > 800 and halves.max() < 1200       # ~1000 per tenth


def test_encode_once_per_profile_and_byte_identical():
    s = SensorStream(1, 20.0)
    f = make_frame()
    p = quantize_profile(10_000, 20, False, 20)
    first = s.encode(f, p)
    second = s.encode(f, p)
    assert first is second and first.data is second.data and s.encodes == 1
    other = s.encode(f, quantize_profile(None, 20, False, 20))
    assert s.encodes == 2 and other.points_sent == 60_000 and not other.subsampled
    assert first.subsampled and first.points_sent == 10_000 and first.points_total == 60_000


def test_cache_is_per_frame():
    s = SensorStream(1, 20.0)
    p = quantize_profile(10_000, 20, False, 20)
    a = s.encode(make_frame(frame=20), p)
    b = s.encode(make_frame(frame=21), p)
    assert a.frame == 20 and b.frame == 21 and s.encodes == 2


def test_max_hz_skips_frames():
    s = SensorStream(1, 20.0)
    p = quantize_profile(None, 10, False, 20)
    got = [s.encode(make_frame(frame=f, n=100), p) for f in range(20, 26)]
    assert [g is not None for g in got] == [True, False, True, False, True, False]


def test_decoded_subsample_is_a_consistent_subset_of_the_frame():
    s = SensorStream(3, 20.0)
    f = make_frame(sensor_id=3)
    enc = s.encode(f, quantize_profile(10_000, 20, True, 20))
    header, pts = decode_frame(enc.data)
    assert header.num_points == 10_000 and header.compressed and header.frame == f.frame
    assert header.sensor_id == 3 and header.ego_actor_id == 7
    # every field was subsampled with the same indices: object_idx was arange, so it identifies the source row
    src = pts["object_idx"].astype(int)
    assert np.array_equal(pts["x"], f.points["x"][src]) and np.array_equal(pts["tag"], f.points["tag"][src])
    assert np.all(np.diff(src) > 0)


def test_small_frame_is_not_subsampled():
    s = SensorStream(1, 20.0)
    enc = s.encode(make_frame(n=500), quantize_profile(10_000, 20, False, 20))
    assert enc.points_sent == 500 and not enc.subsampled


def test_frame_geometry_survives_encoding():
    s = SensorStream(1, 20.0)
    f = make_frame(n=100)
    header, pts = decode_frame(s.encode(f, quantize_profile(None, 20, False, 20)).data)
    assert header.location == (1, 2, 3) and header.rotation_pyr == (0, 90, 0) and header.sim_time == f.sim_time
    assert np.array_equal(pts["y"], f.points["y"])
