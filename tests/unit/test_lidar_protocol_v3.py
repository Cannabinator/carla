"""LiDAR protocol v3: round trips, error handling, and the cross-language check against the JS decoder."""

import json
import shutil
import struct
import subprocess
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).parent.parent.parent))

from src.lidar import protocol as pr

ROOT = Path(__file__).parent.parent.parent


def _header(**kw):
    base = dict(frame=1234567, sim_time=61.725, sensor_id=9, ego_actor_id=77,
                location=(120.5, -33.25, 2.9), rotation_pyr=(1.5, -135.0, 0.25))
    base.update(kw)
    return pr.FrameHeader(**base)


def _semantic(n=50, seed=1, big_tags=False):
    rng = np.random.default_rng(seed)
    return {
        "x": rng.uniform(-80, 80, n).astype(np.float32), "y": rng.uniform(-80, 80, n).astype(np.float32),
        "z": rng.uniform(-3, 8, n).astype(np.float32),
        "tag": rng.integers(0, 28 if not big_tags else 70000, n).astype(np.uint32),
        "object_idx": rng.integers(0, 2**31, n).astype(np.uint32),
        "cos_inc": rng.uniform(0, 1, n).astype(np.float32),
    }


def _raycast(n=40, seed=2):
    rng = np.random.default_rng(seed)
    return {"x": rng.uniform(-50, 50, n).astype(np.float32), "y": rng.uniform(-50, 50, n).astype(np.float32),
            "z": rng.uniform(0, 5, n).astype(np.float32), "intensity": rng.uniform(0, 1, n).astype(np.float32)}


def test_sizes():
    assert (pr.HEADER_SIZE, pr.POSE_SIZE, pr.PREAMBLE_SIZE) == (48, 48, 96)


@pytest.mark.parametrize("compress", [False, True])
@pytest.mark.parametrize("make", [_semantic, _raycast])
def test_roundtrip_is_exact(make, compress):
    pts = make()
    h, out = pr.decode_frame(pr.encode_frame(_header(), pts, compress=compress, wall_time_unix=1.7e9))
    assert (h.frame, h.sim_time, h.sensor_id, h.ego_actor_id) == (1234567, 61.725, 9, 77)
    assert h.location == (120.5, -33.25, 2.9) and h.rotation_pyr == (1.5, -135.0, 0.25)
    assert h.wall_time_unix == 1.7e9 and h.sensor_frame and h.compressed == compress
    assert set(out) == set(pts) and h.num_points == len(pts["x"])
    for k in pts:
        assert np.array_equal(out[k], pts[k]), k


def test_frame_and_wall_clock_are_separate_fields_and_frame_is_64_bit():
    big = 2**40 + 5
    h, _ = pr.decode_frame(pr.encode_frame(_header(frame=big), _raycast(3)))
    assert h.frame == big and h.wall_time_unix > 1.6e9       # wall clock only as its own field


def test_tags_use_one_byte_unless_they_do_not_fit():
    small = pr.encode_frame(_header(), _semantic(10))
    large = pr.encode_frame(_header(), _semantic(10, big_tags=True))
    assert pr.decode_frame(small)[0].field_mask & pr.FIELD_TAG_U8
    assert pr.decode_frame(large)[0].field_mask & pr.FIELD_TAG_U32
    assert len(large) - len(small) == 10 * 3
    assert np.array_equal(pr.decode_frame(large)[1]["tag"], _semantic(10, big_tags=True)["tag"])


def test_object_ids_above_2_pow_24_survive():
    """L8: the old protocol sent object_idx as float32 (exact only below 2**24)."""
    pts = _semantic(5)
    pts["object_idx"] = np.array([16_777_217, 2**30 + 1, 4_000_000_001, 1, 0], dtype=np.uint32)
    _, out = pr.decode_frame(pr.encode_frame(_header(), pts))
    assert np.array_equal(out["object_idx"], pts["object_idx"])


def test_world_frame_flag_and_empty_frame():
    h, out = pr.decode_frame(pr.encode_frame(_header(sensor_frame=False), _raycast(0)))
    assert not h.sensor_frame and h.num_points == 0 and len(out["x"]) == 0


def test_compression_shrinks_redundant_data():
    pts = {"x": np.zeros(20000, np.float32), "y": np.zeros(20000, np.float32), "z": np.zeros(20000, np.float32)}
    assert len(pr.encode_frame(_header(), pts, compress=True)) < 0.1 * len(pr.encode_frame(_header(), pts))


@pytest.mark.parametrize("mutate,message", [
    (lambda b: b[:10], "shorter"),
    (lambda b: b"XXXX" + b[4:], "magic"),
    (lambda b: b[:4] + struct.pack("<H", 2) + b[6:], "version"),
    (lambda b: b[:6] + struct.pack("<H", 0) + b[8:], "flags"),
    (lambda b: b[:6] + struct.pack("<H", 6) + b[8:], "flags"),
    (lambda b: b[:44] + struct.pack("<H", 99) + b[46:], "stride"),
    (lambda b: b[:-1], "expected"),
    (lambda b: b + b"\x00", "expected"),
])
def test_malformed_frames_are_rejected(mutate, message):
    good = pr.encode_frame(_header(), _raycast(6))
    with pytest.raises(pr.ProtocolError, match=message):
        pr.decode_frame(mutate(good))


def test_corrupt_compressed_data_is_rejected():
    good = bytearray(pr.encode_frame(_header(), _raycast(500), compress=True))
    good[-4:] = b"\xff\xff\xff\xff"
    with pytest.raises(pr.ProtocolError):
        pr.decode_frame(bytes(good))


def test_encoder_validation():
    with pytest.raises(pr.ProtocolError, match="missing"):
        pr.encode_frame(_header(), {"x": np.zeros(2, np.float32), "y": np.zeros(2, np.float32)})
    with pytest.raises(pr.ProtocolError, match="same length"):
        pr.encode_frame(_header(), {"x": np.zeros(2, np.float32), "y": np.zeros(3, np.float32), "z": np.zeros(2, np.float32)})


# ── cross-language: the browser decoder must read what Python wrote ────────────────

@pytest.mark.skipif(shutil.which("node") is None, reason="node not installed")
def test_js_decoder_reads_python_frames_identically(tmp_path):
    cases = {
        "semantic": (_header(), _semantic(64)),
        "semantic_compressed": (_header(frame=2**33 + 1), _semantic(200, seed=5)),
        "semantic_u32_tags": (_header(sensor_id=4000), _semantic(30, big_tags=True)),
        "raycast": (_header(sensor_frame=False), _raycast(50)),
        "empty": (_header(), _raycast(0)),
        "big_ids": (_header(), {**_semantic(3), "object_idx": np.array([16_777_217, 4_000_000_001, 7], np.uint32)}),
    }
    for name, (h, pts) in cases.items():
        (tmp_path / f"{name}.bin").write_bytes(pr.encode_frame(h, pts, compress=name.endswith("compressed"),
                                                                wall_time_unix=1_700_000_000.25))
    res = subprocess.run(["node", str(ROOT / "tests/js/decode_fixtures.mjs"), str(tmp_path)],
                         capture_output=True, text=True, timeout=60)
    assert res.returncode == 0, res.stderr
    got = json.loads(res.stdout)
    assert sorted(got) == sorted(f"{n}.bin" for n in cases)
    renames = {"object_idx": "objectIdx", "cos_inc": "cosInc"}
    for name, (h, pts) in cases.items():
        g = got[f"{name}.bin"]
        assert "error" not in g, g
        gh = g["header"]
        assert gh["frame"] == h.frame and gh["sensorId"] == h.sensor_id and gh["egoActorId"] == h.ego_actor_id
        assert gh["simTime"] == h.sim_time and gh["wallTimeUnix"] == 1_700_000_000.25
        assert gh["location"] == list(h.location) and gh["rotationPYR"] == list(h.rotation_pyr)
        assert gh["sensorFrame"] == h.sensor_frame and gh["numPoints"] == len(pts["x"])
        for k, arr in pts.items():
            assert g["points"][renames.get(k, k)] == arr.tolist(), (name, k)


@pytest.mark.skipif(shutil.which("node") is None, reason="node not installed")
def test_js_decoder_rejects_what_python_rejects(tmp_path):
    good = pr.encode_frame(_header(), _raycast(6))
    bad = {"magic": b"XXXX" + good[4:], "version": good[:4] + struct.pack("<H", 2) + good[6:],
           "truncated": good[:-1], "short": good[:20]}
    for name, b in bad.items():
        (tmp_path / f"{name}.bin").write_bytes(b)
    res = subprocess.run(["node", str(ROOT / "tests/js/decode_fixtures.mjs"), str(tmp_path)],
                         capture_output=True, text=True, timeout=60)
    got = json.loads(res.stdout)
    assert all("error" in got[f"{n}.bin"] for n in bad), got
