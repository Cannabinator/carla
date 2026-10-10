"""Run the Node unit tests of the browser modules (tests/js/*.test.mjs).

Python writes the fixtures the JS side must agree with (poses and transformed points from the verified Python
transform, the JSON schemas the forms are generated from), so the two languages cannot drift apart silently.
"""

import json
import shutil
import subprocess
from pathlib import Path

import numpy as np
import pytest

from src.lidar.config import lidar_schema
from src.lidar.transforms import to_world, transform_matrix
from src.server.world_config import WorldConfig

ROOT = Path(__file__).resolve().parents[2]

pytestmark = pytest.mark.skipif(shutil.which("node") is None, reason="node not installed")


def write_fixtures(path: Path) -> None:
    rng = np.random.default_rng(11)
    poses = [((1.5, -2.0, 0.5), (0.0, 0.0, 0.0)), ((100.0, 50.0, 2.4), (10.0, 30.0, 5.0)),
             ((-5.0, 7.0, 1.0), (-35.0, -120.0, 20.0)), ((0.0, 0.0, 0.0), (90.0, 90.0, 90.0))]
    poses += [(tuple(rng.uniform(-200, 200, 3)), tuple(rng.uniform(-80, 80, 3))) for _ in range(12)]
    out = []
    for loc, pyr in poses:
        pts = rng.uniform(-30, 30, (6, 3)).astype(np.float32)
        out.append({"location": list(map(float, loc)), "pyr": list(map(float, pyr)),
                    "matrix": transform_matrix(loc, pyr).reshape(-1).tolist(),
                    "points": pts.tolist(), "world": to_world(pts, loc, pyr).tolist()})
    (path / "coords.json").write_text(json.dumps({"poses": out}))
    write_lidar_frames(path)
    (path / "world_config_schema.json").write_text(json.dumps(WorldConfig.model_json_schema()))
    (path / "lidar_schema.json").write_text(json.dumps(lidar_schema()))


def write_lidar_frames(path: Path) -> None:
    """Frames with a known pitched + rolled pose; the expected render positions are computed with the Python
    transform (rotation only, origin at the sensor) and the CARLA -> Three y mirror."""
    from src.lidar.protocol import FrameHeader, encode_frame
    rng = np.random.default_rng(5)
    n = 200
    pts = {"x": rng.uniform(-40, 40, n).astype(np.float32), "y": rng.uniform(-40, 40, n).astype(np.float32),
           "z": rng.uniform(-3, 8, n).astype(np.float32), "cos_inc": rng.random(n).astype(np.float32),
           "object_idx": rng.choice([0, 7, 42], n).astype(np.uint32), "tag": rng.choice([1, 3, 14], n).astype(np.uint32)}
    loc, pyr = (120.5, -33.25, 2.4), (8.0, 35.0, -4.0)
    h = FrameHeader(frame=77, sim_time=3.85, sensor_id=3, ego_actor_id=7, location=loc, rotation_pyr=pyr)
    (path / "lidar_semantic.bin").write_bytes(encode_frame(h, pts, wall_time_unix=1.0))
    rot = to_world(np.stack([pts["x"], pts["y"], pts["z"]], 1), (0.0, 0.0, 0.0), pyr)
    expected = rot * np.array([1.0, -1.0, 1.0])
    ray = {k: pts[k] for k in ("x", "y", "z")}
    ray["intensity"] = rng.random(n).astype(np.float32)
    (path / "lidar_raycast.bin").write_bytes(encode_frame(h, ray, wall_time_unix=1.0))
    (path / "lidar_expected.json").write_text(json.dumps({
        "positions": expected.tolist(), "object_idx": pts["object_idx"].tolist(), "tag": pts["tag"].tolist(),
        "intensity": ray["intensity"].tolist(), "range": np.linalg.norm(np.stack([pts["x"], pts["y"], pts["z"]], 1), axis=1).tolist(),
        "ego": 7, "pyr": list(pyr), "loc": list(loc)}))


def test_node_unit_tests_pass(tmp_path):
    write_fixtures(tmp_path)
    files = sorted(str(f) for f in (ROOT / "tests/js").glob("*.test.mjs"))
    assert files
    res = subprocess.run(["node", "--test", *files], capture_output=True, text=True, timeout=180,
                         env={**__import__("os").environ, "JS_FIXTURES": str(tmp_path)})
    assert res.returncode == 0, res.stdout[-4000:] + res.stderr[-2000:]
    assert "# fail 0" in res.stdout
    skipped = [ln for ln in res.stdout.splitlines() if ln.startswith("# skipped")]
    assert skipped == ["# skipped 0"], f"JS tests were skipped: {skipped}"
