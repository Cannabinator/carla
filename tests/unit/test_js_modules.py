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
    (path / "world_config_schema.json").write_text(json.dumps(WorldConfig.model_json_schema()))
    (path / "lidar_schema.json").write_text(json.dumps(lidar_schema()))


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
