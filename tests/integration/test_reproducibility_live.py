"""Reproducibility on a REAL CARLA (marker: carla): the same scenario twice must give identical data files.

If this fails, the divergence is real information about CARLA / the Traffic Manager (physics and TM are not guaranteed to be
deterministic): find the first diverging frame, write it and the suspected cause into docs/science/limitations.md, and do
not paper over it. The test prints the first diverging line.
"""

import json
import os
import subprocess
import sys
from pathlib import Path

import pytest

pytestmark = pytest.mark.carla

ROOT = Path(__file__).resolve().parents[2]


def first_difference(a: Path, b: Path):
    la, lb = a.read_text().splitlines(), b.read_text().splitlines()
    for i, (x, y) in enumerate(zip(la, lb)):
        if x != y:
            fx, fy = json.loads(x).get("frame"), json.loads(y).get("frame")
            return f"{a.name}: line {i + 1} (frame {fx} vs {fy})\n  {x[:300]}\n  {y[:300]}"
    return None if len(la) == len(lb) else f"{a.name}: {len(la)} vs {len(lb)} lines"


def run(tmp_path, name):
    sc = ROOT / "scenarios" / "live_repro.yaml"
    host, port = os.environ.get("CARLA_HOST", "localhost"), os.environ.get("CARLA_PORT", "2000")
    text = sc.read_text().replace("host: localhost", f"host: {host}").replace("port: 2000", f"port: {port}")
    scenario = tmp_path / f"{name}.yaml"
    scenario.write_text(text)
    res = subprocess.run([sys.executable, "-m", "src.cli", "run", str(scenario), "--runs-dir", str(tmp_path / name)], cwd=ROOT,
                         capture_output=True, text=True, timeout=1800)
    assert res.returncode == 0, res.stdout + res.stderr
    (d,) = list((tmp_path / name).iterdir())
    return d, json.loads((d / "manifest.json").read_text())


def test_same_scenario_twice_on_carla_gives_identical_data(tmp_path, carla_client):
    (da, ma), (db, mb) = run(tmp_path, "a"), run(tmp_path, "b")
    assert ma["carla"]["fake"] is False and ma["status"] == "finished"
    diffs = [d for f in ("stations.jsonl", "v2x_links.jsonl", "messages.jsonl")
             if (d := first_difference(da / f, db / f))]
    assert not diffs, ("the two runs diverge; record the frame and the cause in docs/science/limitations.md:\n" + "\n".join(diffs))
    for f in ("stations.jsonl", "v2x_links.jsonl", "messages.jsonl"):
        assert ma["files"][f]["sha256"] == mb["files"][f]["sha256"]
