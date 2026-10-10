"""Headless scenario runs on the fake world: same scenario -> byte-identical data files (REFACTOR_HANDOFF Phase 7)."""

import json
import subprocess
import sys
from pathlib import Path

import pytest

from src.lidar.protocol import decode_frame

ROOT = Path(__file__).resolve().parents[2]
SCENARIO = ROOT / "scenarios" / "crossing.yaml"
DATA_FILES = ("stations.jsonl", "v2x_links.jsonl", "messages.jsonl", "lidar_meta.jsonl")


def run(tmp_path, name, scenario=SCENARIO, extra=()):
    out = tmp_path / name
    res = subprocess.run([sys.executable, "-m", "src.cli", "run", str(scenario), "--runs-dir", str(out), *extra], cwd=ROOT,
                         capture_output=True, text=True, timeout=300)
    assert res.returncode == 0, res.stdout + res.stderr
    (d,) = list(out.iterdir())
    return d, json.loads((d / "manifest.json").read_text())


@pytest.fixture(scope="module")
def two_runs(tmp_path_factory):
    tmp = tmp_path_factory.mktemp("repro")
    return run(tmp, "a"), run(tmp, "b")


def test_the_same_scenario_twice_gives_byte_identical_data_files(two_runs):
    (da, ma), (db, mb) = two_runs
    for f in DATA_FILES:
        assert (da / f).read_bytes() == (db / f).read_bytes(), f"{f} differs between two runs of the same scenario"
        assert ma["files"][f]["sha256"] == mb["files"][f]["sha256"] and ma["files"][f]["lines"] > 0
    ra = sorted((da / "lidar").rglob("*.bin"))
    rb = sorted((db / "lidar").rglob("*.bin"))
    assert ra and [p.name for p in ra] == [p.name for p in rb]
    assert all(a.read_bytes() == b.read_bytes() for a, b in zip(ra, rb)), "raw LiDAR frames must be reproducible too"
    assert ma["run_id"] != mb["run_id"] and ma["files"]["events.jsonl"]["sha256"] != mb["files"]["events.jsonl"]["sha256"]   # wall times


def test_a_different_seed_gives_different_data(tmp_path, two_runs):
    sc = tmp_path / "seed8.yaml"
    sc.write_text(SCENARIO.read_text().replace("seed: 7", "seed: 8"))
    _, m8 = run(tmp_path, "c", sc)
    (_, ma), _ = two_runs
    assert m8["files"]["v2x_links.jsonl"]["sha256"] != ma["files"]["v2x_links.jsonl"]["sha256"]
    assert m8["seeds"]["world_seed"] == 8 and m8["seeds"]["dsrc_channel"] == 8


def test_manifest_says_what_produced_the_run(two_runs):
    (d, m), _ = two_runs
    assert m["status"] == "finished" and m["ticks"] == 400 and m["frames_recorded"] == 40 and m["stop_reason"] == "scenario finished"
    assert m["carla"]["fake"] is True and m["git"]["sha"] and m["config"]["realtime"] is False and m["config"]["seed"] == 7
    assert m["scenario"]["name"] == "crossing" and len(m["scenario"]["commands"]) == 5
    assert m["recorded"] == ["lidar_meta", "links", "messages", "raw_lidar", "stations"]
    assert m["files"]["lidar/"]["files"] > 100
    assert m["end_frame"] == m["start_frame"] + 400


def test_scripted_commands_ran_at_their_frames(two_runs):
    (d, m), _ = two_runs
    start = m["start_frame"]
    ev = [json.loads(l) for l in (d / "events.jsonl").read_text().splitlines()]
    by_kind = {e["kind"]: e for e in ev}
    assert by_kind["denm_injected"]["frame"] == start + 60
    assert by_kind["lidar_requested"]["frame"] == start + 100
    assert next(e for e in ev if e["kind"] == "rsu_changed")["frame"] == start + 200
    cc = by_kind["config_changed"]
    assert cc["frame"] == start + 300 and cc["data"]["field"] == "awareness_range_m" and cc["data"]["new"] == 50.0
    assert [c["frame"] for c in m["config_changes"]] == [start + 300]
    assert by_kind["lidar_destroyed"]["frame"] > start + 340
    assert ev[0]["kind"] == "run_started" and ev[-1]["kind"] == "run_stopped"


def test_the_recorded_links_show_the_denm_and_the_disabled_rsu(two_runs):
    (d, m), _ = two_runs
    links = [json.loads(l) for l in (d / "v2x_links.jsonl").read_text().splitlines()]
    start = m["start_frame"]
    denm = [l for l in links if l["msg_type"] == "DENM"]
    assert denm and all(start + 60 < l["frame"] <= start + 60 + 100 + 10 for l in denm), "DENM only between injection and its validity end"
    assert {l["link_type"] for l in denm} == {"I2V"}
    rsu5 = [l for l in links if l["tx_id"] == -5]
    assert any(l["frame"] < start + 200 for l in rsu5) and not any(l["frame"] > start + 210 for l in rsu5), "RSU 5 was disabled at frame 200"
    assert all(set(l) == {"frame", "sim_time", "tx_id", "rx_id", "msg_type", "link_type", "msg_id", "distance_m", "los", "tx_power_dbm",
                          "path_loss_db", "shadowing_db", "rx_power_dbm", "rx_margin_db", "p_channel", "p_mac", "p_total", "delivered",
                          "drop_cause"} for l in links[:50])
    awareness_after = [l for l in links if l["frame"] > start + 310 and l["drop_cause"] == "out_of_awareness_range"]
    assert awareness_after and max(l["distance_m"] for l in links if l["frame"] > start + 310 and l["delivered"] and l["link_type"] == "V2V") <= 50.0


def test_raw_lidar_frames_decode_and_match_the_meta(two_runs):
    (d, m), _ = two_runs
    meta = [json.loads(l) for l in (d / "lidar_meta.jsonl").read_text().splitlines()]
    assert all(x["raw_recorded"] for x in meta)
    f = meta[10]
    header, pts = decode_frame((d / "lidar" / str(f["sensor_id"]) / f"{f['frame']}.bin").read_bytes())
    assert header.frame == f["frame"] and header.num_points == f["num_points"] and header.ego_actor_id == f["ego_actor_id"]
    assert list(header.location) == f["location"] and header.wall_time_unix == 0.0
    assert [x["frame"] for x in meta] == sorted(x["frame"] for x in meta) and len({x["frame"] for x in meta}) == len(meta)


def test_a_bad_scenario_is_refused_with_a_message(tmp_path):
    bad = tmp_path / "bad.yaml"
    bad.write_text("name: x\nduration_frames: 0\nbogus: 1\n")
    res = subprocess.run([sys.executable, "-m", "src.cli", "run", str(bad), "--fake"], cwd=ROOT, capture_output=True, text=True, timeout=60)
    assert res.returncode != 0 and ("duration_frames" in res.stderr or "bogus" in res.stderr)
