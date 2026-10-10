"""RunRecorder: files, reproducible bytes, the bounded queue policy, manifest helpers."""

import json
import threading
import time
from pathlib import Path

import numpy as np
import pytest

from src.lidar.config import LidarConfig
from src.lidar.protocol import decode_frame
from src.lidar.sensor import LidarFrame
from src.recording.manifest import git_info, list_runs, read_manifest, run_id, sha256_file, slug, utc_now, write_manifest
from src.recording.recorder import LINK_FIELDS, RunRecorder
from tests.contract.v2x_samples import sample_frame


def lidar_frame(frame=10, n=30):
    pts = {k: np.arange(n, dtype=np.float32) for k in "xyz"}
    pts.update({"cos_inc": np.ones(n, np.float32), "object_idx": np.arange(n, dtype=np.uint32), "tag": np.full(n, 14, np.uint32)})
    return LidarFrame(3, frame, frame * 0.05, 7, (1.0, 2.0, 3.0), (0.0, 90.0, 0.0), pts, LidarConfig())


def lines(path):
    return [json.loads(l) for l in Path(path).read_text().splitlines()]


def test_files_have_the_documented_content(tmp_path):
    rec = RunRecorder(tmp_path / "r", queue_max=50)
    f = sample_frame(10)
    assert rec.submit("v2x", f)
    rec.submit("event", {"seq": 1, "kind": "x", "frame": 10, "sim_time": 0.5, "wall_time": 1.0, "data": {}})
    rec.submit("lidar", (lidar_frame(), False))
    rec.close()
    assert rec.error is None
    st = lines(tmp_path / "r/stations.jsonl")
    assert len(st) == 1 and st[0]["frame"] == 10 and [s["id"] for s in st[0]["stations"]] == [-1, 101, 102, 300]
    links = lines(tmp_path / "r/v2x_links.jsonl")
    assert len(links) == 4 and set(links[0]) == set(LINK_FIELDS) and links[1]["drop_cause"] == "channel" and links[0]["frame"] == 10
    msgs = lines(tmp_path / "r/messages.jsonl")
    assert {m["tx_id"] for m in msgs} == {-1, 101, 102} and msgs[0]["messages"][0]["type"] in ("DENM", "BSMCore")
    meta = lines(tmp_path / "r/lidar_meta.jsonl")
    assert meta == [{"frame": 10, "sim_time": 0.5, "sensor_id": 3, "ego_actor_id": 7, "num_points": 30, "location": [1.0, 2.0, 3.0],
                     "rotation_pyr": [0.0, 90.0, 0.0], "raw_recorded": False}]
    assert lines(tmp_path / "r/events.jsonl")[0]["kind"] == "x"
    assert not (tmp_path / "r/lidar").exists(), "raw LiDAR was not requested"
    s = rec.status()
    assert s["v2x_updates"] == 1 and s["links_written"] == 4 and s["bytes_written"] > 500 and s["queue_fill"] == 0


def test_raw_lidar_is_written_unsubsampled_and_reproducible(tmp_path):
    a, b = RunRecorder(tmp_path / "a", ("lidar_meta", "raw_lidar")), RunRecorder(tmp_path / "b", ("raw_lidar",))
    for r in (a, b):
        r.submit("lidar", (lidar_frame(11, 500), True))
        r.submit("lidar", (lidar_frame(12, 500), False))          # sensor not marked record_raw: meta only
        r.close()
    pa, pb = tmp_path / "a/lidar/3/11.bin", tmp_path / "b/lidar/3/11.bin"
    assert pa.read_bytes() == pb.read_bytes(), "no wall clock in the file: the bytes are reproducible"
    header, pts = decode_frame(pa.read_bytes())
    assert header.num_points == 500 and header.frame == 11 and header.wall_time_unix == 0.0 and np.array_equal(pts["object_idx"], np.arange(500))
    assert not (tmp_path / "a/lidar/3/12.bin").exists()
    assert lines(tmp_path / "a/lidar_meta.jsonl")[0]["raw_recorded"] is True and lines(tmp_path / "a/lidar_meta.jsonl")[1]["raw_recorded"] is False


def test_same_input_gives_byte_identical_data_files(tmp_path):
    for name in ("one", "two"):
        r = RunRecorder(tmp_path / name)
        for k in range(5):
            r.submit("v2x", sample_frame(10 + 10 * k, extra_links=3))
        r.close()
    for f in ("stations.jsonl", "v2x_links.jsonl", "messages.jsonl"):
        assert sha256_file(tmp_path / "one" / f) == sha256_file(tmp_path / "two" / f), f
    assert sha256_file(tmp_path / "one/v2x_links.jsonl")["lines"] == 5 * 7


def test_only_requested_kinds_are_written(tmp_path):
    r = RunRecorder(tmp_path / "r", ("links",))
    r.submit("v2x", sample_frame())
    r.close()
    assert (tmp_path / "r/v2x_links.jsonl").exists() and not (tmp_path / "r/stations.jsonl").exists() and not (tmp_path / "r/messages.jsonl").exists()
    with pytest.raises(ValueError, match="cannot record nonsense"):
        RunRecorder(tmp_path / "x", ("links", "nonsense"))


def test_a_full_queue_fails_the_run_instead_of_dropping_data(tmp_path):
    gate = threading.Event()
    r = RunRecorder(tmp_path / "r", queue_max=3)
    original = r._v2x
    r._v2x = lambda frame: (gate.wait(5), original(frame))          # a writer stuck on a slow disk
    accepted = [r.submit("v2x", sample_frame(10 * k)) for k in range(1, 12)]
    assert accepted[0] is True and accepted[-1] is False and r.error and "queue full" in r.error
    assert r.submit("v2x", sample_frame(999)) is False, "after the failure nothing more is accepted"
    gate.set()
    r.close()
    written = lines(tmp_path / "r/stations.jsonl")
    assert [w["frame"] for w in written] == [10 * k for k in range(1, len(written) + 1)], "what was accepted is complete and in order"


def test_a_writer_error_is_reported(tmp_path):
    r = RunRecorder(tmp_path / "r")
    r.submit("v2x", object())                                       # not a frame: the writer cannot serialise it
    r.close()
    assert r.error and "writer failed" in r.error


def test_directory_must_be_new(tmp_path):
    RunRecorder(tmp_path / "r").close()
    with pytest.raises(FileExistsError):
        RunRecorder(tmp_path / "r")


def test_status_reports_disk_and_queue(tmp_path):
    r = RunRecorder(tmp_path / "r", queue_max=10)
    s = r.status()
    r.close()
    assert s["queue_max"] == 10 and s["disk_free_bytes"] > 0 and s["recorded"] == sorted(("links", "stations", "messages", "lidar_meta"))


# ── manifest helpers ──────────────────────────────────────────────────────────────────────────────

def test_run_id_slug_and_manifest_roundtrip(tmp_path):
    assert slug("My run: 1/2 !") == "My-run-1-2" and slug("///") == "run" and len(slug("x" * 200)) == 48
    rid = run_id(utc_now(), "abc1234", "cross test")
    assert rid.endswith("_abc1234_cross-test") and rid[8] == "T"
    d = tmp_path / rid
    d.mkdir()
    write_manifest(d, {"run_id": rid, "name": "cross test", "status": "running", "started_utc": "2026-10-10T00:00:00+00:00"})
    assert read_manifest(d)["status"] == "running" and not (d / "manifest.json.tmp").exists()
    write_manifest(d, {"run_id": rid, "name": "cross test", "status": "finished", "frames_recorded": 5})
    listed = list_runs(tmp_path)
    assert listed[0]["run_id"] == rid and listed[0]["status"] == "finished" and listed[0]["frames"] == 5
    (tmp_path / "junk").mkdir()
    (tmp_path / "junk/manifest.json").write_text("{not json")
    assert len(list_runs(tmp_path)) == 1 and list_runs(tmp_path / "nothing") == []


def test_git_info_has_the_fields_even_outside_a_checkout(tmp_path):
    here = git_info()
    assert set(here) == {"sha", "short_sha", "dirty"} and len(here["sha"]) in (7, 40)
    outside = git_info(tmp_path)
    assert outside == {"sha": "unknown", "short_sha": "unknown", "dirty": None}
    _ = time
