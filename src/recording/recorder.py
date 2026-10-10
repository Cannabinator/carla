"""Per-run writers: JSON Lines files filled by ONE writer thread from a bounded queue.

The simulation thread only puts immutable objects on the queue (``V2XFrame``, ``LidarFrame``, event dicts); all
serialisation and file I/O happen on the writer thread. If the queue is full the run FAILS with an error - research
data is never dropped silently (REFACTOR_HANDOFF 5.1). The data files contain no wall-clock times, so the same
scenario produces byte-identical ``stations.jsonl`` / ``v2x_links.jsonl`` / ``messages.jsonl``.

Files (all optional except the manifest; see ``RECORDABLE``):
    stations.jsonl      one line per V2X update: ground truth of every station
    v2x_links.jsonl     one line per link attempt (every field of LinkResult)
    messages.jsonl      one line per update and transmitting station: the messages it sent
    lidar_meta.jsonl    one line per LiDAR frame: sensor, pose, point count
    lidar/<sensor>/<frame>.bin   raw protocol-v3 frames, never subsampled
    events.jsonl        admin actions, session changes, LiDAR transitions, config changes (with wall time)
"""

from __future__ import annotations

import json
import logging
import queue
import shutil
import threading
from pathlib import Path
from typing import Any, Dict, Iterable, Optional, Set

from src.lidar.protocol import FrameHeader, encode_frame
from src.server.published import message_to_dict

logger = logging.getLogger(__name__)

RECORDABLE = ("links", "stations", "messages", "lidar_meta", "raw_lidar")
DEFAULT_RECORD = ("links", "stations", "messages", "lidar_meta")
LINK_FIELDS = ("frame", "sim_time", "tx_id", "rx_id", "msg_type", "link_type", "msg_id", "distance_m", "los", "tx_power_dbm",
               "path_loss_db", "shadowing_db", "rx_power_dbm", "rx_margin_db", "p_channel", "p_mac", "p_total", "delivered",
               "drop_cause")


class RunActiveError(Exception):
    """A run is already being recorded (not a RuntimeError: those mean 'the simulation thread is not running')."""


def dumps(obj: Any) -> bytes:
    return (json.dumps(obj, separators=(",", ":"), allow_nan=False) + "\n").encode()


def _num(v: Any) -> Any:
    return None if isinstance(v, float) and v != v else v


class RunRecorder:
    def __init__(self, run_dir: Path, record: Iterable[str] = DEFAULT_RECORD, queue_max: int = 400):
        bad = sorted(set(record) - set(RECORDABLE))
        if bad:
            raise ValueError(f"cannot record {', '.join(bad)}; choose from {', '.join(RECORDABLE)}")
        self.run_dir = Path(run_dir)
        self.record: Set[str] = set(record)
        self.queue_max = queue_max
        self.q: "queue.Queue[Optional[tuple]]" = queue.Queue(maxsize=queue_max)
        self.error: Optional[str] = None
        self.bytes_written = 0
        self.v2x_updates = 0
        self.links_written = 0
        self.lidar_frames = 0
        self.events_written = 0
        self._files: Dict[str, Any] = {}
        self._thread = threading.Thread(target=self._run, name="run-writer", daemon=True)
        self._closing = False
        self.run_dir.mkdir(parents=True, exist_ok=False)
        names = {"links": "v2x_links.jsonl", "stations": "stations.jsonl", "messages": "messages.jsonl", "lidar_meta": "lidar_meta.jsonl"}
        for key, fname in names.items():
            if key in self.record:
                self._files[key] = open(self.run_dir / fname, "wb")
        self._files["events"] = open(self.run_dir / "events.jsonl", "wb")
        if "raw_lidar" in self.record:
            (self.run_dir / "lidar").mkdir()
        self._thread.start()

    # ── producer side (any thread; never blocks) ─────────────────────────────────────────────────
    def submit(self, kind: str, payload: Any) -> bool:
        """Queue one item. Returns False (and records the error) when the queue is full: the run must then be stopped."""
        if self.error or self._closing:
            return False
        if kind == "v2x" and not ({"links", "stations", "messages"} & self.record):
            return True
        if kind == "lidar" and not ({"lidar_meta", "raw_lidar"} & self.record):
            return True
        try:
            self.q.put_nowait((kind, payload))
            return True
        except queue.Full:
            self.error = (f"writer queue full ({self.queue_max} items): the disk cannot keep up with the simulation; "
                          "the run is stopped instead of dropping data")
            logger.error(self.error)
            return False

    @property
    def queue_fill(self) -> float:
        return self.q.qsize() / self.queue_max

    # ── writer thread ────────────────────────────────────────────────────────────────────────────
    def _write(self, key: str, data: bytes) -> None:
        self._files[key].write(data)
        self.bytes_written += len(data)

    def _run(self) -> None:
        while True:
            item = self.q.get()
            if item is None:
                return
            try:
                kind, payload = item
                if kind == "v2x":
                    self._v2x(payload)
                elif kind == "lidar":
                    self._lidar(payload)
                elif kind == "event":
                    self._write("events", dumps(payload))
                    self.events_written += 1
            except Exception as exc:                       # a failing writer must fail the run, loudly
                self.error = f"writer failed: {type(exc).__name__}: {exc}"
                logger.exception("run writer failed")
                return

    def _v2x(self, frame) -> None:
        self.v2x_updates += 1
        f, t = frame.frame, frame.sim_time
        if "stations" in self._files:
            st = [{"id": s.station_id, "kind": s.kind, "x": s.x_m, "y": s.y_m, "z": s.z_m, "heading_deg": s.heading_deg,
                   "speed_mps": s.speed_mps, "tx_power_dbm": s.tx_power_dbm} for _, s in sorted(frame.stations.items())]
            self._write("stations", dumps({"frame": f, "sim_time": t, "stations": st}))
        if "links" in self._files:
            buf = []
            for l in frame.links:
                d = l._asdict()
                buf.append(dumps({k: _num(d[k]) for k in LINK_FIELDS}))
            self._write("links", b"".join(buf))
            self.links_written += len(frame.links)
        if "messages" in self._files:
            for tx, msgs in sorted(frame.sent.items()):
                if msgs:
                    self._write("messages", dumps({"frame": f, "sim_time": t, "tx_id": tx, "messages": [message_to_dict(m) for m in msgs]}))

    def _lidar(self, item) -> None:
        frame, raw = item                                  # LidarFrame, record_raw flag of its sensor
        self.lidar_frames += 1
        if "lidar_meta" in self._files:
            self._write("lidar_meta", dumps({"frame": frame.frame, "sim_time": frame.sim_time, "sensor_id": frame.sensor_id,
                                             "ego_actor_id": frame.ego_actor_id, "num_points": frame.num_points,
                                             "location": list(frame.location), "rotation_pyr": list(frame.rotation_pyr),
                                             "raw_recorded": bool(raw and "raw_lidar" in self.record)}))
        if raw and "raw_lidar" in self.record:
            d = self.run_dir / "lidar" / str(frame.sensor_id)
            d.mkdir(exist_ok=True)
            header = FrameHeader(frame=frame.frame, sim_time=frame.sim_time, sensor_id=frame.sensor_id, ego_actor_id=frame.ego_actor_id,
                                 location=frame.location, rotation_pyr=frame.rotation_pyr, sensor_frame=True)
            data = encode_frame(header, frame.points, compress=False, wall_time_unix=0.0)      # no wall clock: reproducible bytes
            (d / f"{frame.frame}.bin").write_bytes(data)
            self.bytes_written += len(data)

    # ── finish ───────────────────────────────────────────────────────────────────────────────────
    def close(self, timeout: float = 60.0) -> None:
        """Write everything that is queued, then close the files (waits for the writer thread)."""
        self._closing = True
        try:
            self.q.put(None, timeout=timeout)
        except queue.Full:
            self.error = self.error or "writer queue full at close"
        self._thread.join(timeout)
        for f in self._files.values():
            try:
                f.close()
            except OSError:
                pass

    def status(self) -> Dict[str, Any]:
        try:
            free = shutil.disk_usage(self.run_dir).free
        except OSError:
            free = None
        return {"dir": str(self.run_dir), "recorded": sorted(self.record), "v2x_updates": self.v2x_updates,
                "links_written": self.links_written, "lidar_frames": self.lidar_frames, "events_written": self.events_written,
                "bytes_written": self.bytes_written, "queue_fill": round(self.queue_fill, 3), "queue_max": self.queue_max,
                "disk_free_bytes": free, "error": self.error}
