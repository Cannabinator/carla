"""LiDAR sensor lifecycle, frame-synchronised collection, budget and ownership (REFACTOR_HANDOFF 5.5).

A sensor is a CARLA actor attached to a vehicle and a costly resource: it exists only while someone asked
for it. *Viewers* (WebSocket connections) are separate from sensors; many viewers share one sensor.

State machine of one sensor (every transition is logged with the frame at which it happened)::

    requested --(next tick boundary)--> spawning --> warming_up --(first measurement of the frame)--> active
    active --reconfigure--> reconfiguring (destroy + respawn at a tick boundary) --> warming_up --> active
    active/warming_up --release | owner gone | auto-release--> stopping --> destroyed
    any --(spawn failure, no data for too long, actor gone)--> error

Threading: everything that touches CARLA (``request`` ... ``collect``, ``shutdown``) runs on the simulation
thread only (the world executes them through ``submit``). ``list_sensors``, ``get_frame``, ``attach_viewer``,
``detach_viewer`` and ``budget`` are safe from any thread: they read published snapshots / use a small lock.

Frame synchronisation (L3): CARLA delivers measurements on its own thread into a queue. After
``world.tick()`` returned frame F, ``collect(F, ...)`` takes measurements from the queue and keeps only the one
whose own ``frame`` equals F (older ones are discarded). A frame is therefore never mixed with another one.
"""

from __future__ import annotations

import logging
import queue
import threading
import time
from dataclasses import dataclass
from enum import Enum
from typing import Any, Callable, Dict, List, Optional, Protocol, Tuple

import numpy as np

from .config import SENSOR_SEMANTIC, LidarConfig

logger = logging.getLogger(__name__)


class SensorState(str, Enum):
    REQUESTED = "requested"
    SPAWNING = "spawning"
    WARMING_UP = "warming_up"
    ACTIVE = "active"
    RECONFIGURING = "reconfiguring"
    STOPPING = "stopping"
    DESTROYED = "destroyed"
    ERROR = "error"


_LIVE = (SensorState.REQUESTED, SensorState.SPAWNING, SensorState.WARMING_UP, SensorState.ACTIVE,
         SensorState.RECONFIGURING, SensorState.STOPPING)       # states that hold budget


# ── errors (the API layer maps them to HTTP codes) ───────────────────────────────────────────────────────

class LidarError(Exception):
    pass


class LidarBudgetExceeded(LidarError):
    """409 lidar_budget_exceeded"""
    def __init__(self, reason: str, requested_pps: int, available_pps: int, used_pps: int, max_pps: int,
                 sensors: int, max_sensors: int):
        super().__init__(f"LiDAR budget exceeded ({reason})")
        self.detail = {"error": "lidar_budget_exceeded", "reason": reason, "requested_pps": requested_pps,
                       "available_pps": available_pps, "used_pps": used_pps, "max_pps": max_pps,
                       "sensors": sensors, "max_sensors": max_sensors}


class LidarForbidden(LidarError):
    """403"""


class LidarNotFound(LidarError):
    """404"""


# ── who is asking ─────────────────────────────────────────────────────────────────────────────────────

@dataclass(frozen=True)
class Principal:
    """Admin: any actor. Vehicle: only the actor of its own session. Observer: may not create sensors."""
    kind: str                         # "admin" | "vehicle" | "observer"
    session_id: Optional[str] = None
    actor_id: Optional[int] = None
    label: Optional[str] = None       # shown as the owner (client name). The session id is a credential-adjacent
                                      # identifier and is never put in anything an observer can read.

    @property
    def owner_label(self) -> str:
        return "admin" if self.kind == "admin" else (self.label or self.kind)


ADMIN = Principal("admin")


# ── backend interface (CARLA calls live behind it) ────────────────────────────────────────────────────

class SensorHandle(Protocol):
    def listen(self, callback: Callable[[Any], None]) -> None: ...
    def stop(self) -> None: ...
    def destroy(self) -> None: ...


class SensorBackend(Protocol):
    def spawn(self, actor_id: int, config: LidarConfig) -> SensorHandle:
        """Create the sensor attached to the actor; raise on failure."""
        ...


# ── data ──────────────────────────────────────────────────────────────────────────────────────────────

_SEMANTIC_DTYPE = np.dtype([("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("cos_inc", "<f4"),
                            ("object_idx", "<u4"), ("tag", "<u4")])
_RAYCAST_DTYPE = np.dtype([("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("intensity", "<f4")])


@dataclass(frozen=True)
class LidarFrame:
    """One measurement, sensor frame, exactly as delivered (never decimated, rounded or transformed)."""
    sensor_id: int
    frame: int
    sim_time: float
    ego_actor_id: int
    location: Tuple[float, float, float]            # sensor pose in the world at measurement time
    rotation_pyr: Tuple[float, float, float]
    points: Dict[str, np.ndarray]                   # read-only arrays: x, y, z + the type's extra fields
    config: LidarConfig

    @property
    def num_points(self) -> int:
        return len(self.points["x"])


def parse_measurement(config: LidarConfig, measurement: Any) -> Tuple[Tuple[float, float, float],
                                                                      Tuple[float, float, float],
                                                                      Dict[str, np.ndarray]]:
    """CARLA measurement -> (location, (pitch, yaw, roll), field arrays). Raises ValueError if malformed."""
    dtype = _SEMANTIC_DTYPE if config.sensor_type == SENSOR_SEMANTIC else _RAYCAST_DTYPE
    raw = bytes(measurement.raw_data)
    if len(raw) % dtype.itemsize:
        raise ValueError(f"raw_data length {len(raw)} is not a multiple of {dtype.itemsize}")
    records = np.frombuffer(raw, dtype=dtype)
    points = {name: records[name].copy() for name in dtype.names or ()}
    for a in points.values():
        a.flags.writeable = False
    t = measurement.transform
    return ((float(t.location.x), float(t.location.y), float(t.location.z)),
            (float(t.rotation.pitch), float(t.rotation.yaw), float(t.rotation.roll)), points)


class _LatestQueue:
    """Thread-safe bounded queue for the CARLA callback thread; the oldest item is dropped when full."""
    def __init__(self, maxsize: int = 8):
        self._q: "queue.Queue[Any]" = queue.Queue(maxsize=maxsize)

    def put(self, item: Any) -> None:
        while True:
            try:
                self._q.put_nowait(item)
                return
            except queue.Full:
                try:
                    self._q.get_nowait()
                except queue.Empty:
                    pass

    def get(self, timeout: float) -> Any:
        return self._q.get(timeout=max(timeout, 0.0))

    def get_nowait(self) -> Any:
        return self._q.get_nowait()


@dataclass
class _Sensor:
    sensor_id: int
    actor_id: int
    owner: str
    owner_session: Optional[str]
    config: LidarConfig
    pinned: bool
    record_raw: bool
    created_frame: int
    state: SensorState = SensorState.REQUESTED
    handle: Optional[SensorHandle] = None
    inbox: Optional[_LatestQueue] = None
    last_frame: Optional[int] = None
    missed: int = 0                      # consecutive frames without a measurement
    idle_since: float = 0.0              # wall clock, for auto-release
    pending_config: Optional[LidarConfig] = None
    stop_reason: Optional[str] = None
    error: Optional[str] = None


@dataclass
class LidarLimits:
    max_sensors: int = 4
    max_points_per_second: int = 2_000_000
    keep_alive_s: float = 10.0


class LidarManager:
    MAX_CONSECUTIVE_MISSES = 40          # ~2 s at 20 FPS: a sensor that never delivers goes to ERROR

    def __init__(self, backend: SensorBackend, limits: LidarLimits, fixed_delta_seconds: float, *,
                 clock: Callable[[], float] = time.monotonic, frame_timeout_s: float = 0.5):
        self._backend = backend
        self.limits = limits
        self.dt = fixed_delta_seconds
        self._clock = clock
        self._frame_timeout = frame_timeout_s
        self._sensors: Dict[int, _Sensor] = {}
        self._next_id = 1
        self._viewers: Dict[int, set] = {}
        self._viewer_lock = threading.Lock()
        self._events: List[Dict[str, Any]] = []
        self._events_lock = threading.Lock()
        self._current_frame = 0
        # published snapshots (replaced, never mutated)
        self._latest: Dict[int, LidarFrame] = {}
        self._info: List[Dict[str, Any]] = []
        self._budget: Dict[str, Any] = {}
        self.info_version = 0                    # bumped on every change of the published sensor list
        self._publish_info()

    # ── helpers ────────────────────────────────────────────────────────────────────────────────────
    def _event(self, sensor: _Sensor, to_state: SensorState, reason: str = "") -> None:
        frm = sensor.state
        sensor.state = to_state
        rec = {"frame": self._current_frame, "sensor_id": sensor.sensor_id, "actor_id": sensor.actor_id,
               "owner": sensor.owner, "from": frm.value, "to": to_state.value, "reason": reason}
        with self._events_lock:
            self._events.append(rec)
        logger.info("LiDAR sensor %d: %s -> %s %s", sensor.sensor_id, frm.value, to_state.value, reason)

    def drain_events(self) -> List[Dict[str, Any]]:
        """State transitions since the last call (for the run recorder / UI event stream)."""
        with self._events_lock:
            out, self._events = self._events, []
        return out

    def _used(self, exclude: Optional[int] = None) -> Tuple[int, int]:
        live = [s for s in self._sensors.values() if s.state in _LIVE and s.sensor_id != exclude]
        # a pending reconfiguration holds the larger of the two budgets until the respawn
        return len(live), sum(max(s.config.points_per_second,
                                  s.pending_config.points_per_second if s.pending_config else 0) for s in live)

    def _check_budget(self, new_pps: int, exclude: Optional[int] = None, adds_sensor: bool = True) -> None:
        n, used = self._used(exclude)
        lim = self.limits
        available = max(lim.max_points_per_second - used, 0)
        if adds_sensor and n + 1 > lim.max_sensors:
            raise LidarBudgetExceeded("max_sensors", new_pps, available, used, lim.max_points_per_second, n, lim.max_sensors)
        if new_pps > available:
            raise LidarBudgetExceeded("max_points_per_second", new_pps, available, used, lim.max_points_per_second, n, lim.max_sensors)

    def _get(self, sensor_id: int) -> _Sensor:
        s = self._sensors.get(sensor_id)
        if s is None or s.state in (SensorState.DESTROYED,):
            raise LidarNotFound(f"no LiDAR sensor {sensor_id}")
        return s

    @staticmethod
    def _may_modify(principal: Principal, sensor: _Sensor) -> None:
        if principal.kind == "admin":
            return
        if principal.kind == "vehicle" and principal.session_id is not None and principal.session_id == sensor.owner_session:
            return
        raise LidarForbidden("only the owner session or an admin may change this sensor")

    # ── requests (simulation thread) ──────────────────────────────────────────────────────────────
    def request(self, principal: Principal, actor_id: int, config: Optional[LidarConfig] = None, *,
                record_raw: bool = False, pinned: bool = False) -> int:
        """Ask for a sensor on ``actor_id``. Returns the sensor id (state ``requested``; it is spawned at the
        next tick boundary)."""
        if principal.kind == "observer":
            raise LidarForbidden("observers cannot create LiDAR sensors")
        if principal.kind == "vehicle" and (principal.actor_id is None or principal.actor_id != actor_id):
            raise LidarForbidden("a vehicle session may create a sensor only on its own vehicle")
        if (pinned or record_raw) and principal.kind != "admin":
            raise LidarForbidden("pinned and record_raw sensors are admin only")
        config = config or LidarConfig()
        self._check_budget(config.points_per_second)
        s = _Sensor(sensor_id=self._next_id, actor_id=actor_id, owner=principal.owner_label,
                    owner_session=principal.session_id, config=config, pinned=pinned, record_raw=record_raw,
                    created_frame=self._current_frame, idle_since=self._clock())
        self._next_id += 1
        self._sensors[s.sensor_id] = s
        self._event(s, SensorState.REQUESTED, "created")
        self._publish_info()
        return s.sensor_id

    def reconfigure(self, principal: Principal, sensor_id: int, config: LidarConfig) -> None:
        """Replace the configuration: the CARLA sensor is destroyed and respawned at the next tick boundary
        (attributes cannot change on a live sensor)."""
        s = self._get(sensor_id)
        self._may_modify(principal, s)
        if s.state not in (SensorState.ACTIVE, SensorState.WARMING_UP, SensorState.REQUESTED, SensorState.ERROR):
            raise LidarError(f"cannot reconfigure a sensor in state {s.state.value}")
        self._check_budget(config.points_per_second, exclude=sensor_id, adds_sensor=False)
        s.pending_config = config
        s.idle_since = self._clock()
        if s.state == SensorState.REQUESTED:
            s.config, s.pending_config = config, None          # not spawned yet: just replace
        else:
            self._event(s, SensorState.RECONFIGURING, "config changed")
        self._publish_info()

    def set_flags(self, principal: Principal, sensor_id: int, pinned: Optional[bool] = None,
                  record_raw: Optional[bool] = None) -> None:
        """Pin a sensor (never auto-released) / mark it for raw recording. Admin only (D9: pinning is a resource decision)."""
        s = self._get(sensor_id)
        if principal.kind != "admin":
            raise LidarForbidden("only an admin may pin a sensor or mark it for raw recording")
        if s.state not in _LIVE or s.state == SensorState.STOPPING:
            raise LidarError(f"cannot change a sensor in state {s.state.value}")
        if pinned is not None:
            s.pinned = pinned
        if record_raw is not None:
            s.record_raw = record_raw
        s.idle_since = self._clock()
        self._publish_info()

    def release(self, principal: Principal, sensor_id: int) -> None:
        s = self._get(sensor_id)
        self._may_modify(principal, s)
        self._begin_stop(s, "released")
        self._publish_info()

    def release_owned_by(self, session_id: str, reason: str = "owner session ended") -> int:
        n = 0
        for s in list(self._sensors.values()):
            if s.owner_session == session_id and s.state in _LIVE and s.state != SensorState.STOPPING:
                self._begin_stop(s, reason)
                n += 1
        if n:
            self._publish_info()
        return n

    def on_actor_removed(self, actor_id: int) -> int:
        """The vehicle a sensor is attached to is gone."""
        n = 0
        for s in list(self._sensors.values()):
            if s.actor_id == actor_id and s.state in _LIVE and s.state != SensorState.STOPPING:
                self._begin_stop(s, "vehicle removed")
                n += 1
        if n:
            self._publish_info()
        return n

    def _begin_stop(self, s: _Sensor, reason: str) -> None:
        if s.state in (SensorState.STOPPING, SensorState.DESTROYED):
            return
        s.stop_reason = reason
        self._event(s, SensorState.STOPPING, reason)

    # ── viewers (any thread) ──────────────────────────────────────────────────────────────────────
    def attach_viewer(self, sensor_id: int, viewer_id: str) -> None:
        with self._viewer_lock:
            self._viewers.setdefault(sensor_id, set()).add(viewer_id)

    def detach_viewer(self, sensor_id: int, viewer_id: str) -> None:
        with self._viewer_lock:
            self._viewers.get(sensor_id, set()).discard(viewer_id)

    def viewer_count(self, sensor_id: int) -> int:
        with self._viewer_lock:
            return len(self._viewers.get(sensor_id, ()))

    # ── per-tick step (simulation thread) ─────────────────────────────────────────────────────────
    def collect(self, frame: int, sim_time: float) -> List[LidarFrame]:
        """Call right after ``world.tick()`` returned ``frame``. Advances every sensor's state machine at
        this tick boundary and returns the measurements that belong to exactly this frame."""
        self._current_frame = frame
        now = self._clock()

        # 1. stop what has to stop, respawn what was reconfigured, spawn what was requested. A sensor spawned
        #    now cannot have a measurement of the frame that was just ticked: it is not waited for.
        self.flush_stopping()
        fresh = set()
        for s in list(self._sensors.values()):
            if s.state == SensorState.RECONFIGURING:
                self._destroy(s)
                if s.pending_config is not None:
                    s.config, s.pending_config = s.pending_config, None
                self._spawn(s)
                fresh.add(s.sensor_id)
            elif s.state == SensorState.REQUESTED:
                self._spawn(s)
                fresh.add(s.sensor_id)

        # 2. collect this frame's measurement from every sensor that can deliver
        frames: List[LidarFrame] = []
        deadline = now + self._frame_timeout
        for s in list(self._sensors.values()):
            if s.state not in (SensorState.WARMING_UP, SensorState.ACTIVE) or s.inbox is None \
                    or s.sensor_id in fresh:
                continue
            m = self._measurement_for_frame(s, frame, deadline)
            if m is None:
                s.missed += 1
                if s.missed == 1 or s.missed % 20 == 0:
                    logger.warning("LiDAR sensor %d: no measurement for frame %d (%d in a row)", s.sensor_id, frame, s.missed)
                if s.missed >= self.MAX_CONSECUTIVE_MISSES:
                    self._fail(s, f"no data for {s.missed} frames")
                continue
            try:
                loc, rot, points = parse_measurement(s.config, m)
            except Exception as exc:
                self._fail(s, f"malformed measurement: {exc}")
                continue
            s.missed = 0
            s.last_frame = frame
            if s.state == SensorState.WARMING_UP:
                self._event(s, SensorState.ACTIVE, f"first measurement at frame {frame}")
            frames.append(LidarFrame(s.sensor_id, frame, float(sim_time), s.actor_id, loc, rot, points, s.config))

        # 3. auto-release sensors nobody watches (wall clock: a UI resource policy)
        for s in list(self._sensors.values()):
            if s.state in (SensorState.WARMING_UP, SensorState.ACTIVE):
                if s.pinned or s.record_raw or self.viewer_count(s.sensor_id) > 0:
                    s.idle_since = now
                elif now - s.idle_since >= self.limits.keep_alive_s:
                    self._begin_stop(s, f"auto-release: no viewers for {self.limits.keep_alive_s:g} s")

        self._latest = {f.sensor_id: f for f in frames}
        self._publish_info()
        return frames

    def flush_stopping(self) -> int:
        """Destroy sensors that are STOPPING now (also used before their vehicle is destroyed)."""
        n = 0
        for s in list(self._sensors.values()):
            if s.state == SensorState.STOPPING:
                self._destroy(s)
                self._event(s, SensorState.DESTROYED, s.stop_reason or "")
                n += 1
        if n:
            self._publish_info()
        return n

    def _measurement_for_frame(self, s: _Sensor, frame: int, deadline: float) -> Optional[Any]:
        assert s.inbox is not None
        while True:
            remaining = deadline - self._clock()
            try:
                m = s.inbox.get(timeout=remaining) if remaining > 0 else s.inbox.get_nowait()
            except queue.Empty:
                return None
            if m.frame == frame:
                return m
            if m.frame > frame:          # cannot happen in synchronous mode; never mix frames
                logger.warning("LiDAR sensor %d: measurement of future frame %d at frame %d", s.sensor_id, m.frame, frame)
                return None
            # m.frame < frame: a late measurement of an older frame, discard

    def _spawn(self, s: _Sensor) -> None:
        self._event(s, SensorState.SPAWNING, "")
        try:
            handle = self._backend.spawn(s.actor_id, s.config)
            inbox = _LatestQueue()
            handle.listen(inbox.put)
        except Exception as exc:
            self._fail(s, f"spawn failed: {exc}")
            return
        s.handle, s.inbox, s.missed, s.idle_since = handle, inbox, 0, self._clock()
        self._event(s, SensorState.WARMING_UP, "")

    def _destroy(self, s: _Sensor) -> None:
        h, s.handle, s.inbox = s.handle, None, None
        if h is None:
            return
        for step in (h.stop, h.destroy):          # stop first so no callback arrives for a dead actor
            try:
                step()
            except Exception as exc:
                logger.warning("LiDAR sensor %d: %s failed: %s", s.sensor_id, step.__name__, exc)

    def _fail(self, s: _Sensor, reason: str) -> None:
        self._destroy(s)
        s.error = reason
        self._event(s, SensorState.ERROR, reason)

    def shutdown(self) -> None:
        """Destroy every sensor (world stopping)."""
        for s in list(self._sensors.values()):
            if s.state not in (SensorState.DESTROYED, SensorState.ERROR):
                self._destroy(s)
                self._event(s, SensorState.DESTROYED, "shutdown")
        self._latest = {}
        self._publish_info()

    # ── published views (any thread) ──────────────────────────────────────────────────────────────
    def _publish_info(self) -> None:
        info = []
        for s in self._sensors.values():
            if s.state == SensorState.DESTROYED:
                continue
            info.append({
                "sensor_id": s.sensor_id, "actor_id": s.actor_id, "owner": s.owner, "state": s.state.value,
                "config": s.config.model_dump(), "viewers": self.viewer_count(s.sensor_id), "pinned": s.pinned,
                "record_raw": s.record_raw, "points_per_frame": s.config.points_per_frame(self.dt),
                "created_frame": s.created_frame, "last_frame": s.last_frame, "error": s.error,
            })
        n, used = self._used()
        # last_frame changes every tick and is not a change of the list; everything else is
        def _key(rows):
            return [{k: v for k, v in r.items() if k != "last_frame"} for r in rows]
        if _key(info) != _key(self._info) or (n, used) != (self._budget.get("sensors"), self._budget.get("used_pps")):
            self.info_version += 1
        self._info = info
        self._budget = {"used_pps": used, "max_pps": self.limits.max_points_per_second,
                        "sensors": n, "max_sensors": self.limits.max_sensors}

    def list_sensors(self) -> List[Dict[str, Any]]:
        return list(self._info)

    def budget(self) -> Dict[str, Any]:
        return dict(self._budget)

    def info_of(self, sensor_id: int) -> Optional[Dict[str, Any]]:
        for info in self._info:
            if info["sensor_id"] == sensor_id:
                return info
        return None

    def record_raw_ids(self) -> set:
        """Ids of sensors marked for raw recording (snapshot; any thread)."""
        return {i["sensor_id"] for i in self._info if i["record_raw"]}

    def get_frame(self, sensor_id: int) -> Optional[LidarFrame]:
        return self._latest.get(sensor_id)

    def state_of(self, sensor_id: int) -> Optional[SensorState]:
        s = self._sensors.get(sensor_id)
        return s.state if s else None
