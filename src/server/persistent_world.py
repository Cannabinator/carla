# TODO: CARLA handles / the module-global world are Optional until a connected-world type exists (WorldController state machine)
# pyright: reportOptionalMemberAccess=false
"""
Persistent CARLA world — the heart of the live-server architecture.

The admin starts this once; it runs indefinitely in a background thread.
Clients (laptop, raspi, workstation…) join and leave dynamically — each
gets one "leading vehicle" tracked here.

Usage
-----
    world = PersistentCARLAServer(WorldConfig(host=..., port=...))
    world.start()          # connect to CARLA, start the simulation thread
    ...
    world.stop()           # graceful shutdown + actor cleanup

Threading model (REFACTOR_HANDOFF section 4.1, defects C1-C5, C10)
------------------------------------------------------------------
Exactly one thread, the simulation thread ("carla-tick"), talks to CARLA once ``start()``
has returned. Everything else (HTTP handlers, WebSocket tasks) calls ``submit(fn, ...)``
and waits on the returned Future; the simulation thread runs it between two ticks. No
caller ticks the world: ``apply_batch_sync`` is always called with ``do_tick=False`` and
the one explicit tick (pedestrian controllers) goes through ``_tick()``, which keeps
``_frame`` equal to CARLA's own frame id.

Readers never call CARLA. After every tick the simulation thread publishes telemetry (and,
after every V2X update, the per-client V2X payload) as plain dicts stamped with the frame
they belong to; getters just return the last published dict.
"""

from __future__ import annotations

import logging
import math
import random
import re
import threading
import time
from collections import deque
from concurrent.futures import Future
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional

import carla

from src.lidar.carla_backend import CarlaSensorBackend
from src.lidar.sensor import LidarLimits, LidarManager
from src.lidar.stream import SensorStream
from src.server.events import EventLog
from src.recording.manifest import (git_info, library_versions, read_manifest, run_id as make_run_id, sha256_file, utc_now,
                                    write_manifest)
from src.recording.recorder import DEFAULT_RECORD, RunActiveError, RunRecorder
from src.server.map_geometry import build_map_geometry
from src.server.published import Published
from src.server.v2x_publisher import V2XPublisher
from src.server.session_manager import SessionManager
from src.server.sim_executor import SimThreadExecutor
from src.server.world_config import HOT_FIELDS, WorldConfig

logger = logging.getLogger(__name__)

# Blueprints that are always available in CARLA 0.9.x Town maps.
_FALLBACK_BLUEPRINTS = [
    "vehicle.tesla.model3",
    "vehicle.audi.a2",
    "vehicle.lincoln.mkz_2020",
]

_RUN_STATUS_EVERY_N_FRAMES = 20     # ~1 Hz at 20 FPS
_SESSION_CHECK_EVERY_N_FRAMES = 20  # ~1 s at 20 FPS (expiry and the disconnect grace period)


class PersistentCARLAServer:
    """
    Manages a persistent CARLA world that runs indefinitely.

    Background traffic and pedestrians are controlled by the admin via the
    REST API.  Each client session spawns exactly one "leading vehicle".
    """

    def __init__(self, config: WorldConfig):
        self.config = config
        self.host = config.host
        self.port = config.port
        self.fixed_delta_seconds = config.fixed_delta_seconds
        self.v2x_enabled = config.v2x_enabled
        self.awareness_range_m = config.awareness_range_m

        # Seeded RNG for spawn points / blueprints (defect C8)
        self._rng = random.Random(config.seed)
        # separate stream for the V2X-equipped choice, so changing the penetration rate does not
        # change which spawn points / blueprints the traffic gets
        self._v2x_rng = random.Random(config.seed + 1)

        # CARLA handles
        self.client: Optional[carla.Client] = None
        self.world: Optional[carla.World] = None
        self.tm: Optional[carla.TrafficManager] = None
        self.bp_lib: Optional[carla.BlueprintLibrary] = None
        self.spawn_points: List[carla.Transform] = []
        self._original_settings: Optional[carla.WorldSettings] = None
        self._map_name: Optional[str] = None

        # Managed actors (touched only by the simulation thread; other threads may len() them)
        self._background_vehicles: List[carla.Actor] = []
        self._v2x_background_ids: set = set()  # background vehicles registered as V2X stations
        self._pedestrians: List[carla.Actor] = []
        self._pedestrian_controllers: List[carla.Actor] = []
        self._client_vehicles: Dict[str, carla.Actor] = {}  # session_id → actor

        # Session manager (shared with HTTP routes; has its own lock)
        self.sessions = SessionManager()
        self.session_disconnect_grace_s = config.session_disconnect_grace_s
        # things that happened (thread-safe; read by the "events" topic)
        self.events = EventLog(config.event_log_size)
        # what the WebSocket hub reads (each is replaced as a whole by the simulation thread)
        self.observer_public = config.observer_public
        self.v2x_links_max_rows = config.v2x_links_max_rows
        self._v2x_pub = V2XPublisher()
        self._world_state: Optional[Published] = None
        self._world_state_version = 0
        self._run_status: Optional[Published] = None
        self._tick_ms: deque = deque(maxlen=400)       # work time of recent ticks (without the pacing sleep)
        self._tick_wall: deque = deque(maxlen=41)      # wall-clock start of recent ticks, for the real-time factor
        self._v2x_ms: deque = deque(maxlen=50)
        self._sim_time = 0.0                           # CARLA elapsed seconds of the last tick
        self._map_geometry: Optional[dict] = None      # cached road network (built once per world)
        self._denm_action = 0
        self._denm_expiry: Dict[tuple, float] = {}     # (station_id, action_id) -> sim time at which it ends
        self._rsu_disabled: Dict[int, Any] = {}        # intersection_id -> RoadSideUnit taken out of the network
        self._paused = False                           # headless runs: the loop drains commands but does not tick
        self._tick_index = 0                           # ticks done by the loop since it started (0 = nothing ticked yet)
        self._hooks: Dict[int, List[Callable[[], None]]] = {}   # scripted commands: run just before tick number N
        self._hooks_lock = threading.Lock()
        self._run: Optional[Dict[str, Any]] = None     # the active recorded run

        # V2X network (created in start())
        self.v2x_network = None

        # Simulation thread
        self._exec = SimThreadExecutor()
        self._ready = threading.Event()
        self._running = False
        self._tick_thread: Optional[threading.Thread] = None
        self._frame = 0
        self._start_time: Optional[float] = None
        self._last_v2x_frame = 0
        self._last_session_check_frame = 0

        # Published, immutable-by-convention snapshots read by other threads.
        # A reader sees either the previous or the next dict, never a half-built one.
        self._telemetry: Dict[str, dict] = {}
        self._v2x_payloads: Dict[str, Optional[dict]] = {}

        # Callback fired when a session expires (so WS can notify the client)
        self.on_session_expired: Optional[Callable[[str], None]] = None

        # LiDAR sensors exist only on request (REFACTOR_HANDOFF 5.5). The manager is driven by the simulation
        # thread; viewers (WebSocket tasks) only read its published frames through the per-sensor streams.
        self.lidar: Optional[LidarManager] = None
        self._lidar_streams: Dict[int, SensorStream] = {}
        self._lidar_streams_lock = threading.Lock()

    # ------------------------------------------------------------------ #
    # Threading helpers                                                    #
    # ------------------------------------------------------------------ #

    def submit(self, fn: Callable[..., Any], *args: Any, **kwargs: Any) -> Future:
        """Run ``fn`` on the simulation thread between two ticks; returns a Future."""
        return self._exec.submit(fn, *args, **kwargs)

    def _require_sim_thread(self) -> None:
        if not self._exec.is_sim_thread():
            raise RuntimeError(
                "CARLA state may only be changed on the simulation thread; use submit()"
            )

    def _tick(self) -> None:
        """The only place that ticks the world; ``_frame`` follows CARLA's own frame id."""
        self._frame = self.world.tick()

    # ------------------------------------------------------------------ #
    # Lifecycle                                                            #
    # ------------------------------------------------------------------ #

    def start(self, paused: bool = False) -> None:
        """Connect to CARLA, apply settings, start the simulation thread. ``paused``: the thread only executes submitted
        commands until ``resume()`` (reproducible runs set the world up first, then let it tick)."""
        self._paused = paused
        cfg = self.config
        logger.info("Connecting to CARLA at %s:%s", self.host, self.port)

        self.client = carla.Client(self.host, self.port)
        self.client.set_timeout(cfg.timeout_s)
        self.world = self.client.get_world()
        self._map_name = self.world.get_map().name

        logger.info("Connected to map: %s", self._map_name)

        # Store original settings for restoration on stop
        self._original_settings = self.world.get_settings()

        # Synchronous mode + physics substepping on a fresh copy (WorldConfig has already
        # checked fixed_delta_seconds <= max_substep_delta_time * max_substeps).
        settings = self.world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = cfg.fixed_delta_seconds
        settings.substepping = True
        settings.max_substep_delta_time = cfg.max_substep_delta_time
        settings.max_substeps = cfg.max_substeps
        self.world.apply_settings(settings)

        # Traffic Manager
        self.tm = self.client.get_trafficmanager(cfg.tm_port)
        self.tm.set_synchronous_mode(True)
        self.tm.set_random_device_seed(cfg.seed)
        self.tm.set_global_distance_to_leading_vehicle(cfg.safety_distance_m)
        self.tm.global_percentage_speed_difference(cfg.global_speed_pct)

        self.lidar = LidarManager(
            CarlaSensorBackend(carla, self.world),
            LidarLimits(max_sensors=cfg.lidar_max_sensors, max_points_per_second=cfg.lidar_max_points_per_second,
                        keep_alive_s=cfg.lidar_keep_alive_s),
            cfg.fixed_delta_seconds,
        )

        self.bp_lib = self.world.get_blueprint_library()
        self.spawn_points = self.world.get_map().get_spawn_points()
        logger.info("Map has %d spawn points", len(self.spawn_points))

        # Optional V2X network (BSM between vehicles + V2I RSU + V2P VRU)
        if self.v2x_enabled:
            self._init_v2x()

        self._running = True
        self._start_time = time.monotonic()
        self._ready.clear()
        self._tick_thread = threading.Thread(
            target=self._sim_main, daemon=True, name="carla-tick"
        )
        self._tick_thread.start()
        if not self._ready.wait(timeout=10.0):
            raise RuntimeError("simulation thread did not start")
        logger.info("Persistent CARLA server started (%.0f FPS target)", 1 / self.fixed_delta_seconds)

    def stop(self) -> None:
        """Stop the simulation thread; it destroys all managed actors on its way out."""
        logger.info("Stopping persistent CARLA server…")
        self._running = False
        thread = self._tick_thread
        if thread and thread.is_alive():
            thread.join(timeout=30.0)
        else:
            # Simulation thread never ran (start() failed part-way): clean up directly.
            self._cleanup_all()
            self._restore_settings()
        logger.info("Persistent CARLA server stopped")

    def _restore_settings(self) -> None:
        if self.world and self._original_settings:
            try:
                self.world.apply_settings(self._original_settings)
                logger.info("World settings restored")
            except Exception as e:
                logger.warning("Could not restore world settings: %s", e)

    def _init_v2x(self) -> None:
        try:
            from src.v2x.network import V2XNetwork
            from src.v2x.rsu import RoadSideUnit, group_traffic_lights
            from src.v2x.channel.dsrc_logdistance import DSRCConfig
            self.v2x_network = V2XNetwork(
                awareness_range_m=self.awareness_range_m,
                update_rate_hz=self.config.v2x_update_rate_hz,
                world=self.world,
                # the run seed seeds the radio too (its default, 42, equals WorldConfig's default seed: nothing changes there)
                dsrc_config=DSRCConfig(random_seed=self.config.seed),
            )
            # One RSU per intersection (CARLA light group), deterministic order, optionally capped
            all_tl = list(self.world.get_actors().filter("traffic.traffic_light"))
            groups = group_traffic_lights(all_tl)
            if self.config.rsu_max_count is not None:
                groups = groups[: self.config.rsu_max_count]
            for rsu_idx, group in enumerate(groups):
                rsu = RoadSideUnit.from_traffic_lights(group, intersection_id=rsu_idx + 1)
                self.v2x_network.register_rsu(rsu)
            logger.info(
                "V2X network initialised: range=%.0f m, %d RSUs",
                self.awareness_range_m if self.awareness_range_m is not None else float('inf'),
                len(self.v2x_network._rsus),
            )
        except ImportError:
            logger.warning("V2X module not available — V2X disabled")
            self.v2x_network = None

    # ------------------------------------------------------------------ #
    # Simulation thread                                                    #
    # ------------------------------------------------------------------ #

    def _sim_main(self) -> None:
        """Body of the simulation thread: tick loop, then orderly shutdown."""
        self._exec.bind_current_thread()
        self._last_v2x_frame = self._last_session_check_frame = self._frame
        self._ready.set()
        try:
            self._tick_loop()
        finally:
            self._exec.close("simulation stopped")
            try:
                self._cleanup_all()
            finally:
                self._restore_settings()

    def _tick_loop(self) -> None:
        logger.info("Tick loop running…")
        every_n = self.config.v2x_every_n_frames
        while self._running:
            if self._paused:
                self._exec.drain()
                time.sleep(0.001)
                continue
            # Real-time pacing: record wall-clock time before tick so we can
            # sleep the remainder of fixed_delta_seconds afterwards.
            tick_start = time.monotonic()
            try:
                self._exec.drain()  # submitted work runs here, at a frame boundary
                self._run_hooks()
                self._tick()
                self._tick_index += 1
                snapshot = self.world.get_snapshot()
                self._sim_time = float(snapshot.timestamp.elapsed_seconds)
                self._publish_telemetry(snapshot)
                self._collect_lidar(snapshot)
                self._run_periodic_work(snapshot, every_n)
                self._publish_hub_views(snapshot, tick_start)
                self._check_run()

            except RuntimeError as exc:
                logger.error("Tick error (frame %d): %s", self._frame, exc)
                time.sleep(0.1)  # brief back-off before retry
                continue

            if not self.config.realtime:
                continue                    # as fast as possible: no pacing
            elapsed = time.monotonic() - tick_start
            sleep_for = self.fixed_delta_seconds - elapsed
            if sleep_for > 0.001:
                time.sleep(sleep_for)

        logger.info("Tick loop stopped at frame %d", self._frame)

    # ------------------------------------------------------------------ #
    # Scripted commands and recorded runs                                  #
    # ------------------------------------------------------------------ #

    def resume(self) -> None:
        self._paused = False

    @property
    def tick_index(self) -> int:
        return self._tick_index

    def at_tick(self, index: int, fn: Callable[[], None]) -> None:
        """Run ``fn`` on the simulation thread just before tick number ``index`` (0 = the first tick of the loop), after any
        submitted command. Commands scripted this way execute at the same frame in every run."""
        with self._hooks_lock:
            self._hooks.setdefault(index, []).append(fn)

    def _run_hooks(self) -> None:
        with self._hooks_lock:
            due = self._hooks.pop(self._tick_index, [])
        for fn in due:
            try:
                fn()
            except Exception:
                logger.exception("scripted command failed before tick %d", self._tick_index)
                self._event("scripted_command_failed", tick=self._tick_index)

    def start_run(self, name: str, record=DEFAULT_RECORD, config_overrides: Optional[Dict[str, Any]] = None,
                  scenario: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
        """Start recording a run (simulation thread). One run at a time. ``config_overrides`` are hot fields applied first."""
        self._require_sim_thread()
        if self._run is not None:
            raise RunActiveError(f"a run is already active ({self._run['run_id']})")
        if config_overrides:
            self.apply_config(config_overrides)
        now = utc_now()
        git = git_info()
        rid = make_run_id(now, git["short_sha"], name)
        root = Path(self.config.runs_dir)
        recorder = RunRecorder(root / rid, record, self.config.recorder_queue_max)
        dsrc = getattr(getattr(self.v2x_network, "_dsrc_channel", None), "config", None)
        sensors = self.lidar.list_sensors() if self.lidar else []
        manifest: Dict[str, Any] = {
            "schema_version": 1, "run_id": rid, "name": name, "status": "running", "started_utc": now.isoformat(),
            "ended_utc": None, "git": git, "versions": library_versions(), "carla": self._carla_info(),
            "map": self._map_name, "fixed_delta_seconds": self.fixed_delta_seconds, "start_frame": self._frame,
            "start_sim_time": self._sim_time, "recorded": sorted(recorder.record),
            "config": self.config.model_dump(),
            "seeds": {"world_seed": self.config.seed, "v2x_penetration_rng": self.config.seed + 1,
                      "dsrc_channel": getattr(dsrc, "random_seed", None)},
            "channel": {k: v for k, v in vars(dsrc).items() if isinstance(v, (int, float, bool, str))} if dsrc else None,
            "lidar_sensors_at_start": sensors,
            "warnings": sorted({w for s in sensors for w in self._lidar_warnings(s)}),
            "scenario": scenario, "config_changes": [], "frames_recorded": None, "files": None,
        }
        write_manifest(root / rid, manifest)
        listener = lambda e: recorder.submit("event", e)  # noqa: E731
        self.events.add_listener(listener)
        self._run = {"run_id": rid, "name": name, "recorder": recorder, "manifest": manifest, "dir": root / rid,
                     "listener": listener, "start_frame": self._frame, "tick0": self._tick_index}
        self._event("run_started", run_id=rid, name=name, recorded=sorted(recorder.record))
        return {"run_id": rid, "dir": str(root / rid), "start_frame": self._frame}

    def _carla_info(self) -> Dict[str, Any]:
        info: Dict[str, Any] = {"module": getattr(carla, "__version__", "unknown"), "fake": bool(getattr(carla, "IS_FAKE", False))}
        try:
            info["server_version"] = self.client.get_server_version()
        except Exception:
            info["server_version"] = "unknown"
        return info

    def _lidar_warnings(self, sensor: Dict[str, Any]) -> List[str]:
        from src.lidar.config import LidarConfig
        return LidarConfig.model_validate(sensor["config"]).warnings(self.fixed_delta_seconds)

    def stop_run(self, reason: str = "stopped") -> Dict[str, Any]:
        """Finish the active run: write what is queued, hash the files, finalise the manifest (simulation thread)."""
        self._require_sim_thread()
        run = self._run
        if run is None:
            raise LookupError("no run is active")
        self._run = None
        rec: RunRecorder = run["recorder"]
        self._event("run_stopped", run_id=run["run_id"], reason=reason)       # recorded too: the listener is still attached
        self.events.remove_listener(run["listener"])
        rec.close()
        m = run["manifest"]
        failed = rec.error is not None
        files: Dict[str, Any] = {}
        for f in sorted(run["dir"].iterdir()):
            if f.is_file() and f.name != "manifest.json":
                files[f.name] = sha256_file(f)
        raw_dir = run["dir"] / "lidar"
        if raw_dir.exists():
            raws = [p for p in raw_dir.rglob("*.bin")]
            files["lidar/"] = {"files": len(raws), "bytes": sum(p.stat().st_size for p in raws)}
        changes = [{"frame": e["frame"], **e["data"]} for e in self.events.since(0, 100000)[0]
                   if e["kind"] == "config_changed" and e["frame"] is not None and e["frame"] >= run["start_frame"]]
        m.update({"status": "failed" if failed else "finished", "ended_utc": utc_now().isoformat(), "stop_reason": reason,
                  "end_frame": self._frame, "end_sim_time": self._sim_time, "frames_recorded": rec.v2x_updates,
                  "ticks": self._tick_index - run["tick0"], "files": files, "config_changes": changes,
                  "error": rec.error, "writer": rec.status()})
        write_manifest(run["dir"], m)
        return {"run_id": run["run_id"], "status": m["status"], "error": rec.error, "frames_recorded": rec.v2x_updates}

    def _check_run(self) -> None:
        run = self._run
        if run is not None and run["recorder"].error:
            logger.error("stopping run %s: %s", run["run_id"], run["recorder"].error)
            self.stop_run("writer error")

    def _record(self, kind: str, payload: Any) -> None:
        run = self._run
        if run is None:
            return
        if kind == "v2x":
            if run.get("last_v2x") == payload.frame:         # the same update twice is not two updates
                return
            run["last_v2x"] = payload.frame
        run["recorder"].submit(kind, payload)

    def run_status(self) -> Optional[Dict[str, Any]]:
        run = self._run
        if run is None:
            return None
        return {"run_id": run["run_id"], "name": run["name"], "start_frame": run["start_frame"], **run["recorder"].status()}

    def list_runs(self) -> List[Dict[str, Any]]:
        from src.recording.manifest import list_runs
        return list_runs(Path(self.config.runs_dir))

    def run_manifest(self, run_id: str) -> Dict[str, Any]:
        if not re.fullmatch(r"[A-Za-z0-9._-]+", run_id) or run_id.startswith("."):
            raise LookupError("bad run id")
        d = Path(self.config.runs_dir) / run_id
        if not (d / "manifest.json").exists():
            raise LookupError(f"no run {run_id}")
        return read_manifest(d)

    def _collect_lidar(self, snapshot) -> None:
        """Sensor state machine and this frame's measurements. An error here must not stop the simulation."""
        if self.lidar is None:
            return
        try:
            frames = self.lidar.collect(snapshot.frame, float(snapshot.timestamp.elapsed_seconds))
            if self._run is not None and frames:
                raw_ids = self.lidar.record_raw_ids()
                for f in frames:
                    self._record("lidar", (f, f.sensor_id in raw_ids))
            for e in self.lidar.drain_events():
                self.events.add("lidar_" + e["to"], frame=e["frame"], sim_time=snapshot.timestamp.elapsed_seconds,
                                **{k: v for k, v in e.items() if k != "frame"})
        except Exception:
            logger.exception("LiDAR collection failed at frame %d (simulation continues)", self._frame)

    # LiDAR requests: simulation thread only (the routes go through ``on_sim_thread``)
    def lidar_request(self, principal, actor_id, config=None, **kwargs) -> int:
        self._require_sim_thread()
        return self.lidar.request(principal, actor_id, config, **kwargs)

    def lidar_reconfigure(self, principal, sensor_id, config) -> None:
        self._require_sim_thread()
        self.lidar.reconfigure(principal, sensor_id, config)

    def lidar_set_flags(self, principal, sensor_id, pinned=None, record_raw=None) -> None:
        self._require_sim_thread()
        self.lidar.set_flags(principal, sensor_id, pinned, record_raw)

    def lidar_release(self, principal, sensor_id) -> None:
        self._require_sim_thread()
        self.lidar.release(principal, sensor_id)

    def get_lidar_stream(self, sensor_id: int) -> SensorStream:
        """Encode-once cache for a sensor (any thread)."""
        with self._lidar_streams_lock:
            stream = self._lidar_streams.get(sensor_id)
            if stream is None:
                stream = self._lidar_streams[sensor_id] = SensorStream(sensor_id, 1.0 / self.fixed_delta_seconds)
            return stream

    def _run_periodic_work(self, snapshot, every_n: int) -> None:
        """V2X update and session expiry. An error here must not stop the simulation."""
        if self._frame - self._last_v2x_frame >= every_n:
            self._last_v2x_frame = self._frame
            self._expire_denms()
            if self.v2x_network is not None:
                try:
                    t0 = time.monotonic()
                    self.v2x_network.update(snapshot)
                    self._publish_v2x()
                    frame = self.v2x_network.get_frame()
                    if frame is not None:
                        self._v2x_pub.on_update(frame)
                        self._record("v2x", frame)
                    self._v2x_ms.append((time.monotonic() - t0) * 1000.0)
                except Exception:
                    logger.exception("V2X update failed at frame %d (simulation continues)", self._frame)

        if self._frame - self._last_session_check_frame >= _SESSION_CHECK_EVERY_N_FRAMES:
            self._last_session_check_frame = self._frame
            try:
                self._check_expired_sessions()
            except Exception:
                logger.exception("Session expiry check failed at frame %d (simulation continues)", self._frame)

    def _publish_telemetry(self, snapshot) -> None:
        """Telemetry of every client vehicle, from the snapshot of the frame just ticked."""
        telemetry: Dict[str, dict] = {}
        sim_time = float(snapshot.timestamp.elapsed_seconds)
        for sid, actor in self._client_vehicles.items():
            snap = snapshot.find(actor.id)
            if snap is None:
                continue
            telemetry[sid] = self._telemetry_entry(
                actor.id, snap.get_transform(), snap.get_velocity(), snapshot.frame, sim_time
            )
        self._telemetry = telemetry

    @staticmethod
    def _telemetry_entry(actor_id, transform, velocity, frame, sim_time) -> dict:
        speed_kmh = (velocity.x**2 + velocity.y**2 + velocity.z**2) ** 0.5 * 3.6
        return {
            "actor_id": actor_id,
            "x": round(transform.location.x, 2),
            "y": round(transform.location.y, 2),
            "z": round(transform.location.z, 2),
            "yaw": round(transform.rotation.yaw, 1),
            "speed_kmh": round(speed_kmh, 1),
            "frame": frame,
            "sim_time": sim_time,
        }

    def _publish_v2x(self) -> None:
        payloads: Dict[str, Optional[dict]] = {}
        for sid in self._client_vehicles:
            try:
                payloads[sid] = self._build_v2x_data(sid)
            except Exception as exc:  # one bad client must not stop the others
                logger.warning("V2X payload for session %s failed: %s", sid, exc)
        self._v2x_payloads = payloads

    def _check_expired_sessions(self) -> None:
        self.sessions.prune_ended()
        for session in self.sessions.get_expired_sessions():
            logger.info("Session %s expired — removing vehicle", session.session_id)
            self._despawn_client_vehicle(session.session_id)
            self.sessions.end_session(session.session_id)
            self._event("session_expired", client=session.client_name, actor_id=session.vehicle_actor_id)
            if self.on_session_expired:
                self.on_session_expired(session.session_id)
        if self.session_disconnect_grace_s > 0:
            for session in self.sessions.get_grace_expired(self.session_disconnect_grace_s):
                logger.info("Session %s: no connection for %.0f s — removing vehicle",
                            session.session_id, self.session_disconnect_grace_s)
                self._despawn_client_vehicle(session.session_id)
                self.sessions.end_session(session.session_id)
                self._event("session_disconnected_removed", client=session.client_name,
                            actor_id=session.vehicle_actor_id, grace_s=self.session_disconnect_grace_s)

    # ------------------------------------------------------------------ #
    # Views for the WebSocket hub (built on the simulation thread)         #
    # ------------------------------------------------------------------ #

    def _publish_hub_views(self, snapshot, tick_start: float) -> None:
        """Ground-truth world state (every ``world_state_every_n_frames``) and run status (about 1 Hz). Failures
        are logged and never stop the simulation."""
        self._tick_wall.append(tick_start)
        self._tick_ms.append((time.monotonic() - tick_start) * 1000.0)
        try:
            if snapshot.frame % self.config.world_state_every_n_frames == 0:
                self._publish_world_state(snapshot)
            if snapshot.frame % _RUN_STATUS_EVERY_N_FRAMES == 0:
                self._publish_run_status(snapshot)
        except Exception:
            logger.exception("Publishing hub views failed at frame %d (simulation continues)", self._frame)

    def _station_entry(self, actor, snap, kind: str, role: str, name: Optional[str], v2x: bool) -> Optional[dict]:
        if snap is None:
            return None
        t, v = snap.get_transform(), snap.get_velocity()
        return {"id": actor.id, "kind": kind, "role": role, "name": name, "v2x": v2x,
                "x": round(t.location.x, 2), "y": round(t.location.y, 2), "z": round(t.location.z, 2),
                "yaw": round(t.rotation.yaw, 1), "speed_mps": round((v.x ** 2 + v.y ** 2 + v.z ** 2) ** 0.5, 2)}

    def _publish_world_state(self, snapshot) -> None:
        stations = []
        names = {s.session_id: s.client_name for s in self.sessions.get_active_sessions()}
        for sid, actor in self._client_vehicles.items():
            e = self._station_entry(actor, snapshot.find(actor.id), "vehicle", "client", names.get(sid), True)
            if e:
                stations.append(e)
        for actor in self._background_vehicles:
            e = self._station_entry(actor, snapshot.find(actor.id), "vehicle", "background", None,
                                    actor.id in self._v2x_background_ids)
            if e:
                stations.append(e)
        vrus = getattr(self.v2x_network, "_vrus", {})
        for actor in self._pedestrians:
            e = self._station_entry(actor, snapshot.find(actor.id), "pedestrian", "background", None, actor.id in vrus)
            if e:
                stations.append(e)
        rsus = []
        for enabled, group in ((True, getattr(self.v2x_network, "_rsus", {})), (False, self._rsu_disabled)):
            for iid, rsu in list(group.items()):
                x, y, z = rsu.config.position
                rsus.append({"station_id": -int(iid), "intersection_id": iid, "x": round(x, 2), "y": round(y, 2), "z": round(z, 2),
                             "enabled": enabled, "tx_power_dbm": rsu.config.tx_power_dbm,
                             "broadcast_range_m": rsu.config.broadcast_range_m})
        rsus.sort(key=lambda r: r["intersection_id"])
        self._world_state_version += 1
        self._world_state = Published(self._world_state_version, snapshot.frame,
                                      float(snapshot.timestamp.elapsed_seconds),
                                      {"map": self._map_name, "stations": stations, "rsus": rsus})

    @staticmethod
    def _percentiles(values) -> dict:
        v = sorted(values)
        if not v:
            return {"n": 0, "p50": None, "p95": None, "max": None}
        pick = lambda q: v[min(len(v) - 1, int(round(q * (len(v) - 1))))]  # noqa: E731
        return {"n": len(v), "p50": round(pick(0.5), 2), "p95": round(pick(0.95), 2), "max": round(v[-1], 2)}

    def _publish_run_status(self, snapshot) -> None:
        walls = list(self._tick_wall)
        rtf = None
        if len(walls) >= 2 and walls[-1] > walls[0]:
            rtf = round((len(walls) - 1) * self.fixed_delta_seconds / (walls[-1] - walls[0]), 3)
        sim_time = float(snapshot.timestamp.elapsed_seconds)
        self._run_status = Published(snapshot.frame // _RUN_STATUS_EVERY_N_FRAMES, snapshot.frame, sim_time, {
            "running": self._running, "frame": snapshot.frame, "sim_time": sim_time, "map": self._map_name,
            "fixed_delta_seconds": self.fixed_delta_seconds, "real_time_factor": rtf,
            "tick_duration_ms": self._percentiles(self._tick_ms),
            "v2x_update_ms": self._percentiles(self._v2x_ms),
            "sessions": len(self.sessions.get_active_sessions()),
            "background_vehicles": len(self._background_vehicles), "pedestrians": len(self._pedestrians),
            "lidar_budget": self.lidar.budget() if self.lidar else None,
            "recording": self.run_status(),
        })

    # ------------------------------------------------------------------ #
    # Hot configuration and RSU management (admin)                         #
    # ------------------------------------------------------------------ #

    def apply_config(self, changes: Dict[str, Any]) -> Dict[str, Any]:
        """Change hot fields of the configuration (simulation thread). Everything is validated first (the cross-field
        rules of WorldConfig included) and either all of it is applied or none. Every change is an event with the frame
        at which it was applied."""
        self._require_sim_thread()
        unknown = sorted(k for k in changes if k not in WorldConfig.model_fields)
        if unknown:
            raise ValueError(f"unknown configuration fields: {', '.join(unknown)}")
        restart = sorted(k for k in changes if k not in HOT_FIELDS)
        if restart:
            raise PermissionError(f"these fields need a new world (restart): {', '.join(restart)}")
        merged = {**self.config.model_dump(), **changes}
        new = WorldConfig.model_validate(merged)               # raises pydantic.ValidationError (a ValueError)
        applied: Dict[str, Any] = {}
        for k in changes:
            old, val = getattr(self.config, k), getattr(new, k)
            if old != val:
                applied[k] = {"old": old, "new": val, "effective": HOT_FIELDS[k]}
        self.config = new
        if "awareness_range_m" in applied:
            self.awareness_range_m = new.awareness_range_m
            if self.v2x_network is not None:
                self.v2x_network.awareness_range_m = new.awareness_range_m
        if "session_disconnect_grace_s" in applied:
            self.session_disconnect_grace_s = new.session_disconnect_grace_s
        if "observer_public" in applied:
            self.observer_public = new.observer_public
        if "v2x_links_max_rows" in applied:
            self.v2x_links_max_rows = new.v2x_links_max_rows
        if self.lidar is not None:
            lim = self.lidar.limits
            lim.max_sensors, lim.max_points_per_second, lim.keep_alive_s = (
                new.lidar_max_sensors, new.lidar_max_points_per_second, new.lidar_keep_alive_s)
        frame = self._frame
        for k, v in applied.items():
            self._event("config_changed", field=k, old=v["old"], new=v["new"], applied_at_frame=frame, effective=v["effective"])
        return {"applied": applied, "applied_at_frame": frame, "sim_time": self._sim_time}

    def set_rsu(self, station_id: int, enabled: Optional[bool] = None, tx_power_dbm: Optional[float] = None) -> Dict[str, Any]:
        """Enable/disable one RSU or change its transmit power (simulation thread)."""
        self._require_sim_thread()
        if self.v2x_network is None:
            raise ValueError("V2X is disabled")
        iid = -int(station_id)
        net = self.v2x_network
        rsu = net._rsus.get(iid) or self._rsu_disabled.get(iid)
        if station_id >= 0 or rsu is None:
            raise LookupError(f"no RSU with station id {station_id}")
        changes: Dict[str, Any] = {}
        if tx_power_dbm is not None and rsu.config.tx_power_dbm != tx_power_dbm:
            changes["tx_power_dbm"] = {"old": rsu.config.tx_power_dbm, "new": tx_power_dbm}
            rsu.config.tx_power_dbm = tx_power_dbm
        if enabled is not None:
            now_enabled = iid in net._rsus
            if enabled and not now_enabled:
                net.register_rsu(self._rsu_disabled.pop(iid))
                changes["enabled"] = {"old": False, "new": True}
            elif not enabled and now_enabled:
                self._rsu_disabled[iid] = net._rsus[iid]
                net.unregister_rsu(iid)
                changes["enabled"] = {"old": True, "new": False}
        for k, v in changes.items():
            self._event("rsu_changed", station_id=station_id, field=k, old=v["old"], new=v["new"], applied_at_frame=self._frame)
        return {"station_id": station_id, "changes": changes, "applied_at_frame": self._frame,
                "enabled": iid in net._rsus, "tx_power_dbm": rsu.config.tx_power_dbm}

    # ------------------------------------------------------------------ #
    # Map and DENM injection (decision D11)                                #
    # ------------------------------------------------------------------ #

    def get_map_geometry(self) -> dict:
        """Road network for the map view; built once on the simulation thread, then cached."""
        self._require_sim_thread()
        if self._map_geometry is None:
            self._map_geometry = build_map_geometry(self.world.get_map())
        return self._map_geometry

    def cached_map_geometry(self) -> Optional[dict]:
        return self._map_geometry

    def inject_denm(self, x: float, y: float, z: float, cause_code: int, subcause_code: int,
                    relevance_distance_m: float, information_quality: int,
                    station_id: Optional[int] = None, duration_s: Optional[float] = None) -> dict:
        """Create a DENM at a map position (admin action). It is transmitted at every V2X update until it is
        cancelled or ``duration_s`` of simulation time have passed. Without ``station_id`` a fixed roadside
        transmitter at the event position sends it (ids -1_000_001 and below; they cannot clash with actors or
        RSUs)."""
        from src.v2x.messages.denm import DENM, DENMCauseCode
        self._require_sim_thread()
        if self.v2x_network is None:
            raise ValueError("V2X is disabled")
        self._denm_action += 1
        action_id = self._denm_action
        if station_id is None:
            station_id = -(1_000_000 + action_id)
        elif station_id >= 0 and station_id not in self.v2x_network.vehicles:
            raise ValueError(f"station {station_id} is not a registered V2X vehicle")
        elif station_id < 0 and -station_id not in getattr(self.v2x_network, "_rsus", {}):
            raise ValueError(f"station {station_id} is not a registered RSU")
        now = self._sim_time
        denm = DENM(station_id=station_id, action_id=action_id, detection_time=now, reference_time=now,
                    event_position=(x, y, z), cause_code=DENMCauseCode(cause_code), subcause_code=subcause_code,
                    information_quality=information_quality, relevance_distance=relevance_distance_m)
        self.v2x_network.publish_denm(denm)
        expires = None
        if duration_s is not None:
            expires = now + duration_s
            self._denm_expiry[(station_id, action_id)] = expires
        self._event("denm_injected", station_id=station_id, action_id=action_id, cause_code=int(cause_code),
                    x=x, y=y, relevance_distance_m=relevance_distance_m, expires_sim_time=expires)
        return {"station_id": station_id, "action_id": action_id, "expires_sim_time": expires}

    def cancel_denm(self, station_id: int, action_id: int) -> bool:
        self._require_sim_thread()
        if self.v2x_network is None:
            return False
        self._denm_expiry.pop((station_id, action_id), None)
        ok = self.v2x_network.cancel_denm(station_id, action_id)
        if ok:
            self._event("denm_cancelled", station_id=station_id, action_id=action_id)
        return ok

    def list_denm(self) -> list:
        """Active DENMs as plain dicts (any thread; reads the last V2X frame)."""
        from src.server.published import message_to_dict
        frame = self.get_v2x_frame()
        if frame is None:
            return []
        return [message_to_dict(m) for msgs in frame.sent.values() for m in msgs if type(m).__name__ == "DENM"]

    def _expire_denms(self) -> None:
        if not self._denm_expiry or self.v2x_network is None:
            return
        for key, end in list(self._denm_expiry.items()):
            if self._sim_time >= end:
                del self._denm_expiry[key]
                if self.v2x_network.cancel_denm(*key):
                    self._event("denm_expired", station_id=key[0], action_id=key[1])

    def get_world_state(self) -> Optional[Published]:
        return self._world_state

    def get_v2x_published(self) -> Optional[Published]:
        return self._v2x_pub.summary

    def get_v2x_frame(self):
        return self._v2x_pub.frame

    def get_v2x_stats(self) -> Optional[Published]:
        return self._v2x_pub.stats

    def get_run_status(self) -> Optional[Published]:
        return self._run_status

    def _event(self, kind: str, **data: Any) -> None:
        self.events.add(kind, frame=self._frame, sim_time=self._sim_time, **data)

    # ------------------------------------------------------------------ #
    # Admin — background traffic  (simulation thread only: use submit())  #
    # ------------------------------------------------------------------ #

    def spawn_background_traffic(self, count: int) -> int:
        """Spawn *count* autopilot vehicles. Returns number actually spawned."""
        self._require_sim_thread()
        if not self.world:
            raise RuntimeError("Server not started")

        occupied = self._all_vehicle_locations()
        free_points = [
            sp for sp in self.spawn_points
            if not self._too_close(sp.location, occupied, min_dist=8.0)
        ]
        self._rng.shuffle(free_points)
        targets = free_points[:count]

        bps = [
            bp for bp in self.bp_lib.filter("vehicle.*")
            if int(bp.get_attribute("number_of_wheels")) >= 4
        ]
        if not bps:
            logger.error("No 4-wheel blueprints available")
            return 0

        batch = []
        for sp in targets:
            bp = self._rng.choice(bps)
            if bp.has_attribute("color"):
                color = self._rng.choice(bp.get_attribute("color").recommended_values)
                bp.set_attribute("color", color)
            batch.append(
                carla.command.SpawnActor(bp, sp).then(
                    carla.command.SetAutopilot(
                        carla.command.FutureActor, True, self.tm.get_port()
                    )
                )
            )

        results = self.client.apply_batch_sync(batch, False)
        spawned = 0
        for r in results:
            if r.error:
                logger.debug("Spawn error: %s", r.error)
                continue
            actor = self.world.get_actor(r.actor_id)
            if actor:
                self._background_vehicles.append(actor)
                self._maybe_register_background(actor)
                spawned += 1

        logger.info(
            "Spawned %d / %d background vehicles (%d V2X stations in total)",
            spawned, count, len(self._v2x_background_ids),
        )
        self._event("background_traffic_spawned", requested=count, spawned=spawned)
        return spawned

    def _unregister_background_stations(self) -> None:
        if self.v2x_network is not None:
            for actor_id in self._v2x_background_ids:
                self.v2x_network.unregister(actor_id)
        self._v2x_background_ids.clear()

    def _maybe_register_background(self, actor) -> None:
        """S3: make a background vehicle a V2X station with probability penetration_rate (seeded)."""
        if self.v2x_network is None:
            return
        if self._v2x_rng.random() < self.config.v2x_penetration_rate:
            self.v2x_network.register(actor.id, actor)
            self._v2x_background_ids.add(actor.id)

    def despawn_background_traffic(self) -> int:
        """Destroy all admin-managed background vehicles."""
        self._require_sim_thread()
        count = len(self._background_vehicles)
        if count == 0:
            return 0
        self._unregister_background_stations()
        if self.lidar is not None:
            for v in self._background_vehicles:
                self.lidar.on_actor_removed(v.id)
            self.lidar.flush_stopping()
        batch = [carla.command.DestroyActor(v) for v in self._background_vehicles]
        self.client.apply_batch_sync(batch, False)
        self._background_vehicles.clear()
        logger.info("Despawned %d background vehicles", count)
        self._event("background_traffic_removed", count=count)
        return count

    # ------------------------------------------------------------------ #
    # Admin — pedestrians                                                  #
    # ------------------------------------------------------------------ #

    def spawn_pedestrians(self, count: int) -> int:
        """Spawn *count* AI-controlled pedestrians. Returns number spawned."""
        self._require_sim_thread()
        if not self.world:
            raise RuntimeError("Server not started")

        ped_bps = list(self.bp_lib.filter("walker.pedestrian.*"))
        ctrl_bp = self.bp_lib.find("controller.ai.walker")
        if not ped_bps or ctrl_bp is None:
            logger.error("Pedestrian blueprints not found")
            return 0

        # Collect random navigation locations first
        spawn_batch = []
        for _ in range(count):
            loc = self.world.get_random_location_from_navigation()
            if loc is None:
                continue
            bp = self._rng.choice(ped_bps)
            if bp.has_attribute("is_invincible"):
                bp.set_attribute("is_invincible", "false")
            spawn_batch.append(carla.command.SpawnActor(bp, carla.Transform(loc)))

        results = self.client.apply_batch_sync(spawn_batch, False)
        ped_actors: List[carla.Actor] = []
        for r in results:
            if r.error:
                continue
            actor = self.world.get_actor(r.actor_id)
            if actor:
                self._pedestrians.append(actor)
                ped_actors.append(actor)

        # Spawn AI controllers attached to each pedestrian
        ctrl_batch = [
            carla.command.SpawnActor(ctrl_bp, carla.Transform(), p)
            for p in ped_actors
        ]
        self._tick()  # needed before attaching controllers (counted in _frame)
        ctrl_results = self.client.apply_batch_sync(ctrl_batch, False)

        for r in ctrl_results:
            if r.error:
                continue
            ctrl = self.world.get_actor(r.actor_id)
            if ctrl:
                self._pedestrian_controllers.append(ctrl)
                ctrl.start()
                dest = self.world.get_random_location_from_navigation()
                if dest:
                    ctrl.go_to_location(dest)
                ctrl.set_max_speed(1.4)

        spawned = len(ped_actors)
        # Register spawned pedestrians as V2P VRUs in the V2X network
        if self.v2x_network is not None:
            from src.v2x.messages import VRUProfile
            for ped in ped_actors:
                self.v2x_network.register_vru(ped.id, ped, VRUProfile.PEDESTRIAN)
        logger.info("Spawned %d / %d pedestrians", spawned, count)
        self._event("pedestrians_spawned", requested=count, spawned=spawned)
        return spawned

    def despawn_pedestrians(self) -> int:
        """Destroy all admin-managed pedestrians."""
        self._require_sim_thread()
        for ctrl in self._pedestrian_controllers:
            try:
                ctrl.stop()
            except Exception:
                pass
        # Unregister VRUs from V2X network before destroying actors
        if self.v2x_network is not None:
            for ped in self._pedestrians:
                try:
                    self.v2x_network.unregister_vru(ped.id)
                except Exception:
                    pass
        count = len(self._pedestrians)
        batch = (
            [carla.command.DestroyActor(c) for c in self._pedestrian_controllers]
            + [carla.command.DestroyActor(p) for p in self._pedestrians]
        )
        if batch:
            self.client.apply_batch_sync(batch, False)
        self._pedestrian_controllers.clear()
        self._pedestrians.clear()
        logger.info("Despawned %d pedestrians", count)
        self._event("pedestrians_removed", count=count)
        return count

    # ------------------------------------------------------------------ #
    # Client — leading vehicle                                             #
    # ------------------------------------------------------------------ #

    def spawn_client_vehicle(
        self,
        session_id: str,
        blueprint_id: str,
        spawn_index: Optional[int] = None,
    ) -> Optional[int]:
        """
        Spawn a leading vehicle for a client session.

        Returns the CARLA actor ID, or None if spawn failed.
        """
        self._require_sim_thread()
        if not self.world:
            raise RuntimeError("Server not started")

        session = self.sessions.get_session(session_id)
        if session is None:
            raise ValueError(f"Unknown session: {session_id}")

        # Already has a vehicle?
        if session_id in self._client_vehicles:
            return self._client_vehicles[session_id].id

        bp = self._resolve_blueprint(blueprint_id)
        if bp.has_attribute("color"):
            bp.set_attribute("color", "255,50,50")  # distinctive red

        sp = self._pick_spawn_point(spawn_index)
        actor = self.world.try_spawn_actor(bp, sp)
        if actor is None:
            logger.error("Failed to spawn vehicle for session %s", session_id)
            return None

        actor.set_autopilot(True, self.tm.get_port())
        self._client_vehicles[session_id] = actor
        session.vehicle_actor_id = actor.id

        # Publish a first telemetry sample now so the joining client can read it at once
        # (the next per-frame publish replaces it).
        self._telemetry = {
            **self._telemetry,
            session_id: self._telemetry_entry(
                actor.id, actor.get_transform(), actor.get_velocity(), self._frame,
                self._frame * self.fixed_delta_seconds,
            ),
        }

        if self.v2x_network is not None:
            self.v2x_network.register(actor.id, actor)

        logger.info(
            "Spawned client vehicle actor=%d for session %s (%s)",
            actor.id, session_id, session.client_name,
        )
        self._event("client_vehicle_spawned", client=session.client_name, actor_id=actor.id)
        return actor.id

    def despawn_client_vehicle(self, session_id: str) -> None:
        """Despawn a client vehicle (called on leave, kick or expiry)."""
        self._require_sim_thread()
        self._despawn_client_vehicle(session_id)

    def _despawn_client_vehicle(self, session_id: str) -> None:
        if self.lidar is not None:
            # sensors go before the vehicle they are attached to
            self.lidar.release_owned_by(session_id)
            vehicle = self._client_vehicles.get(session_id)
            if vehicle is not None:
                self.lidar.on_actor_removed(vehicle.id)
            self.lidar.flush_stopping()
        actor = self._client_vehicles.pop(session_id, None)
        self._telemetry = {k: v for k, v in self._telemetry.items() if k != session_id}
        self._v2x_payloads = {k: v for k, v in self._v2x_payloads.items() if k != session_id}
        if actor is None:
            return
        try:
            if self.v2x_network is not None:
                self.v2x_network.unregister(actor.id)
        except Exception:
            pass
        try:
            actor.destroy()
        except Exception as exc:
            logger.warning("Error destroying client vehicle: %s", exc)
        logger.info("Despawned client vehicle for session %s", session_id)
        self._event("client_vehicle_removed", actor_id=actor.id)

    # ------------------------------------------------------------------ #
    # State queries (any thread; read published data, never call CARLA)    #
    # ------------------------------------------------------------------ #

    def get_status(self) -> dict:
        elapsed = time.monotonic() - self._start_time if self._start_time else 0
        v2x = self.v2x_network
        return {
            "running": self._running,
            "frame": self._frame,
            "elapsed_seconds": round(elapsed, 1),
            "map": self._map_name,
            "background_vehicles": len(self._background_vehicles),
            "pedestrians": len(self._pedestrians),
            "active_sessions": len(self.sessions.get_active_sessions()),
            "client_vehicles": len(self._client_vehicles),
            "v2x_enabled": v2x is not None,
            "v2x_stations": len(getattr(v2x, "vehicles", {})),
            "v2x_penetration_rate": self.config.v2x_penetration_rate,
            "awareness_range_m": self.awareness_range_m if v2x else None,
            "rsu_count": len(getattr(v2x, "_rsus", {})),
            "vru_count": len(getattr(v2x, "_vrus", {})),
        }

    def get_vehicle_telemetry(self, session_id: str) -> Optional[dict]:
        """Last published telemetry for a client's vehicle (stamped with its frame)."""
        return self._telemetry.get(session_id)

    def get_vehicle_v2x_data(self, session_id: str) -> Optional[dict]:
        """Last published V2X payload for a client (updated at each V2X update)."""
        return self._v2x_payloads.get(session_id)

    def _build_v2x_data(self, session_id: str) -> Optional[dict]:
        """Build the V2X payload for one client (simulation thread only)."""
        if self.v2x_network is None:
            return None
        actor = self._client_vehicles.get(session_id)
        if actor is None:
            return None
        vid = actor.id
        bsm = self.v2x_network.get_bsm(vid)
        if bsm is None:
            return None  # first few frames before first V2X update

        def _safe_ttc(ttc: float) -> Optional[float]:
            return None if not math.isfinite(ttc) else round(ttc, 2)

        neighbors = self.v2x_network.get_neighbors(vid)
        threats = self.v2x_network.get_threats(vid)

        # SPAT and VRUAM THIS vehicle actually received over the radio in the last V2X update
        # (not a geometric guess): delivered messages only, each with its link quality.
        spat_nearby = []
        for rm in self.v2x_network.get_received(vid, "SPAT"):
            spat = rm.message
            spat_nearby.append({
                "intersection_id": spat.intersection_id,
                "distance": round(rm.link.distance_m, 1),
                "rx_power_dbm": round(rm.link.rx_power_dbm, 1),
                "states": [
                    {
                        "signal_group_id": st.signal_group_id,
                        "movement_event": st.movement_event,
                        "likely_time_s": round(st.likely_time_s, 1)
                        if st.likely_time_s is not None else None,
                    }
                    for st in spat.states
                ],
            })

        vru_nearby = []
        for rm in self.v2x_network.get_received(vid, "VRUAM"):
            vruam = rm.message
            vru_nearby.append({
                "station_id": vruam.station_id,
                "profile": vruam.vru_profile.name,
                "distance": round(rm.link.distance_m, 1),
                "rx_power_dbm": round(rm.link.rx_power_dbm, 1),
                "speed": round(vruam.speed, 2),
                "motion_state": vruam.motion_state,
            })

        denm_received = []
        for rm in self.v2x_network.get_received(vid, "DENM"):
            denm = rm.message
            ex, ey = denm.event_position[0], denm.event_position[1]
            denm_received.append({
                "station_id": denm.station_id,
                "action_id": denm.action_id,
                "cause_code": denm.cause_code.name,
                "subcause_code": denm.subcause_code,
                "distance_to_event": round(math.hypot(bsm.x_m - ex, bsm.y_m - ey), 1),
                "rx_power_dbm": round(rm.link.rx_power_dbm, 1),
            })

        frame = self.v2x_network.get_frame()

        return {
            "bsm": {
                "vehicle_id": bsm.vehicle_id,
                "speed": round(bsm.speed, 2),
                "heading": round(bsm.heading, 1),
                "position": {
                    "x": round(bsm.x_m, 2),
                    "y": round(bsm.y_m, 2),
                    "z": round(bsm.z_m, 2),
                },
            },
            "neighbors": [
                {
                    "vehicle_id": n.vehicle_id,
                    "distance": round(
                        self.v2x_network.get_distance(vid, n.vehicle_id) or 0.0, 1
                    ),
                    "speed": round(n.speed, 2),
                    "heading": round(n.heading, 1),
                }
                for n in neighbors
            ],
            "threats": [
                {
                    "other_vehicle_id": t["other_vehicle_id"],
                    "threat_level": t["level"],
                    "time_to_collision": _safe_ttc(t["ttc"]),
                    "distance": round(t["distance"], 1),
                }
                for t in threats
            ],
            "spat_nearby": spat_nearby,
            "vru_nearby": vru_nearby,
            "denm_received": denm_received,
            "v2x_frame": frame.frame if frame else None,
            "v2x_sim_time": frame.sim_time if frame else None,
        }

    # ------------------------------------------------------------------ #
    # Helpers                                                              #
    # ------------------------------------------------------------------ #

    def _resolve_blueprint(self, blueprint_id: str) -> carla.ActorBlueprint:
        bp = self.bp_lib.find(blueprint_id)
        if bp is not None:
            return bp
        for fallback_id in _FALLBACK_BLUEPRINTS:
            bp = self.bp_lib.find(fallback_id)
            if bp is not None:
                return bp
        return list(self.bp_lib.filter("vehicle.*"))[0]

    def _pick_spawn_point(self, spawn_index: Optional[int]) -> carla.Transform:
        if spawn_index is not None and 0 <= spawn_index < len(self.spawn_points):
            return self.spawn_points[spawn_index]
        occupied = self._all_vehicle_locations()
        free = [
            sp for sp in self.spawn_points
            if not self._too_close(sp.location, occupied, min_dist=10.0)
        ]
        if free:
            return self._rng.choice(free)
        return self._rng.choice(self.spawn_points)

    def _all_vehicle_locations(self) -> List[carla.Location]:
        locs = []
        for actor in list(self._background_vehicles) + list(self._client_vehicles.values()):
            try:
                locs.append(actor.get_location())
            except Exception:
                pass
        return locs

    @staticmethod
    def _too_close(loc: carla.Location, others: List[carla.Location], min_dist: float) -> bool:
        for other in others:
            if ((loc.x - other.x) ** 2 + (loc.y - other.y) ** 2) ** 0.5 < min_dist:
                return True
        return False

    def _cleanup_all(self) -> None:
        """Destroy everything this server spawned (simulation thread, or start() failure)."""
        # Stop LiDAR sensor callbacks before batch-destroying vehicles
        if self._run is not None:
            try:
                self.stop_run("world stopped")
            except Exception:
                logger.exception("could not finish the active run")
        if self.lidar is not None:
            self.lidar.shutdown()

        self._unregister_background_stations()
        all_actors = (
            list(self._client_vehicles.values())
            + self._background_vehicles
            + self._pedestrian_controllers
            + self._pedestrians
        )
        if all_actors and self.client:
            batch = [carla.command.DestroyActor(a) for a in all_actors]
            try:
                self.client.apply_batch_sync(batch, False)
            except Exception as exc:
                logger.warning("Cleanup error: %s", exc)
        self._client_vehicles.clear()
        self._background_vehicles.clear()
        self._pedestrian_controllers.clear()
        self._pedestrians.clear()
        self._telemetry = {}
        self._v2x_payloads = {}
