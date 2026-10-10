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
import threading
import time
from concurrent.futures import Future
from typing import Any, Callable, Dict, List, Optional

import carla

from src.lidar.carla_backend import CarlaSensorBackend
from src.lidar.sensor import LidarLimits, LidarManager
from src.lidar.stream import SensorStream
from src.server.events import EventLog
from src.server.session_manager import SessionManager
from src.server.sim_executor import SimThreadExecutor
from src.server.world_config import WorldConfig

logger = logging.getLogger(__name__)

# Blueprints that are always available in CARLA 0.9.x Town maps.
_FALLBACK_BLUEPRINTS = [
    "vehicle.tesla.model3",
    "vehicle.audi.a2",
    "vehicle.lincoln.mkz_2020",
]

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

    def start(self) -> None:
        """Connect to CARLA, apply settings, start the simulation thread."""
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
            self.v2x_network = V2XNetwork(
                awareness_range_m=self.awareness_range_m,
                update_rate_hz=self.config.v2x_update_rate_hz,
                world=self.world,
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
            # Real-time pacing: record wall-clock time before tick so we can
            # sleep the remainder of fixed_delta_seconds afterwards.
            tick_start = time.monotonic()
            try:
                self._exec.drain()  # submitted work runs here, at a frame boundary
                self._tick()
                snapshot = self.world.get_snapshot()
                self._publish_telemetry(snapshot)
                self._collect_lidar(snapshot)
                self._run_periodic_work(snapshot, every_n)

            except RuntimeError as exc:
                logger.error("Tick error (frame %d): %s", self._frame, exc)
                time.sleep(0.1)  # brief back-off before retry
                continue

            elapsed = time.monotonic() - tick_start
            sleep_for = self.fixed_delta_seconds - elapsed
            if sleep_for > 0.001:
                time.sleep(sleep_for)

        logger.info("Tick loop stopped at frame %d", self._frame)

    def _collect_lidar(self, snapshot) -> None:
        """Sensor state machine and this frame's measurements. An error here must not stop the simulation."""
        if self.lidar is None:
            return
        try:
            self.lidar.collect(snapshot.frame, float(snapshot.timestamp.elapsed_seconds))
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
            if self.v2x_network is not None:
                try:
                    self.v2x_network.update(snapshot)
                    self._publish_v2x()
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

    def _event(self, kind: str, **data: Any) -> None:
        self.events.add(kind, frame=self._frame, sim_time=self._frame * self.fixed_delta_seconds, **data)

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
