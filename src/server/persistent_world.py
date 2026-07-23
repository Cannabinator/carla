"""
Persistent CARLA world — the heart of the live-server architecture.

The admin starts this once; it runs indefinitely in a background thread.
Clients (laptop, raspi, workstation…) join and leave dynamically — each
gets one "leading vehicle" tracked here.

Usage
-----
    world = PersistentCARLAServer(host, port, config)
    world.start()          # connect to CARLA, start tick loop
    ...
    world.stop()           # graceful shutdown + actor cleanup

Thread safety
-------------
All actor-list mutations go through ``_lock``.  The tick thread only reads
the vehicle dict (for V2V updates) and calls ``_check_expired_sessions``
which re-acquires the lock for any despawn operations.
"""

import carla
import threading
import logging
import random
import time
from typing import Optional, Dict, List, Callable

from src.server.session_manager import SessionManager, ClientSession

logger = logging.getLogger(__name__)

# Blueprints that are always available in CARLA 0.9.x Town maps.
_FALLBACK_BLUEPRINTS = [
    "vehicle.tesla.model3",
    "vehicle.audi.a2",
    "vehicle.lincoln.mkz_2020",
]


class PersistentCARLAServer:
    """
    Manages a persistent CARLA world that runs indefinitely.

    Background traffic and pedestrians are controlled by the admin via the
    REST API.  Each client session spawns exactly one "leading vehicle".
    """

    def __init__(
        self,
        host: str,
        port: int,
        *,
        fixed_delta_seconds: float = 0.05,
        timeout: float = 30.0,
        tm_port: int = 8001,
        tm_seed: int = 42,
        global_speed_pct: float = -20.0,
        safety_distance: float = 2.5,
        v2v_enabled: bool = True,
        v2v_range: float = 75.0,
    ):
        self.host = host
        self.port = port
        self.fixed_delta_seconds = fixed_delta_seconds
        self.timeout = timeout
        self.tm_port = tm_port
        self.tm_seed = tm_seed
        self.global_speed_pct = global_speed_pct
        self.safety_distance = safety_distance
        self.v2v_enabled = v2v_enabled
        self.v2v_range = v2v_range

        # CARLA handles
        self.client: Optional[carla.Client] = None
        self.world: Optional[carla.World] = None
        self.tm: Optional[carla.TrafficManager] = None
        self.bp_lib: Optional[carla.BlueprintLibrary] = None
        self.spawn_points: List[carla.Transform] = []
        self._original_settings: Optional[carla.WorldSettings] = None

        # Managed actors
        self._background_vehicles: List[carla.Actor] = []
        self._pedestrians: List[carla.Actor] = []
        self._pedestrian_controllers: List[carla.Actor] = []
        self._client_vehicles: Dict[str, carla.Actor] = {}  # session_id → actor

        # Session manager (shared with HTTP routes)
        self.sessions = SessionManager()

        # V2V / LiDAR references (set externally or created here)
        self.v2v_network = None
        self.lidar_collector = None

        # State
        self._running = False
        self._tick_thread: Optional[threading.Thread] = None
        self._lock = threading.Lock()
        self._frame = 0
        self._start_time: Optional[float] = None

        # Callback fired when a session expires (so WS can notify the client)
        self.on_session_expired: Optional[Callable[[str], None]] = None

    # ------------------------------------------------------------------ #
    # Lifecycle                                                            #
    # ------------------------------------------------------------------ #

    def start(self) -> None:
        """Connect to CARLA and start the continuous tick loop."""
        logger.info("Connecting to CARLA at %s:%s", self.host, self.port)

        self.client = carla.Client(self.host, self.port)
        self.client.set_timeout(self.timeout)
        self.world = self.client.get_world()

        logger.info("Connected to map: %s", self.world.get_map().name)

        # Store original settings for restoration on stop
        self._original_settings = self.world.get_settings()

        # Apply synchronous mode + physics substepping on a fresh copy.
        # Per CARLA docs: substepping keeps physics dt <= 0.01 s for accurate
        # vehicle dynamics.  Constraint: fixed_delta_seconds <= max_substep_delta_time
        # * max_substeps  →  0.05 <= 0.01 * 10 = 0.10 ✓
        settings = self.world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = self.fixed_delta_seconds
        settings.substepping = True
        settings.max_substep_delta_time = 0.01   # optimal physics step (< 0.01666 s)
        settings.max_substeps = 10
        self.world.apply_settings(settings)

        # Traffic Manager
        self.tm = self.client.get_trafficmanager(self.tm_port)
        self.tm.set_synchronous_mode(True)
        self.tm.set_random_device_seed(self.tm_seed)
        self.tm.set_global_distance_to_leading_vehicle(self.safety_distance)
        self.tm.global_percentage_speed_difference(self.global_speed_pct)

        self.bp_lib = self.world.get_blueprint_library()
        self.spawn_points = self.world.get_map().get_spawn_points()
        logger.info("Map has %d spawn points", len(self.spawn_points))

        # Optional V2V network
        if self.v2v_enabled:
            self._init_v2v()

        self._running = True
        self._start_time = time.monotonic()
        self._tick_thread = threading.Thread(
            target=self._tick_loop, daemon=True, name="carla-tick"
        )
        self._tick_thread.start()
        logger.info("Persistent CARLA server started (%.0f FPS target)", 1 / self.fixed_delta_seconds)

    def stop(self) -> None:
        """Stop the tick loop and destroy all managed actors."""
        logger.info("Stopping persistent CARLA server…")
        self._running = False

        if self._tick_thread and self._tick_thread.is_alive():
            self._tick_thread.join(timeout=5.0)

        self._cleanup_all()

        if self.world and self._original_settings:
            try:
                self.world.apply_settings(self._original_settings)
                logger.info("World settings restored")
            except Exception as e:
                logger.warning("Could not restore world settings: %s", e)

        logger.info("Persistent CARLA server stopped")

    def _init_v2v(self) -> None:
        try:
            from src.v2v.network_enhanced import V2VNetworkEnhanced
            self.v2v_network = V2VNetworkEnhanced(
                max_range=self.v2v_range,
                update_rate_hz=2.0,
                world=self.world,
            )
            logger.info("V2V network initialised (range=%.0f m)", self.v2v_range)
        except ImportError:
            logger.warning("V2V module not available — V2V disabled")
            self.v2v_network = None

    # ------------------------------------------------------------------ #
    # Tick loop                                                            #
    # ------------------------------------------------------------------ #

    def _tick_loop(self) -> None:
        logger.info("Tick loop running…")
        while self._running:
            # Real-time pacing: record wall-clock time before tick so we can
            # sleep the remainder of fixed_delta_seconds afterwards.  This keeps
            # the simulation at ≈ 1× real-time speed for a realistic V2X feel.
            tick_start = time.monotonic()
            try:
                self.world.tick()
                self._frame += 1

                # V2V update at 2 Hz (every 10 frames @ 20 FPS)
                if self._frame % 10 == 0 and self.v2v_network is not None:
                    snapshot = self.world.get_snapshot()
                    self.v2v_network.update(snapshot)

                # Session expiry check every ~5 s (100 frames @ 20 FPS)
                if self._frame % 100 == 0:
                    self._check_expired_sessions()

            except RuntimeError as exc:
                logger.error("Tick error (frame %d): %s", self._frame, exc)
                time.sleep(0.1)  # brief back-off before retry
                continue

            # Sleep the remainder of the target frame period so the simulation
            # advances at real-time speed (real-time factor ≈ 1.0).
            elapsed = time.monotonic() - tick_start
            sleep_for = self.fixed_delta_seconds - elapsed
            if sleep_for > 0.001:
                time.sleep(sleep_for)

        logger.info("Tick loop stopped after %d frames", self._frame)

    def _check_expired_sessions(self) -> None:
        for session in self.sessions.get_expired_sessions():
            logger.info("Session %s expired — removing vehicle", session.session_id)
            self._despawn_client_vehicle_locked(session.session_id)
            self.sessions.end_session(session.session_id)
            if self.on_session_expired:
                self.on_session_expired(session.session_id)

    # ------------------------------------------------------------------ #
    # Admin — background traffic                                           #
    # ------------------------------------------------------------------ #

    def spawn_background_traffic(self, count: int) -> int:
        """Spawn *count* autopilot vehicles. Returns number actually spawned."""
        if not self.world:
            raise RuntimeError("Server not started")

        with self._lock:
            occupied = self._all_vehicle_locations()
            free_points = [
                sp for sp in self.spawn_points
                if not self._too_close(sp.location, occupied, min_dist=8.0)
            ]
            random.shuffle(free_points)
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
                bp = random.choice(bps)
                if bp.has_attribute("color"):
                    color = random.choice(bp.get_attribute("color").recommended_values)
                    bp.set_attribute("color", color)
                batch.append(
                    carla.command.SpawnActor(bp, sp).then(
                        carla.command.SetAutopilot(
                            carla.command.FutureActor, True, self.tm.get_port()
                        )
                    )
                )

            results = self.client.apply_batch_sync(batch, True)
            spawned = 0
            for r in results:
                if r.error:
                    logger.debug("Spawn error: %s", r.error)
                    continue
                actor = self.world.get_actor(r.actor_id)
                if actor:
                    self._background_vehicles.append(actor)
                    spawned += 1

            logger.info("Spawned %d / %d background vehicles", spawned, count)
            return spawned

    def despawn_background_traffic(self) -> int:
        """Destroy all admin-managed background vehicles."""
        with self._lock:
            count = len(self._background_vehicles)
            if count == 0:
                return 0
            batch = [carla.command.DestroyActor(v) for v in self._background_vehicles]
            self.client.apply_batch_sync(batch, True)
            self._background_vehicles.clear()
            logger.info("Despawned %d background vehicles", count)
            return count

    # ------------------------------------------------------------------ #
    # Admin — pedestrians                                                  #
    # ------------------------------------------------------------------ #

    def spawn_pedestrians(self, count: int) -> int:
        """Spawn *count* AI-controlled pedestrians. Returns number spawned."""
        if not self.world:
            raise RuntimeError("Server not started")

        with self._lock:
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
                bp = random.choice(ped_bps)
                if bp.has_attribute("is_invincible"):
                    bp.set_attribute("is_invincible", "false")
                spawn_batch.append(carla.command.SpawnActor(bp, carla.Transform(loc)))

            results = self.client.apply_batch_sync(spawn_batch, True)
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
            self.world.tick()  # needed before attaching controllers
            ctrl_results = self.client.apply_batch_sync(ctrl_batch, True)

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
            logger.info("Spawned %d / %d pedestrians", spawned, count)
            return spawned

    def despawn_pedestrians(self) -> int:
        """Destroy all admin-managed pedestrians."""
        with self._lock:
            for ctrl in self._pedestrian_controllers:
                try:
                    ctrl.stop()
                except Exception:
                    pass
            count = len(self._pedestrians)
            batch = (
                [carla.command.DestroyActor(c) for c in self._pedestrian_controllers]
                + [carla.command.DestroyActor(p) for p in self._pedestrians]
            )
            if batch:
                self.client.apply_batch_sync(batch, True)
            self._pedestrian_controllers.clear()
            self._pedestrians.clear()
            logger.info("Despawned %d pedestrians", count)
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
        if not self.world:
            raise RuntimeError("Server not started")

        session = self.sessions.get_session(session_id)
        if session is None:
            raise ValueError(f"Unknown session: {session_id}")

        with self._lock:
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

            if self.v2v_network is not None:
                self.v2v_network.register(actor.id, actor)

            logger.info(
                "Spawned client vehicle actor=%d for session %s (%s)",
                actor.id, session_id, session.client_name,
            )
            return actor.id

    def despawn_client_vehicle(self, session_id: str) -> None:
        """Public: despawn a client vehicle (called on leave or kick)."""
        with self._lock:
            self._despawn_client_vehicle_locked(session_id)

    def _despawn_client_vehicle_locked(self, session_id: str) -> None:
        """Must be called with ``_lock`` held (or from tick thread)."""
        actor = self._client_vehicles.pop(session_id, None)
        if actor is None:
            return
        try:
            if self.v2v_network is not None:
                self.v2v_network.unregister(actor.id)
        except Exception:
            pass
        try:
            actor.destroy()
        except Exception as exc:
            logger.warning("Error destroying client vehicle: %s", exc)
        logger.info("Despawned client vehicle for session %s", session_id)

    # ------------------------------------------------------------------ #
    # State queries (called from HTTP routes / WebSocket tasks)           #
    # ------------------------------------------------------------------ #

    def get_status(self) -> dict:
        elapsed = time.monotonic() - self._start_time if self._start_time else 0
        return {
            "running": self._running,
            "frame": self._frame,
            "elapsed_seconds": round(elapsed, 1),
            "map": self.world.get_map().name if self.world else None,
            "background_vehicles": len(self._background_vehicles),
            "pedestrians": len(self._pedestrians),
            "active_sessions": len(self.sessions.get_active_sessions()),
            "client_vehicles": len(self._client_vehicles),
            "v2v_enabled": self.v2v_network is not None,
        }

    def get_vehicle_telemetry(self, session_id: str) -> Optional[dict]:
        """Return live telemetry for a client's vehicle."""
        actor = self._client_vehicles.get(session_id)
        if actor is None:
            return None
        try:
            t = actor.get_transform()
            v = actor.get_velocity()
            speed_kmh = (v.x**2 + v.y**2 + v.z**2) ** 0.5 * 3.6
            return {
                "actor_id": actor.id,
                "x": round(t.location.x, 2),
                "y": round(t.location.y, 2),
                "z": round(t.location.z, 2),
                "yaw": round(t.rotation.yaw, 1),
                "speed_kmh": round(speed_kmh, 1),
                "frame": self._frame,
            }
        except Exception as exc:
            logger.debug("Telemetry read error for %s: %s", session_id, exc)
            return None

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
            return random.choice(free)
        return random.choice(self.spawn_points)

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
        with self._lock:
            all_actors = (
                list(self._client_vehicles.values())
                + self._background_vehicles
                + self._pedestrian_controllers
                + self._pedestrians
            )
            if all_actors and self.client:
                batch = [carla.command.DestroyActor(a) for a in all_actors]
                try:
                    self.client.apply_batch_sync(batch, True)
                except Exception as exc:
                    logger.warning("Cleanup error: %s", exc)
            self._client_vehicles.clear()
            self._background_vehicles.clear()
            self._pedestrian_controllers.clear()
            self._pedestrians.clear()
