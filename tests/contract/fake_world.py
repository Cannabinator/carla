"""A CARLA-free stand-in for PersistentCARLAServer, exposing what the routes use.

It uses the real SessionManager, so session behaviour (create/end/expire) is the
production code; only spawning and telemetry are faked.
"""

from concurrent.futures import Future

from src.lidar.sensor import LidarLimits, LidarManager
from src.server.events import EventLog
from src.server.published import Published
from src.server.v2x_publisher import V2XPublisher
from src.server.world_config import WorldConfig
from src.server.session_manager import SessionManager
from tests.unit.lidar_fake import FakeBackend


class FakeWorld:
    def __init__(self, running: bool = True, spawn_ok: bool = True):
        self.sessions = SessionManager()
        self._running = running
        self.spawn_ok = spawn_ok
        self.spawned = {}  # session_id -> actor_id
        self.despawned = []
        self.stopped = False
        self._next_actor = 100
        self.background = 0
        self.pedestrians = 0
        self.v2x_network = None
        self.fixed_delta_seconds = 0.05
        self.session_disconnect_grace_s = 0.0
        self.events = EventLog(1000)
        self.lidar_backend = FakeBackend()
        self.lidar = LidarManager(self.lidar_backend, LidarLimits(max_sensors=2, max_points_per_second=1_000_000,
                                                                   keep_alive_s=10.0), 0.05, frame_timeout_s=0.0)
        self._streams = {}
        self.config = WorldConfig(host="fake")
        self.map_geometry = {"map_name": "Fake", "version": "abc123", "step_m": 2.0, "junctions": [],
                             "lanes": [[[0.0, 0.0], [2.0, 0.0], [4.0, 0.0]]]}
        self._map_cached = False
        self.denms = {}
        self._denm_action = 0
        self.rsus = {-1: {"station_id": -1, "intersection_id": 1, "x": 0.0, "y": 0.0, "z": 3.5, "enabled": True,
                          "tx_power_dbm": 33.0, "broadcast_range_m": 300.0}}
        # what the hub reads (the real world publishes the same things from its simulation thread)
        self.observer_public = True
        self.v2x_links_max_rows = 2000
        self.frame_no = 42
        self._v2x = V2XPublisher()
        self._world_state = None
        self._world_state_version = 0

    def submit(self, fn, *args, **kwargs):
        """Mirror PersistentCARLAServer.submit: run now, hand back a Future."""
        fut = Future()
        if not self._running:
            fut.set_exception(RuntimeError("simulation is not running"))
            return fut
        try:
            fut.set_result(fn(*args, **kwargs))
        except BaseException as exc:  # noqa: BLE001
            fut.set_exception(exc)
        return fut

    # --- lifecycle -------------------------------------------------------
    def stop(self):
        self.stopped = True
        self._running = False

    def get_status(self):
        return {
            "running": self._running, "frame": 42, "elapsed_seconds": 1.0,
            "map": "Town01", "background_vehicles": self.background,
            "pedestrians": self.pedestrians,
            "active_sessions": len(self.sessions.get_active_sessions()),
            "client_vehicles": len(self.spawned), "v2x_enabled": False,
            "awareness_range_m": None, "rsu_count": 0, "vru_count": 0,
        }

    # --- client vehicles -------------------------------------------------
    def spawn_client_vehicle(self, session_id, blueprint_id):
        if not self.spawn_ok:
            return None
        self._next_actor += 1
        self.spawned[session_id] = self._next_actor
        session = self.sessions.get_session(session_id)
        if session is not None:
            session.vehicle_actor_id = self._next_actor
        return self._next_actor

    def despawn_client_vehicle(self, session_id):
        self.despawned.append(session_id)
        self.lidar.release_owned_by(session_id)
        self.spawned.pop(session_id, None)

    def get_vehicle_telemetry(self, session_id):
        if session_id not in self.spawned:
            return None
        return {"actor_id": self.spawned[session_id], "x": 1.0, "y": 2.0, "z": 0.0,
                "yaw": 90.0, "speed_kmh": 12.5, "frame": self.frame_no, "sim_time": self.frame_no * 0.05}

    def get_vehicle_v2x_data(self, session_id):
        return {"neighbors": [], "threats": []}

    # --- hot configuration and RSUs (same exceptions as the real world raises) ------------
    def apply_config(self, changes):
        from src.server.world_config import HOT_FIELDS
        unknown = [k for k in changes if k not in WorldConfig.model_fields]
        if unknown:
            raise ValueError(f"unknown configuration fields: {', '.join(unknown)}")
        restart = [k for k in changes if k not in HOT_FIELDS]
        if restart:
            raise PermissionError(f"these fields need a new world (restart): {', '.join(restart)}")
        new = WorldConfig.model_validate({**self.config.model_dump(), **changes})
        applied = {k: {"old": getattr(self.config, k), "new": getattr(new, k), "effective": HOT_FIELDS[k]}
                   for k in changes if getattr(self.config, k) != getattr(new, k)}
        self.config = new
        if "observer_public" in applied:
            self.observer_public = new.observer_public
        return {"applied": applied, "applied_at_frame": self.frame_no, "sim_time": self.frame_no * 0.05}

    def set_rsu(self, station_id, enabled=None, tx_power_dbm=None):
        if station_id >= 0 or station_id not in self.rsus:
            raise LookupError(f"no RSU with station id {station_id}")
        r = self.rsus[station_id]
        if enabled is not None:
            r["enabled"] = enabled
        if tx_power_dbm is not None:
            r["tx_power_dbm"] = tx_power_dbm
        self.publish_world_state([])
        return {"station_id": station_id, "applied_at_frame": self.frame_no, "enabled": r["enabled"], "tx_power_dbm": r["tx_power_dbm"]}

    # --- map / DENM -----------------------------------------------------------
    def get_map_geometry(self):
        self._map_cached = True
        return self.map_geometry

    def cached_map_geometry(self):
        return self.map_geometry if self._map_cached else None

    def inject_denm(self, x, y, z, cause_code, subcause_code, relevance_distance_m, information_quality,
                    station_id=None, duration_s=None):
        if station_id is not None and station_id not in self.spawned.values():
            raise ValueError(f"station {station_id} is not a registered V2X vehicle")
        self._denm_action += 1
        sid = station_id if station_id is not None else -(1_000_000 + self._denm_action)
        self.denms[(sid, self._denm_action)] = {"x": x, "y": y, "cause_code": cause_code,
                                                "relevance_distance_m": relevance_distance_m}
        return {"station_id": sid, "action_id": self._denm_action, "expires_sim_time": None}

    def cancel_denm(self, station_id, action_id):
        return self.denms.pop((station_id, action_id), None) is not None

    def list_denm(self):
        return [{"station_id": k[0], "action_id": k[1], **v} for k, v in self.denms.items()]

    # --- published views for the hub ---------------------------------------
    def publish_world_state(self, stations):
        self._world_state_version += 1
        self._world_state = Published(self._world_state_version, self.frame_no, self.frame_no * 0.05,
                                      {"stations": stations, "rsus": list(self.rsus.values())})

    def get_world_state(self):
        return self._world_state

    def publish_v2x(self, frame):
        self._v2x.on_update(frame)

    def get_v2x_frame(self):
        return self._v2x.frame

    def get_v2x_published(self):
        return self._v2x.summary

    def get_v2x_stats(self):
        return self._v2x.stats

    def get_run_status(self):
        return Published(self.frame_no // 20, self.frame_no, self.frame_no * 0.05,
                         {"running": self._running, "frame": self.frame_no, "real_time_factor": 1.0})

    # --- LiDAR: the real manager over a fake CARLA backend ---------------
    def lidar_request(self, principal, actor_id, config=None, **kw):
        return self.lidar.request(principal, actor_id, config, **kw)

    def lidar_reconfigure(self, principal, sensor_id, config):
        self.lidar.reconfigure(principal, sensor_id, config)

    def lidar_set_flags(self, principal, sensor_id, pinned=None, record_raw=None):
        self.lidar.set_flags(principal, sensor_id, pinned, record_raw)

    def lidar_release(self, principal, sensor_id):
        self.lidar.release(principal, sensor_id)

    def get_lidar_stream(self, sensor_id):
        from src.lidar.stream import SensorStream
        return self._streams.setdefault(sensor_id, SensorStream(sensor_id, 20.0))

    def lidar_tick(self, frame):
        """One simulation step for the LiDAR manager: deliver data for ``frame`` and collect it."""
        self.lidar_backend.emit_all(frame, n=200)
        return self.lidar.collect(frame, frame * 0.05)

    # --- admin population ------------------------------------------------
    def spawn_background_traffic(self, count):
        self.background += count
        return count

    def despawn_background_traffic(self):
        n, self.background = self.background, 0
        return n

    def spawn_pedestrians(self, count):
        self.pedestrians += count
        return count

    def despawn_pedestrians(self):
        n, self.pedestrians = self.pedestrians, 0
        return n
