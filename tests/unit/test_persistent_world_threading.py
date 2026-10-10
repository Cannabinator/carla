"""Threading rules of PersistentCARLAServer, checked against a recording stub of ``carla``.

Phase 2 invariants (REFACTOR_HANDOFF section 4):
  * after start() only the simulation thread ("carla-tick") calls CARLA        (C1-C5)
  * nothing ticks the world behind the tick loop's back, _frame == CARLA frame (C1, C2)
  * published telemetry carries the frame it belongs to, readers make no RPC   (C10)
  * same seed -> same spawn choices                                            (C8)
"""

import importlib
import sys
import threading
import time

import pytest

from src.server.world_config import WorldConfig
from tests.unit.carla_stub import Recorder, make_carla_module

SIM = "carla-tick"


@pytest.fixture
def env(monkeypatch):
    rec = Recorder()
    monkeypatch.setitem(sys.modules, "carla", make_carla_module(rec))
    sys.modules.pop("src.server.persistent_world", None)
    pw = importlib.import_module("src.server.persistent_world")
    servers = []

    def make(**cfg):
        srv = pw.PersistentCARLAServer(WorldConfig(host="stub", **cfg))
        srv.start()
        servers.append(srv)
        return srv

    yield rec, make
    for s in servers:
        s.stop()
    sys.modules.pop("src.server.persistent_world", None)


def _calls_after(rec, mark):
    return rec.calls[mark:]


def _join(srv, name="c1"):
    session = srv.sessions.create_session(name, "127.0.0.1", "vehicle.test.car0", None)
    actor_id = srv.submit(srv.spawn_client_vehicle, session.session_id, "vehicle.test.car0").result(5)
    return session, actor_id


def test_only_the_sim_thread_calls_carla_after_start(env):
    rec, make = env
    srv = make()
    mark = len(rec.calls)

    srv.submit(srv.spawn_background_traffic, 5).result(5)
    srv.submit(srv.spawn_pedestrians, 3).result(5)
    session, actor_id = _join(srv)
    # readers on this (non-sim) thread
    srv.get_status(); srv.get_vehicle_telemetry(session.session_id); srv.get_vehicle_v2x_data(session.session_id)
    time.sleep(0.2)
    srv.submit(srv.despawn_client_vehicle, session.session_id).result(5)
    srv.submit(srv.despawn_pedestrians).result(5)
    srv.submit(srv.despawn_background_traffic).result(5)

    foreign = [c for c in _calls_after(rec, mark) if c[1] != SIM]
    assert foreign == [], f"CARLA called from non-simulation threads: {foreign[:5]}"
    assert any(c[0] == "world.tick" for c in _calls_after(rec, mark))


def test_mutating_methods_refuse_foreign_threads(env):
    rec, make = env
    srv = make()
    for fn, args in [(srv.spawn_background_traffic, (1,)), (srv.despawn_background_traffic, ()),
                     (srv.spawn_pedestrians, (1,)), (srv.despawn_pedestrians, ()),
                     (srv.despawn_client_vehicle, ("x",)), (srv.lidar_request, (None, 1)),
                     (srv.lidar_reconfigure, (None, 1, None)), (srv.lidar_release, (None, 1))]:
        with pytest.raises(RuntimeError, match="simulation thread"):
            fn(*args)


def test_no_batch_ticks_the_world_and_frame_matches_carla(env):
    rec, make = env
    srv = make()
    srv.submit(srv.spawn_background_traffic, 4).result(5)
    srv.submit(srv.spawn_pedestrians, 2).result(5)   # needs one explicit tick for controllers
    _join(srv)
    srv.submit(srv.despawn_pedestrians).result(5)
    srv.submit(srv.despawn_background_traffic).result(5)
    batches = [c for c in rec.calls if c[0] == "client.apply_batch_sync"]
    assert batches and not any(c[2]["do_tick"] for c in batches), "a batch ticked the world (C1)"
    assert not any(c[0] == "batch.implicit_tick" for c in rec.calls)
    srv.submit(lambda: None).result(5)
    time.sleep(0.15)
    # _frame is CARLA's frame id (no drift from the extra pedestrian tick, C2); the stub's
    # frame may be one tick ahead if the loop ticked between our reads.
    assert 0 <= rec.frame - srv._frame <= 2


def test_telemetry_is_stamped_with_the_snapshot_frame_and_needs_no_rpc(env):
    rec, make = env
    srv = make()
    session, actor_id = _join(srv)
    first = srv.get_vehicle_telemetry(session.session_id)
    assert first and first["actor_id"] == actor_id  # available immediately after join
    time.sleep(0.3)
    mark = len(rec.calls)
    tele = srv.get_vehicle_telemetry(session.session_id)
    assert tele["frame"] > first["frame"], "telemetry is not refreshed every frame"
    assert tele["frame"] <= rec.frame and tele["speed_kmh"] == 18.0  # (3,4,0) m/s
    assert [c for c in rec.calls[mark:] if c[1] != SIM] == []


def test_submit_after_stop_fails_instead_of_touching_carla(env):
    rec, make = env
    srv = make()
    srv.stop()
    with pytest.raises(RuntimeError):
        srv.submit(srv.spawn_background_traffic, 1).result(5)


def test_stop_destroys_everything_on_the_sim_thread_and_restores_settings(env):
    rec, make = env
    srv = make()
    srv.submit(srv.spawn_background_traffic, 3).result(5)
    _join(srv)
    mark = len(rec.calls)
    srv.stop()
    after = rec.calls[mark:]
    assert any(c[0] == "client.apply_batch_sync" and c[1] == SIM for c in after), "no cleanup batch on sim thread"
    assert any(c[0] == "world.apply_settings" for c in after), "world settings not restored"
    assert rec.actors == {}, f"actors left behind: {list(rec.actors)}"
    assert not threading.enumerate() or SIM not in [t.name for t in threading.enumerate()]


def test_same_seed_gives_same_spawn_choices(env):
    rec, make = env

    def run(seed):
        before = set(rec.actors)
        srv = make(seed=seed)
        srv.submit(srv.spawn_background_traffic, 6).result(5)
        spawned = sorted(
            (a.transform.location.x, a.bp.id) for i, a in rec.actors.items() if i not in before
        )
        srv.stop()
        return spawned

    assert run(7) == run(7)
    assert run(7) != run(8)


def test_expired_session_is_cleaned_up_by_the_sim_thread(env):
    rec, make = env
    srv = make()
    session = srv.sessions.create_session("tmp", "127.0.0.1", "vehicle.test.car0", 10)
    srv.submit(srv.spawn_client_vehicle, session.session_id, "vehicle.test.car0").result(5)
    session.expires_at = session.spawn_time  # already expired
    mark = len(rec.calls)
    deadline = time.time() + 10
    while time.time() < deadline and session.is_active:
        time.sleep(0.05)
    assert not session.is_active, "expired session was never ended"
    assert session.session_id not in srv._client_vehicles
    assert [c for c in rec.calls[mark:] if c[1] != SIM] == []


def test_v2x_failure_does_not_stop_the_simulation(env):
    rec, make = env
    srv = make()

    def boom(snapshot, **kw):
        raise ValueError("v2x exploded")

    srv.v2x_network.update = boom
    f0 = srv._frame
    time.sleep(1.0)  # > 10 frames at 20 FPS: the V2X update runs, fails, loop carries on
    assert srv._tick_thread.is_alive() and srv._running
    assert srv._frame > f0 + 10
    assert srv.submit(lambda: 42).result(5) == 42


# ── S3: V2X penetration rate of background traffic (decision D2) ────────────────

def _spawn_background(srv, n):
    srv.submit(srv.spawn_background_traffic, n).result(5)
    return sorted((a.transform.location.x, a.bp.id) for a in srv._background_vehicles)


def test_default_penetration_equips_every_background_vehicle(env):
    rec, make = env
    srv = make()
    _spawn_background(srv, 12)
    assert len(srv.v2x_network.vehicles) == 12
    assert srv.get_status()["v2x_stations"] == 12 and srv.get_status()["v2x_penetration_rate"] == 1.0


def test_zero_penetration_equips_no_background_vehicle_but_clients_still_transmit(env):
    rec, make = env
    srv = make(v2x_penetration_rate=0.0)
    _spawn_background(srv, 12)
    assert len(srv.v2x_network.vehicles) == 0
    session, actor_id = _join(srv)
    assert set(srv.v2x_network.vehicles) == {actor_id}


def test_partial_penetration_is_a_seeded_subset(env):
    rec, make = env

    def run(seed):
        srv = make(v2x_penetration_rate=0.5, seed=seed)
        _spawn_background(srv, 30)
        chosen = sorted(srv.v2x_network.vehicles)
        # identify the equipped ones by spawn position, since actor ids differ between runs
        pos = sorted(srv._background_vehicles[[a.id for a in srv._background_vehicles].index(i)].transform.location.x
                     for i in chosen)
        srv.stop()
        return pos

    first = run(5)
    assert 5 <= len(first) <= 25, "a rate of 0.5 should equip roughly half of 30 vehicles"
    assert run(5) == first, "same seed must give the same equipped vehicles"
    assert run(6) != first


def test_penetration_rate_does_not_change_which_vehicles_spawn(env):
    """The equipped choice has its own random stream (checked: traffic layout is identical)."""
    rec, make = env

    def layout(rate):
        srv = make(v2x_penetration_rate=rate, seed=11)
        out = _spawn_background(srv, 15)
        srv.stop()
        return out

    assert layout(0.0) == layout(1.0) == layout(0.4)


def test_stations_are_removed_when_the_traffic_is_cleared(env):
    rec, make = env
    srv = make()
    _spawn_background(srv, 8)
    session, actor_id = _join(srv)
    assert len(srv.v2x_network.vehicles) == 9
    srv.submit(srv.despawn_background_traffic).result(5)
    assert set(srv.v2x_network.vehicles) == {actor_id}


def test_penetration_with_v2x_disabled_is_harmless(env):
    rec, make = env
    srv = make(v2x_enabled=False)
    _spawn_background(srv, 5)
    assert srv.v2x_network is None and len(srv._background_vehicles) == 5


# ── S7: one RSU per intersection ───────────────────────────────────────────────────

def _junctions(rec, sizes):
    from tests.unit.carla_stub import TrafficLight
    next_id, lights = 100, []
    for n, size in enumerate(sizes):
        grp = [TrafficLight(next_id + i, n * 200.0 + i, 0.0) for i in range(size)]
        for tl in grp:
            tl.group = list(grp)
        lights += grp
        next_id += 100
    rec.traffic_lights = lights


def test_one_rsu_per_intersection_not_per_light(env):
    rec, make = env
    _junctions(rec, [4, 4, 2, 1])
    srv = make()
    rsus = srv.v2x_network._rsus
    assert sorted(rsus) == [1, 2, 3, 4]
    assert [len(rsus[i]._traffic_lights) for i in sorted(rsus)] == [4, 4, 2, 1]


def test_all_intersections_get_an_rsu_by_default_and_the_cap_is_configurable(env):
    rec, make = env
    _junctions(rec, [2] * 15)
    assert len(make().v2x_network._rsus) == 15          # the old code stopped at 10
    assert len(make(rsu_max_count=3).v2x_network._rsus) == 3
    assert len(make(rsu_max_count=0).v2x_network._rsus) == 0


def test_rsu_creation_is_deterministic(env):
    rec, make = env
    _junctions(rec, [3, 3, 3])
    first = [[tl.id for tl in r._traffic_lights] for _, r in sorted(make().v2x_network._rsus.items())]
    rec.traffic_lights = list(reversed(rec.traffic_lights))
    second = [[tl.id for tl in r._traffic_lights] for _, r in sorted(make().v2x_network._rsus.items())]
    assert first == second


# ── S4: a client's SPAT / VRUAM view is what its radio received ────────────────────────

def _wait_for_v2x_payload(srv, sid, timeout=8.0):
    t0 = time.time()
    while time.time() - t0 < timeout:
        p = srv.get_vehicle_v2x_data(sid)
        if p and p.get("v2x_frame") is not None:
            return p
        time.sleep(0.05)
    raise AssertionError("no V2X payload published")


def test_client_payload_lists_spat_it_received_and_not_rsus_out_of_coverage(env):
    rec, make = env
    from tests.unit.carla_stub import TrafficLight
    near = [TrafficLight(100, 5.0, 0.0), TrafficLight(101, 6.0, 0.0)]
    far = [TrafficLight(200, 700.0, 0.0)]
    for grp in (near, far):
        for tl in grp:
            tl.group = list(grp)
    rec.traffic_lights = near + far
    srv = make()
    session = srv.sessions.create_session("c", "127.0.0.1", "vehicle.test.car0", None)
    srv.submit(srv.spawn_client_vehicle, session.session_id, "vehicle.test.car0", 0).result(5)  # spawn point 0 = (0, 0)
    payload = _wait_for_v2x_payload(srv, session.session_id)
    ids = [s["intersection_id"] for s in payload["spat_nearby"]]
    assert ids == [1], f"only the RSU within 300 m can have been received, got {ids}"
    spat = payload["spat_nearby"][0]
    assert spat["rx_power_dbm"] < 0 and len(spat["states"]) == 2          # two lights = two signal groups
    assert payload["v2x_frame"] >= 0 and payload["v2x_sim_time"] > 0     # the view is stamped


def test_client_payload_lists_the_denms_it_received(env):
    rec, make = env
    from src.v2x.messages import DENM, DENMCauseCode
    srv = make()
    session = srv.sessions.create_session("c", "127.0.0.1", "vehicle.test.car0", None)
    srv.submit(srv.spawn_client_vehicle, session.session_id, "vehicle.test.car0", 0).result(5)   # at (0, 0)
    srv.submit(srv.v2x_network.publish_denm, DENM(
        station_id=9001, action_id=7, detection_time=0.0, reference_time=0.0,
        event_position=(40.0, 0.0, 0.0), cause_code=DENMCauseCode.OBSTACLE_ON_ROAD, relevance_distance=150.0)).result(5)
    deadline = time.time() + 8
    got = []
    while time.time() < deadline and not got:
        p = srv.get_vehicle_v2x_data(session.session_id)
        got = (p or {}).get("denm_received", [])
        time.sleep(0.05)
    assert got and got[0]["cause_code"] == "OBSTACLE_ON_ROAD" and got[0]["distance_to_event"] == 40.0
    assert got[0]["action_id"] == 7 and got[0]["rx_power_dbm"] < 0


def test_lidar_sensor_lifecycle_runs_on_the_sim_thread_and_sensors_die_before_their_vehicle(env):
    from src.lidar.config import LidarConfig
    from src.lidar.sensor import Principal
    rec, make = env
    srv = make()
    mark = len(rec.calls)
    session, actor_id = _join(srv)
    sid = srv.submit(srv.lidar_request, Principal("vehicle", session.session_id, actor_id), actor_id,
                     LidarConfig()).result(5)
    srv.lidar.attach_viewer(sid, "test-viewer")
    deadline = time.time() + 5
    while srv.lidar.state_of(sid).value != "active" and time.time() < deadline:
        time.sleep(0.02)
    assert srv.lidar.state_of(sid).value == "active"
    frame = srv.lidar.get_frame(sid)
    assert frame.num_points == 10 and frame.ego_actor_id == actor_id and frame.points["x"][0] == frame.frame

    sensor_actor = next(a for a in rec.actors.values() if hasattr(a, "measurement"))
    srv.submit(srv.despawn_client_vehicle, session.session_id).result(5)
    assert sensor_actor.destroyed and srv.lidar.state_of(sid).value == "destroyed"
    names = [(c[0], c[2]) for c in _calls_after(rec, mark)]
    assert names.index(("actor.destroy", sensor_actor.id)) < names.index(("actor.destroy", actor_id)), \
        "the sensor must be destroyed before the vehicle it is attached to"
    foreign = [c for c in _calls_after(rec, mark) if c[1] not in (SIM, "carla-sensor-callback") and c[1] != threading.current_thread().name]
    assert foreign == [], f"CARLA called from non-simulation threads: {foreign[:5]}"


def test_server_stop_destroys_lidar_sensors(env):
    from src.lidar.config import LidarConfig
    from src.lidar.sensor import ADMIN
    rec, make = env
    srv = make()
    _, actor_id = _join(srv)
    sid = srv.submit(srv.lidar_request, ADMIN, actor_id, LidarConfig(), pinned=True).result(5)
    time.sleep(0.3)
    sensors = [a for a in rec.actors.values() if hasattr(a, "measurement")]
    assert len(sensors) == 1 and srv.lidar.state_of(sid).value == "active"
    srv.stop()
    assert all(s.destroyed for s in sensors)


def _wait(cond, seconds=6.0):
    end = time.time() + seconds
    while time.time() < end:
        if cond():
            return True
        time.sleep(0.05)
    return False


def test_disconnected_session_is_removed_by_the_sim_thread_after_the_grace_period(env):
    rec, make = env
    srv = make(session_disconnect_grace_s=1.0)
    session, actor_id = _join(srv)
    srv.sessions.mark_connected(session.session_id)
    time.sleep(2.5)                                   # connected: never removed, however long
    assert session.is_active and actor_id in rec.actors

    srv.sessions.mark_disconnected(session.session_id)
    time.sleep(0.4)
    assert session.is_active and actor_id in rec.actors, "removed before the grace period ended"
    assert _wait(lambda: not session.is_active), "not removed after the grace period"
    assert actor_id not in rec.actors
    kinds = [e["kind"] for e in srv.events.since(0)[0]]
    assert "client_vehicle_spawned" in kinds and "session_disconnected_removed" in kinds


def test_reconnecting_within_the_grace_period_keeps_the_session(env):
    rec, make = env
    srv = make(session_disconnect_grace_s=1.5)
    session, actor_id = _join(srv)
    srv.sessions.mark_connected(session.session_id)
    srv.sessions.mark_disconnected(session.session_id)
    time.sleep(0.8)
    srv.sessions.mark_connected(session.session_id)
    time.sleep(2.5)
    assert session.is_active and actor_id in rec.actors


def test_lidar_transitions_end_up_in_the_event_log(env):
    from src.lidar.config import LidarConfig
    from src.lidar.sensor import ADMIN
    rec, make = env
    srv = make()
    _, actor_id = _join(srv)
    srv.submit(srv.lidar_request, ADMIN, actor_id, LidarConfig(), pinned=True).result(5)
    assert _wait(lambda: any(e["kind"] == "lidar_active" for e in srv.events.since(0)[0]))


# ── what the WebSocket hub reads ──────────────────────────────────────────────────────────────────

def test_published_world_state_run_status_and_v2x_views(env):
    import json
    rec, make = env
    srv = make()
    session, actor_id = _join(srv, "car-a")
    srv.submit(srv.spawn_background_traffic, 3).result(5)
    srv.submit(srv.spawn_pedestrians, 2).result(5)
    assert _wait(lambda: srv.get_v2x_frame() is not None and srv.get_run_status() is not None
                 and srv.get_world_state() is not None, 8)

    ws = srv.get_world_state()
    by_id = {s["id"]: s for s in ws.data["stations"]}
    me = by_id[actor_id]
    assert me["kind"] == "vehicle" and me["role"] == "client" and me["name"] == "car-a" and me["v2x"] is True
    assert sum(1 for s in ws.data["stations"] if s["role"] == "background" and s["kind"] == "vehicle") == 3
    assert ws.frame is not None and ws.sim_time == pytest.approx(ws.frame * 0.05, abs=1e-6)
    text = json.dumps(ws.data)
    assert session.session_id not in text and session.session_token not in text

    rs = srv.get_run_status().data
    assert rs["tick_duration_ms"]["p50"] is not None and rs["tick_duration_ms"]["p50"] < 25
    assert rs["real_time_factor"] is None or 0.5 < rs["real_time_factor"] < 1.5
    assert rs["sessions"] == 1 and rs["lidar_budget"]["max_sensors"] == 4

    summary = srv.get_v2x_published()
    assert actor_id in {s["station_id"] for s in summary.data["stations"]} and summary.frame == srv.get_v2x_frame().frame
    assert srv.get_v2x_stats().data["window"]["updates"] >= 1


def test_world_state_versions_increase_and_follow_the_configured_rate(env):
    rec, make = env
    srv = make(world_state_rate_hz=10.0)
    _join(srv)
    assert _wait(lambda: srv.get_world_state() is not None)
    v0, f0 = srv.get_world_state().version, srv.get_world_state().frame
    time.sleep(1.0)
    v1, f1 = srv.get_world_state().version, srv.get_world_state().frame
    assert v1 > v0 and (f1 - f0) / (v1 - v0) == pytest.approx(2.0, abs=0.01)     # every 2nd frame at 20 FPS


def test_a_stalled_hub_client_does_not_slow_the_simulation(env):
    import asyncio
    from src.server.ws_hub import Hub
    from tests.contract.ws_helpers import SlowSocket
    rec, make = env
    srv = make()
    _join(srv)
    srv.submit(srv.spawn_background_traffic, 5).result(5)
    time.sleep(1.0)

    def frames_in(seconds):
        f0 = rec.frame
        time.sleep(seconds)
        return rec.frame - f0

    baseline = min(frames_in(2.0), frames_in(2.0))          # the slower of two undisturbed runs (CI machines are noisy)

    async def scenario():
        hub = Hub(srv, send_timeout_s=30.0)
        ws = SlowSocket(send_delay=30.0)            # a reader that never finishes reading
        task = asyncio.create_task(hub.serve(ws))
        for topic, flt in [("world.state", {}), ("v2x.frame", {}), ("v2x.stats", {}), ("run.status", {}),
                           ("lidar.sensors", {}), ("events", {}), ("v2x.links", {"outcome": "all"})]:
            ws.feed(op="subscribe", topic=topic, filter=flt, max_hz=10)
        await asyncio.sleep(0.3)
        # the simulation keeps its pace while the hub is blocked in a send
        loop = asyncio.get_running_loop()
        f0 = rec.frame
        await loop.run_in_executor(None, time.sleep, 2.0)
        advanced = rec.frame - f0
        task.cancel()
        await asyncio.gather(task, return_exceptions=True)
        return advanced

    with_client = asyncio.run(scenario())
    assert baseline >= 30, baseline
    assert with_client >= 0.75 * baseline, (baseline, with_client)
    p95 = srv.get_run_status()
    assert p95 is None or p95.data["tick_duration_ms"]["p95"] < 30


# ── DENM injection and map geometry on the (stub) world ───────────────────────────────────────────

def test_injected_denm_is_transmitted_listed_and_cancelled(env):
    rec, make = env
    srv = make()
    session, actor_id = _join(srv)
    assert _wait(lambda: srv.get_v2x_frame() is not None)
    loc = rec.actors[actor_id].transform.location
    out = srv.submit(srv.inject_denm, loc.x + 5, loc.y, 0.0, 10, 0, 200.0, 7).result(5)
    assert out["action_id"] == 1 and out["station_id"] <= -1_000_001 and out["expires_sim_time"] is None

    def denm_links():
        f = srv.get_v2x_frame()
        return [l for l in f.links if l.msg_type == "DENM"] if f else []
    # the stub world has only one V2X vehicle, which is the only possible receiver
    assert _wait(lambda: srv.list_denm() != []), "DENM never showed up in a V2X frame"
    assert srv.list_denm()[0]["cause_code"] == "OBSTACLE_ON_ROAD"
    assert _wait(lambda: any(l.rx_id == actor_id for l in denm_links()))
    assert any(e["kind"] == "denm_injected" for e in srv.events.since(0)[0])

    assert srv.submit(srv.cancel_denm, out["station_id"], out["action_id"]).result(5) is True
    assert srv.submit(srv.cancel_denm, out["station_id"], out["action_id"]).result(5) is False
    assert _wait(lambda: srv.list_denm() == [])


def test_injected_denm_with_duration_ends_by_itself(env):
    rec, make = env
    srv = make()
    _join(srv)
    srv.submit(srv.inject_denm, 0.0, 0.0, 0.0, 2, 0, 100.0, 7, None, 1.0).result(5)
    assert _wait(lambda: srv.list_denm() != [])
    assert _wait(lambda: srv.list_denm() == [], 8), "a DENM with duration_s must end"
    assert any(e["kind"] == "denm_expired" for e in srv.events.since(0)[0])


def test_denm_injection_validates_the_originator(env):
    rec, make = env
    srv = make()
    _join(srv)
    with pytest.raises(ValueError, match="not a registered V2X vehicle"):
        srv.submit(srv.inject_denm, 0, 0, 0, 2, 0, 100.0, 7, 424242).result(5)
    with pytest.raises(ValueError, match="not a registered RSU"):
        srv.submit(srv.inject_denm, 0, 0, 0, 2, 0, 100.0, 7, -5).result(5)
    with pytest.raises(RuntimeError, match="simulation thread"):
        srv.inject_denm(0, 0, 0, 2, 0, 100.0, 7)


def test_map_geometry_is_built_on_the_sim_thread_once_and_cached(env, monkeypatch):
    import src.server.persistent_world as pw
    rec, make = env
    srv = make()
    calls = []
    monkeypatch.setattr(pw, "build_map_geometry", lambda m: calls.append(1) or {"version": "v1", "lanes": []})
    assert srv.cached_map_geometry() is None
    a = srv.submit(srv.get_map_geometry).result(5)
    b = srv.submit(srv.get_map_geometry).result(5)
    assert a is b and srv.cached_map_geometry() is a and len(calls) == 1
    with pytest.raises(RuntimeError, match="simulation thread"):
        srv.get_map_geometry()


# ── hot configuration and RSU management on the (stub) world ─────────────────────────────────────

def test_hot_config_changes_are_applied_on_the_sim_thread_and_logged_with_the_frame(env):
    rec, make = env
    srv = make()
    out = srv.submit(srv.apply_config, {"awareness_range_m": 120.0, "lidar_max_sensors": 2, "lidar_keep_alive_s": 3.0,
                                         "v2x_penetration_rate": 0.5, "observer_public": False}).result(5)
    assert set(out["applied"]) == {"awareness_range_m", "lidar_max_sensors", "lidar_keep_alive_s", "v2x_penetration_rate", "observer_public"}
    assert srv.v2x_network.awareness_range_m == 120.0 and srv.awareness_range_m == 120.0
    assert srv.lidar.limits.max_sensors == 2 and srv.lidar.limits.keep_alive_s == 3.0
    assert srv.config.v2x_penetration_rate == 0.5 and srv.observer_public is False
    ev = [e for e in srv.events.since(0)[0] if e["kind"] == "config_changed"]
    assert {e["data"]["field"] for e in ev} == set(out["applied"]) and all(e["data"]["applied_at_frame"] == out["applied_at_frame"] for e in ev)
    assert srv.get_status()["awareness_range_m"] == 120.0


def test_hot_config_is_all_or_nothing_and_refuses_restart_fields(env):
    rec, make = env
    srv = make()
    before = srv.config
    with pytest.raises(PermissionError, match="fixed_delta_seconds"):
        srv.submit(srv.apply_config, {"awareness_range_m": 100.0, "fixed_delta_seconds": 0.1}).result(5)
    with pytest.raises(ValueError):
        srv.submit(srv.apply_config, {"awareness_range_m": 1.0}).result(5)
    with pytest.raises(ValueError, match="unknown"):
        srv.submit(srv.apply_config, {"bogus": 1}).result(5)
    assert srv.config == before and srv.v2x_network.awareness_range_m == before.awareness_range_m
    assert not [e for e in srv.events.since(0)[0] if e["kind"] == "config_changed"]
    with pytest.raises(RuntimeError, match="simulation thread"):
        srv.apply_config({"awareness_range_m": 100.0})


def test_lowering_the_lidar_budget_does_not_kill_running_sensors_but_refuses_new_ones(env):
    from src.lidar.config import LidarConfig
    from src.lidar.sensor import ADMIN, LidarBudgetExceeded
    rec, make = env
    srv = make()
    _, actor_id = _join(srv)
    srv.submit(srv.lidar_request, ADMIN, actor_id, LidarConfig(points_per_second=500_000), pinned=True).result(5)
    srv.submit(srv.apply_config, {"lidar_max_points_per_second": 100_000}).result(5)
    time.sleep(0.3)
    assert srv.lidar.list_sensors()[0]["state"] in ("active", "warming_up")
    with pytest.raises(LidarBudgetExceeded):
        srv.submit(srv.lidar_request, ADMIN, actor_id, LidarConfig(points_per_second=50_000)).result(5)


def test_rsu_can_be_disabled_changed_and_enabled_again(env):
    rec, make = env
    srv = make()
    _join(srv)
    # the stub world has no traffic lights, so register one RSU by hand
    from src.v2x.rsu import RoadSideUnit, RSUConfig
    rsu = RoadSideUnit(RSUConfig(intersection_id=5, position=(10.0, 20.0, 3.0)))
    srv.submit(srv.v2x_network.register_rsu, rsu).result(5)
    assert _wait(lambda: any(r["station_id"] == -5 for r in srv.get_world_state().data["rsus"]))
    r = srv.submit(srv.set_rsu, -5, False, 20.0).result(5)
    assert r["enabled"] is False and r["tx_power_dbm"] == 20.0 and set(r["changes"]) == {"enabled", "tx_power_dbm"}
    assert 5 not in srv.v2x_network._rsus
    assert _wait(lambda: next(x for x in srv.get_world_state().data["rsus"] if x["station_id"] == -5)["enabled"] is False)
    assert _wait(lambda: -5 not in srv.get_v2x_frame().stations)
    r = srv.submit(srv.set_rsu, -5, True).result(5)
    assert r["enabled"] is True and 5 in srv.v2x_network._rsus and r["changes"] == {"enabled": {"old": False, "new": True}}
    assert _wait(lambda: -5 in srv.get_v2x_frame().stations)
    assert srv.submit(srv.set_rsu, -5, True).result(5)["changes"] == {}, "no change, no event"
    with pytest.raises(LookupError):
        srv.submit(srv.set_rsu, -77, True).result(5)
    with pytest.raises(LookupError):
        srv.submit(srv.set_rsu, 5, True).result(5)
    kinds = [e["data"]["field"] for e in srv.events.since(0)[0] if e["kind"] == "rsu_changed"]
    assert kinds.count("enabled") == 2 and kinds.count("tx_power_dbm") == 1


# ── recorded runs on the (stub) world ─────────────────────────────────────────────────────────────

def test_a_run_records_v2x_updates_events_and_a_manifest_with_hashes(env, tmp_path):
    import json
    rec, make = env
    srv = make(runs_dir=str(tmp_path / "runs"))
    _join(srv)
    info = srv.submit(srv.start_run, "unit run", ("links", "stations", "messages")).result(5)
    run_dir = tmp_path / "runs" / info["run_id"]
    assert info["run_id"].endswith("_unit-run") and (run_dir / "manifest.json").exists()
    assert json.loads((run_dir / "manifest.json").read_text())["status"] == "running"
    assert _wait(lambda: srv.get_run_status() and (srv.get_run_status().data["recording"] or {}).get("run_id") == info["run_id"], 10), "run.status shows the active run"
    assert _wait(lambda: srv.run_status() and srv.run_status()["v2x_updates"] >= 3, 10)
    srv.submit(srv.apply_config, {"awareness_range_m": 111.0}).result(5)
    out = srv.submit(srv.stop_run, "test").result(10)
    assert out["status"] == "finished" and srv.run_status() is None
    m = json.loads((run_dir / "manifest.json").read_text())
    assert m["status"] == "finished" and m["stop_reason"] == "test" and m["frames_recorded"] >= 3 and m["ended_utc"]
    assert m["config"]["seed"] == 42 and m["config"]["runs_dir"].endswith("runs") and m["seeds"]["world_seed"] == 42
    assert m["git"]["sha"] and m["versions"]["python"] and m["map"].endswith("TestTown") and m["start_frame"] > 0
    assert set(m["files"]) == {"events.jsonl", "messages.jsonl", "stations.jsonl", "v2x_links.jsonl"}
    assert m["files"]["stations.jsonl"]["lines"] == m["frames_recorded"] and len(m["files"]["stations.jsonl"]["sha256"]) == 64
    assert [c["field"] for c in m["config_changes"]] == ["awareness_range_m"] and m["config_changes"][0]["new"] == 111.0
    kinds = [json.loads(l)["kind"] for l in (run_dir / "events.jsonl").read_text().splitlines()]
    assert kinds[0] == "run_started" and "config_changed" in kinds and kinds[-1] == "run_stopped"
    with pytest.raises(LookupError):
        srv.submit(srv.stop_run).result(5)


def test_only_one_run_at_a_time_and_a_run_ends_with_the_world(env, tmp_path):
    import json
    from src.recording.recorder import RunActiveError
    rec, make = env
    srv = make(runs_dir=str(tmp_path / "runs"))
    info = srv.submit(srv.start_run, "a").result(5)
    with pytest.raises(RunActiveError):
        srv.submit(srv.start_run, "b").result(5)
    srv.stop()
    m = json.loads((tmp_path / "runs" / info["run_id"] / "manifest.json").read_text())
    assert m["status"] == "finished" and m["stop_reason"] == "world stopped"


def test_a_failing_writer_stops_the_run_and_says_why(env, tmp_path, monkeypatch):
    import json
    from src.recording import recorder as recmod
    rec, make = env
    srv = make(runs_dir=str(tmp_path / "runs"))
    monkeypatch.setattr(recmod.RunRecorder, "_v2x", lambda self, frame: (_ for _ in ()).throw(OSError("disk full")))
    info = srv.submit(srv.start_run, "doomed", ("links",)).result(5)
    assert _wait(lambda: srv.run_status() is None, 10), "the run must stop by itself"
    m = json.loads((tmp_path / "runs" / info["run_id"] / "manifest.json").read_text())
    assert m["status"] == "failed" and "disk full" in m["error"] and m["stop_reason"] == "writer error"
    assert any(e["kind"] == "run_stopped" for e in srv.events.since(0)[0])


def test_scripted_commands_run_before_the_tick_with_their_index_and_the_loop_can_start_paused(env):
    rec, make = env
    from src.server.world_config import WorldConfig
    import src.server.persistent_world as pw
    srv = pw.PersistentCARLAServer(WorldConfig(host="stub", realtime=False))
    srv.start(paused=True)
    try:
        seen = []
        srv.at_tick(0, lambda: seen.append(("t0", srv.tick_index, rec.frame)))
        srv.at_tick(3, lambda: seen.append(("t3", srv.tick_index, rec.frame)))
        f0 = rec.frame
        time.sleep(0.3)
        assert rec.frame == f0 and srv.tick_index == 0, "paused: no tick"
        assert srv.submit(lambda: "ran while paused").result(5) == "ran while paused"
        srv.resume()
        assert _wait(lambda: srv.tick_index >= 5, 10)
        assert seen == [("t0", 0, f0), ("t3", 3, f0 + 3)], seen
    finally:
        srv.stop()


def test_realtime_false_ticks_much_faster_than_the_step(env):
    rec, make = env
    make(realtime=False)
    f0 = rec.frame
    time.sleep(1.0)
    assert rec.frame - f0 > 100, "50 ms steps would give 20 frames per second"
