"""LidarManager: lifecycle, budget, ownership, frame synchronisation, auto-release (all with a fake backend)."""

import pytest

from src.lidar.config import LidarConfig
from src.lidar.sensor import (ADMIN, LidarBudgetExceeded, LidarError, LidarForbidden, LidarLimits, LidarManager,
                              LidarNotFound, Principal, SensorState)
from tests.unit.lidar_fake import FakeBackend

DT = 0.05
VEH = Principal("vehicle", session_id="s1", actor_id=10, label="car-1")
OBS = Principal("observer")


class Clock:
    def __init__(self):
        self.t = 100.0

    def __call__(self):
        return self.t


@pytest.fixture
def env():
    backend, clock = FakeBackend(), Clock()
    mgr = LidarManager(backend, LidarLimits(max_sensors=2, max_points_per_second=1_000_000, keep_alive_s=10.0), DT,
                       clock=clock, frame_timeout_s=0.0)
    return mgr, backend, clock


def tick(mgr, backend, frame, emit=True):
    """One simulation step: collect (state machine) then deliver this frame's data the way CARLA's thread would
    (before world.tick returns)."""
    if emit:
        backend.emit_all(frame)
    return mgr.collect(frame, frame * DT)


def advance(mgr, backend, start, n):
    for f in range(start, start + n):
        tick(mgr, backend, f)
    return start + n


# ── lifecycle ───────────────────────────────────────────────────────────────────────────────────────

def test_requested_then_spawned_at_next_tick_then_active(env):
    mgr, backend, _ = env
    sid = mgr.request(VEH, 10)
    assert mgr.state_of(sid) == SensorState.REQUESTED and backend.handles == []
    mgr.attach_viewer(sid, "v")
    # spawn happens inside collect(); measurement of the *same* frame cannot exist yet, so warming_up
    frames = mgr.collect(1, 0.05)
    assert mgr.state_of(sid) == SensorState.WARMING_UP and frames == []
    frames = tick(mgr, backend, 2)
    assert mgr.state_of(sid) == SensorState.ACTIVE
    assert [f.frame for f in frames] == [2] and frames[0].ego_actor_id == 10
    transitions = [(e["from"], e["to"]) for e in mgr.drain_events()]
    assert transitions == [("requested", "requested"), ("requested", "spawning"), ("spawning", "warming_up"),
                           ("warming_up", "active")]


def test_events_carry_the_frame(env):
    mgr, backend, _ = env
    mgr.request(VEH, 10)
    mgr.collect(7, 0.35)
    assert {e["frame"] for e in mgr.drain_events() if e["to"] == "spawning"} == {7}


def test_frame_data_is_the_ticked_frame_and_read_only(env):
    mgr, backend, _ = env
    sid = mgr.request(VEH, 10)
    mgr.attach_viewer(sid, "v")
    mgr.collect(1, 0.05)
    tick(mgr, backend, 5)
    f = mgr.get_frame(sid)
    assert f.frame == 5 and f.points["x"][0] == 5.0 and f.num_points == 5
    assert f.location == (1.0, 2.0, 3.0) and f.rotation_pyr == (0.0, 90.0, 0.0)
    with pytest.raises(ValueError):
        f.points["x"][0] = 1.0


def test_spawn_failure_goes_to_error_and_frees_budget(env):
    mgr, backend, _ = env
    backend.fail_next = RuntimeError("blueprint missing")
    sid = mgr.request(VEH, 10)
    mgr.collect(1, 0.05)
    assert mgr.state_of(sid) == SensorState.ERROR
    assert "blueprint missing" in mgr.list_sensors()[0]["error"]
    assert mgr.budget()["used_pps"] == 0 and mgr.budget()["sensors"] == 0


def test_no_data_for_too_long_is_an_error_and_the_actor_is_destroyed(env):
    mgr, backend, _ = env
    sid = mgr.request(VEH, 10)
    mgr.attach_viewer(sid, "v")
    mgr.collect(1, 0.05)
    h = backend.handles[0]
    for f in range(2, 2 + LidarManager.MAX_CONSECUTIVE_MISSES):
        tick(mgr, backend, f, emit=False)
    assert mgr.state_of(sid) == SensorState.ERROR
    assert h.stopped and h.destroyed


def test_recovers_miss_counter_when_data_arrives(env):
    mgr, backend, _ = env
    sid = mgr.request(VEH, 10)
    mgr.attach_viewer(sid, "v")
    mgr.collect(1, 0.05)
    for f in range(2, 12):
        tick(mgr, backend, f, emit=False)
    tick(mgr, backend, 12)
    for f in range(13, 13 + 30):
        tick(mgr, backend, f, emit=False)
    assert mgr.state_of(sid) == SensorState.ACTIVE


def test_malformed_measurement_is_an_error(env):
    mgr, backend, _ = env
    sid = mgr.request(VEH, 10)
    mgr.attach_viewer(sid, "v")
    mgr.collect(1, 0.05)
    h = backend.handles[0]
    from tests.unit.lidar_fake import make_measurement
    m = make_measurement(2, h.config)
    m.raw_data = m.raw_data + b"\x00"
    h.callback(m)
    mgr.collect(2, 0.1)
    assert mgr.state_of(sid) == SensorState.ERROR


# ── frame synchronisation ─────────────────────────────────────────────────────────────────────────

def test_older_frames_are_discarded_and_exact_frame_is_used(env):
    mgr, backend, _ = env
    sid = mgr.request(VEH, 10)
    mgr.attach_viewer(sid, "v")
    mgr.collect(1, 0.05)
    h = backend.handles[0]
    h.emit(2)
    h.emit(3)          # frames 2 and 3 queued, but we tick frame 4
    h.emit(4)
    frames = mgr.collect(4, 0.2)
    assert [f.frame for f in frames] == [4]


def test_late_measurement_only_is_a_miss_not_a_mixed_frame(env):
    mgr, backend, _ = env
    sid = mgr.request(VEH, 10)
    mgr.attach_viewer(sid, "v")
    mgr.collect(1, 0.05)
    backend.handles[0].emit(2)         # only frame 2 available, we tick frame 3
    assert mgr.collect(3, 0.15) == []
    assert mgr.get_frame(sid) is None


def test_measurement_from_the_future_is_never_used(env):
    mgr, backend, _ = env
    sid = mgr.request(VEH, 10)
    mgr.attach_viewer(sid, "v")
    mgr.collect(1, 0.05)
    backend.handles[0].emit(9)
    assert mgr.collect(3, 0.15) == []


# ── budget ────────────────────────────────────────────────────────────────────────────────────────

def test_budget_points_per_second_409_detail(env):
    mgr, _, _ = env
    mgr.request(ADMIN, 1, LidarConfig(points_per_second=700_000))
    with pytest.raises(LidarBudgetExceeded) as ei:
        mgr.request(ADMIN, 2, LidarConfig(points_per_second=400_000))
    d = ei.value.detail
    assert d["error"] == "lidar_budget_exceeded" and d["reason"] == "max_points_per_second"
    assert d["requested_pps"] == 400_000 and d["available_pps"] == 300_000 and d["used_pps"] == 700_000
    assert len(mgr.list_sensors()) == 1          # nothing half-created


def test_budget_max_sensors(env):
    mgr, _, _ = env
    mgr.request(ADMIN, 1, LidarConfig(points_per_second=10_000))
    mgr.request(ADMIN, 2, LidarConfig(points_per_second=10_000))
    with pytest.raises(LidarBudgetExceeded) as ei:
        mgr.request(ADMIN, 3, LidarConfig(points_per_second=10_000))
    assert ei.value.detail["reason"] == "max_sensors" and ei.value.detail["max_sensors"] == 2


def test_release_frees_budget_after_destruction(env):
    mgr, backend, _ = env
    a = mgr.request(ADMIN, 1, LidarConfig(points_per_second=900_000), pinned=True)
    mgr.collect(1, 0.05)
    mgr.release(ADMIN, a)
    assert mgr.state_of(a) == SensorState.STOPPING
    with pytest.raises(LidarBudgetExceeded):      # still holds budget until it is destroyed
        mgr.request(ADMIN, 2, LidarConfig(points_per_second=500_000))
    mgr.collect(2, 0.1)
    assert mgr.state_of(a) == SensorState.DESTROYED and backend.live == []
    mgr.request(ADMIN, 2, LidarConfig(points_per_second=500_000))


def test_reconfigure_checks_budget_excluding_itself(env):
    mgr, _, _ = env
    a = mgr.request(ADMIN, 1, LidarConfig(points_per_second=900_000))
    mgr.reconfigure(ADMIN, a, LidarConfig(points_per_second=950_000))       # own old budget does not count
    with pytest.raises(LidarBudgetExceeded):
        mgr.reconfigure(ADMIN, a, LidarConfig(points_per_second=1_100_000 // 1 + 1))


def test_pending_reconfiguration_holds_the_larger_budget(env):
    mgr, backend, _ = env
    a = mgr.request(ADMIN, 1, LidarConfig(points_per_second=300_000), pinned=True)
    mgr.collect(1, 0.05)
    mgr.reconfigure(ADMIN, a, LidarConfig(points_per_second=800_000))
    assert mgr.budget()["used_pps"] == 800_000
    with pytest.raises(LidarBudgetExceeded):
        mgr.request(ADMIN, 2, LidarConfig(points_per_second=300_000))


# ── ownership ─────────────────────────────────────────────────────────────────────────────────────

def test_observer_cannot_create(env):
    mgr, _, _ = env
    with pytest.raises(LidarForbidden):
        mgr.request(OBS, 10)


def test_vehicle_only_own_actor(env):
    mgr, _, _ = env
    with pytest.raises(LidarForbidden):
        mgr.request(VEH, 11)
    with pytest.raises(LidarForbidden):
        mgr.request(Principal("vehicle", session_id="s1"), 10)       # no actor bound
    mgr.request(VEH, 10)


def test_admin_any_actor_and_pin_record_are_admin_only(env):
    mgr, _, _ = env
    mgr.request(ADMIN, 99, pinned=True, record_raw=True)
    with pytest.raises(LidarForbidden):
        mgr.request(VEH, 10, pinned=True)
    with pytest.raises(LidarForbidden):
        mgr.request(VEH, 10, record_raw=True)


def test_only_owner_or_admin_may_change(env):
    mgr, _, _ = env
    sid = mgr.request(VEH, 10)
    other = Principal("vehicle", session_id="s2", actor_id=11)
    for bad in (other, OBS):
        with pytest.raises(LidarForbidden):
            mgr.release(bad, sid)
        with pytest.raises(LidarForbidden):
            mgr.reconfigure(bad, sid, LidarConfig())
    mgr.reconfigure(VEH, sid, LidarConfig(channels=64))
    mgr.release(ADMIN, sid)


def test_unknown_or_destroyed_sensor_is_not_found(env):
    mgr, _, _ = env
    with pytest.raises(LidarNotFound):
        mgr.release(ADMIN, 42)
    sid = mgr.request(ADMIN, 1)
    mgr.release(ADMIN, sid)
    mgr.collect(1, 0.05)
    with pytest.raises(LidarNotFound):
        mgr.release(ADMIN, sid)


def test_release_owned_by_and_actor_removed(env):
    mgr, backend, _ = env
    a = mgr.request(VEH, 10)
    b = mgr.request(ADMIN, 77, pinned=True)
    mgr.collect(1, 0.05)
    assert mgr.release_owned_by("s1") == 1
    assert mgr.release_owned_by("s1") == 0         # idempotent
    assert mgr.on_actor_removed(77) == 1
    mgr.collect(2, 0.1)
    assert mgr.state_of(a) == mgr.state_of(b) == SensorState.DESTROYED
    assert backend.live == []


# ── reconfigure ───────────────────────────────────────────────────────────────────────────────────

def test_reconfigure_destroys_and_respawns_with_new_config(env):
    mgr, backend, _ = env
    sid = mgr.request(VEH, 10, LidarConfig(channels=32))
    mgr.attach_viewer(sid, "v")
    mgr.collect(1, 0.05)
    tick(mgr, backend, 2)
    old = backend.handles[0]
    mgr.reconfigure(VEH, sid, LidarConfig(channels=64))
    assert mgr.state_of(sid) == SensorState.RECONFIGURING
    assert mgr.collect(3, 0.15) == []            # respawned this boundary, no data for frame 3 yet
    assert old.destroyed and len(backend.handles) == 2 and backend.handles[1].config.channels == 64
    assert mgr.state_of(sid) == SensorState.WARMING_UP
    frames = tick(mgr, backend, 4)
    assert mgr.state_of(sid) == SensorState.ACTIVE and frames[0].config.channels == 64


def test_cannot_reconfigure_a_stopping_sensor(env):
    mgr, _, _ = env
    sid = mgr.request(ADMIN, 1)
    mgr.release(ADMIN, sid)
    with pytest.raises(LidarError):
        mgr.reconfigure(ADMIN, sid, LidarConfig())


def test_reconfigure_requested_sensor_just_replaces_config(env):
    mgr, backend, _ = env
    sid = mgr.request(ADMIN, 1, LidarConfig(channels=16))
    mgr.reconfigure(ADMIN, sid, LidarConfig(channels=64))
    mgr.collect(1, 0.05)
    assert backend.handles[0].config.channels == 64


# ── auto-release ──────────────────────────────────────────────────────────────────────────────────

def test_auto_release_after_keep_alive_without_viewers(env):
    mgr, backend, clock = env
    sid = mgr.request(VEH, 10)
    mgr.attach_viewer(sid, "v")
    f = advance(mgr, backend, 1, 3)
    clock.t += 1000                     # a viewer is attached: never released however long
    tick(mgr, backend, f)
    assert mgr.state_of(sid) == SensorState.ACTIVE
    mgr.detach_viewer(sid, "v")
    tick(mgr, backend, f + 1)           # idle clock starts at the first tick without viewers... refreshed above
    clock.t += 9.9
    tick(mgr, backend, f + 2)
    assert mgr.state_of(sid) == SensorState.ACTIVE
    clock.t += 0.2
    tick(mgr, backend, f + 3)
    assert mgr.state_of(sid) == SensorState.STOPPING
    tick(mgr, backend, f + 4)
    assert mgr.state_of(sid) == SensorState.DESTROYED and backend.live == []
    assert any("auto-release" in e["reason"] for e in mgr.drain_events())


def test_viewer_returning_in_time_keeps_the_sensor(env):
    mgr, backend, clock = env
    sid = mgr.request(VEH, 10)
    mgr.collect(1, 0.05)
    clock.t += 8
    tick(mgr, backend, 2)
    mgr.attach_viewer(sid, "v")
    clock.t += 8
    tick(mgr, backend, 3)
    assert mgr.state_of(sid) == SensorState.ACTIVE


def test_two_viewers_refcount(env):
    mgr, backend, clock = env
    sid = mgr.request(VEH, 10)
    mgr.attach_viewer(sid, "a")
    mgr.attach_viewer(sid, "b")
    mgr.detach_viewer(sid, "a")
    assert mgr.viewer_count(sid) == 1
    mgr.collect(1, 0.05)
    clock.t += 100
    tick(mgr, backend, 2)
    assert mgr.state_of(sid) == SensorState.ACTIVE


def test_pinned_and_record_raw_are_never_auto_released(env):
    mgr, backend, clock = env
    a = mgr.request(ADMIN, 1, LidarConfig(points_per_second=10_000), pinned=True)
    b = mgr.request(ADMIN, 2, LidarConfig(points_per_second=10_000), record_raw=True)
    mgr.collect(1, 0.05)
    clock.t += 1000
    tick(mgr, backend, 2)
    tick(mgr, backend, 3)
    assert mgr.state_of(a) == mgr.state_of(b) == SensorState.ACTIVE


# ── shutdown / views ──────────────────────────────────────────────────────────────────────────────

def test_shutdown_destroys_everything(env):
    mgr, backend, _ = env
    mgr.request(ADMIN, 1, pinned=True)
    mgr.request(ADMIN, 2, LidarConfig(points_per_second=10_000), pinned=True)
    mgr.collect(1, 0.05)
    mgr.shutdown()
    assert backend.live == [] and all(h.stopped for h in backend.handles)
    assert mgr.list_sensors() == [] and mgr.budget()["used_pps"] == 0


def test_list_sensors_snapshot_content(env):
    mgr, _, _ = env
    sid = mgr.request(VEH, 10, LidarConfig(channels=16, points_per_second=100_000))
    mgr.attach_viewer(sid, "v")
    mgr.collect(1, 0.05)
    (info,) = mgr.list_sensors()
    assert info["sensor_id"] == sid and info["actor_id"] == 10 and info["owner"] == "car-1"
    assert info["state"] == "warming_up" and info["viewers"] == 1 and info["config"]["channels"] == 16
    assert info["points_per_frame"] == pytest.approx(5000)
    assert mgr.budget() == {"used_pps": 100_000, "max_pps": 1_000_000, "sensors": 1, "max_sensors": 2}


def test_a_sensor_spawned_at_this_boundary_is_not_waited_for():
    """It cannot have a measurement of the frame that was just ticked; blocking for frame_timeout would stall
    the simulation thread (found with the CARLA stub)."""
    import time
    mgr = LidarManager(FakeBackend(), LidarLimits(), DT, frame_timeout_s=5.0)
    mgr.request(ADMIN, 1, pinned=True)
    t0 = time.monotonic()
    mgr.collect(1, 0.05)
    assert time.monotonic() - t0 < 1.0
    assert mgr.list_sensors()[0]["state"] == "warming_up" and mgr.budget()  # and no miss was counted
    assert mgr._sensors[1].missed == 0


def test_info_version_bumps_on_change_but_not_every_frame(env):
    mgr, backend, _ = env
    v0 = mgr.info_version
    sid = mgr.request(VEH, 10)
    mgr.attach_viewer(sid, "v")
    assert mgr.info_version > v0
    mgr.collect(1, 0.05)
    tick(mgr, backend, 2)                    # -> active
    v = mgr.info_version
    for f in range(3, 12):                   # steady state: only last_frame moves
        tick(mgr, backend, f)
    assert mgr.info_version == v
    mgr.detach_viewer(sid, "v")
    mgr.attach_viewer(sid, "a")
    mgr.attach_viewer(sid, "b")
    tick(mgr, backend, 12)
    assert mgr.info_version == v + 1         # viewers 1 -> 2
