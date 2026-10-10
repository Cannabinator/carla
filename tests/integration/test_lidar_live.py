"""LiDAR against a real CARLA server (marker: carla; needs the carla module, no web server).

Settles what the offline tests can only assume (REFACTOR_HANDOFF 5.5 "verify" items):
  * the sensor frame convention: points transformed with ``src.lidar.transforms`` land on the vehicle that
    produced them (no y flip, correct handedness)                                         (L1)
  * ``object_idx`` of the semantic sensor is the hit actor id
  * every collected frame is the frame that was ticked                                    (L3)
  * reconfiguring respawns the sensor with the new attributes
  * shutdown leaves no sensor actors behind
"""

import numpy as np
import pytest

from src.lidar.carla_backend import CarlaSensorBackend
from src.lidar.config import LidarConfig
from src.lidar.sensor import ADMIN, LidarLimits, LidarManager, SensorState
from src.lidar.transforms import to_sensor, to_world

pytestmark = pytest.mark.carla

DT = 0.05


@pytest.fixture
def sync_world(carla_client):
    carla = pytest.importorskip("carla")
    world = carla_client.get_world()
    original = world.get_settings()
    s = world.get_settings()
    s.synchronous_mode = True
    s.fixed_delta_seconds = DT
    world.apply_settings(s)
    spawned = []
    try:
        yield carla, world, spawned
    finally:
        for a in spawned:
            try:
                a.destroy()
            except RuntimeError:
                pass
        world.apply_settings(original)


def _spawn_pair(carla, world, spawned, gap_m=10.0):
    bp = world.get_blueprint_library().find("vehicle.tesla.model3")
    sp = world.get_map().get_spawn_points()[0]
    fwd = sp.get_forward_vector()
    a = world.try_spawn_actor(bp, sp)
    assert a is not None, "could not spawn the ego vehicle"
    spawned.append(a)
    ahead = carla.Transform(carla.Location(sp.location.x + fwd.x * gap_m, sp.location.y + fwd.y * gap_m,
                                           sp.location.z), sp.rotation)
    b = world.try_spawn_actor(bp, ahead)
    assert b is not None, "could not spawn the target vehicle"
    spawned.append(b)
    for _ in range(5):        # let them settle on the ground
        world.tick()
    return a, b


def _collect(mgr, world, n):
    out = []
    for _ in range(n):
        frame = world.tick()
        got = mgr.collect(frame, world.get_snapshot().timestamp.elapsed_seconds)
        for f in got:
            assert f.frame == frame, "a collected measurement must belong to the frame that was ticked (L3)"
        out.extend(got)
    return out


def test_points_land_on_the_vehicle_that_produced_them(sync_world):
    carla, world, spawned = sync_world
    ego, target = _spawn_pair(carla, world, spawned)
    mgr = LidarManager(CarlaSensorBackend(carla, world), LidarLimits(), DT)
    try:
        sid = mgr.request(ADMIN, ego.id, LidarConfig(), pinned=True)
        frames = _collect(mgr, world, 6)
        assert mgr.state_of(sid) == SensorState.ACTIVE, mgr.list_sensors()
        frame = frames[-1]
        hit = frame.points["object_idx"] == target.id
        assert hit.sum() >= 20, f"only {int(hit.sum())} points on the target vehicle (object_idx is the hit actor id?)"
        xyz = np.stack([frame.points[k][hit] for k in "xyz"], axis=1).astype(np.float64)
        world_pts = to_world(xyz, frame.location, frame.rotation_pyr)
        t = target.get_transform()
        local = to_sensor(world_pts, (t.location.x, t.location.y, t.location.z),
                          (t.rotation.pitch, t.rotation.yaw, t.rotation.roll))
        ext, off = target.bounding_box.extent, target.bounding_box.location
        inside = (np.abs(local[:, 0] - off.x) <= ext.x + 0.35) & (np.abs(local[:, 1] - off.y) <= ext.y + 0.35) \
            & (np.abs(local[:, 2] - off.z) <= ext.z + 0.35)
        assert inside.mean() > 0.95, (
            f"only {inside.mean():.0%} of the target's points are inside its bounding box after to_world(): "
            f"the frame convention differs from the one assumed (y flip?)")
    finally:
        mgr.shutdown()


def test_reconfigure_respawns_with_new_attributes_and_nothing_leaks(sync_world):
    carla, world, spawned = sync_world
    ego, _ = _spawn_pair(carla, world, spawned)
    mgr = LidarManager(CarlaSensorBackend(carla, world), LidarLimits(), DT)
    try:
        sid = mgr.request(ADMIN, ego.id, LidarConfig(channels=32), pinned=True)
        _collect(mgr, world, 4)
        assert mgr.state_of(sid) == SensorState.ACTIVE
        assert len(world.get_actors().filter("sensor.lidar.*")) == 1
        mgr.reconfigure(ADMIN, sid, LidarConfig(channels=64))
        frames = _collect(mgr, world, 4)
        assert mgr.state_of(sid) == SensorState.ACTIVE and frames and frames[-1].config.channels == 64
        sensors = list(world.get_actors().filter("sensor.lidar.*"))
        assert len(sensors) == 1 and sensors[0].attributes["channels"] == "64"
        mgr.release(ADMIN, sid)
        _collect(mgr, world, 2)
        assert mgr.state_of(sid) == SensorState.DESTROYED
        assert len(world.get_actors().filter("sensor.lidar.*")) == 0, "a sensor actor leaked"
    finally:
        mgr.shutdown()
    assert len(world.get_actors().filter("sensor.lidar.*")) == 0
