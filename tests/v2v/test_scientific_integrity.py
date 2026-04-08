#!/usr/bin/env python3
"""Scientific integrity tests for V2V timing and threat calculations."""

import unittest
import sys
import math
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent.parent))

from src.v2v import BSMCore, calculate_threat_level, create_bsm_from_carla, V2VNetworkEnhanced


class MockVector:
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x = x
        self.y = y
        self.z = z


class MockRotation:
    def __init__(self, pitch=0.0, yaw=0.0, roll=0.0):
        self.pitch = pitch
        self.yaw = yaw
        self.roll = roll


class MockTransform:
    def __init__(self, x=0.0, y=0.0, z=0.0, yaw=0.0):
        self.location = MockVector(x, y, z)
        self.rotation = MockRotation(0.0, yaw, 0.0)


class MockControl:
    def __init__(self):
        self.throttle = 0.0
        self.brake = 0.0
        self.steer = 0.0
        self.reverse = False


class MockWheelPhysicsControl:
    def __init__(self):
        self.max_steer_angle = 70.0


class MockPhysicsControl:
    def __init__(self):
        self.wheels = [MockWheelPhysicsControl() for _ in range(4)]


class MockVehicle:
    def __init__(self, vid, x=0.0, y=0.0, speed_x=0.0):
        self.id = vid
        self._transform = MockTransform(x, y, 0.0)
        self._velocity = MockVector(speed_x, 0.0, 0.0)
        self._angular_velocity = MockVector(0.0, 0.0, 0.0)
        self._control = MockControl()
        self._physics_control = MockPhysicsControl()
        self.bounding_box = type('obj', (object,), {'extent': MockVector(2.25, 0.9, 0.75)})()

    def get_transform(self):
        return self._transform

    def get_velocity(self):
        return self._velocity

    def get_angular_velocity(self):
        return self._angular_velocity

    def get_control(self):
        return self._control

    def get_physics_control(self):
        return self._physics_control


class MockActorSnapshot:
    def __init__(self, vehicle, transform=None):
        self.vehicle = vehicle
        self._transform = transform or vehicle.get_transform()

    def get_transform(self):
        return self._transform

    def get_velocity(self):
        return self.vehicle.get_velocity()

    def get_angular_velocity(self):
        return self.vehicle.get_angular_velocity()


class MockSnapshotTimestamp:
    def __init__(self, elapsed_seconds):
        self.elapsed_seconds = elapsed_seconds


class MockWorldSnapshot:
    def __init__(self, vehicles, elapsed_seconds, transform_overrides=None):
        self.vehicles = vehicles
        self.timestamp = MockSnapshotTimestamp(elapsed_seconds)
        self.transform_overrides = transform_overrides or {}

    def find(self, vehicle_id):
        vehicle = next((v for v in self.vehicles if v.id == vehicle_id), None)
        if vehicle is None:
            return None
        transform = self.transform_overrides.get(vehicle_id)
        return MockActorSnapshot(vehicle, transform=transform)


class MockWorld:
    def __init__(self):
        self._snapshot = None

    def set_snapshot(self, snapshot):
        self._snapshot = snapshot

    def get_snapshot(self):
        return self._snapshot


class TestScientificIntegrity(unittest.TestCase):
    def test_ttc_receding_vehicle_is_not_threat(self):
        ego = BSMCore(timestamp=0.0, msg_count=0, vehicle_id=1, latitude=0.0, longitude=0.0, speed=10.0, heading=0.0)
        other = BSMCore(timestamp=0.0, msg_count=0, vehicle_id=2, latitude=20.0, longitude=0.0, speed=20.0, heading=0.0)

        level, ttc, _ = calculate_threat_level(ego, other)
        self.assertEqual(level, 0)
        self.assertEqual(ttc, float('inf'))

    def test_ttc_converging_vehicle_is_finite(self):
        ego = BSMCore(timestamp=0.0, msg_count=0, vehicle_id=1, latitude=0.0, longitude=0.0, speed=20.0, heading=0.0)
        other = BSMCore(timestamp=0.0, msg_count=0, vehicle_id=2, latitude=100.0, longitude=0.0, speed=20.0, heading=180.0)

        level, ttc, distance = calculate_threat_level(ego, other)
        self.assertGreater(distance, 0.0)
        self.assertTrue(0.0 < ttc < float('inf'))
        self.assertGreaterEqual(level, 1)

    def test_threat_level_thresholds(self):
        """Threat classification should follow TTC threshold bands."""
        ego = BSMCore(timestamp=0.0, msg_count=0, vehicle_id=1, latitude=0.0, longitude=0.0, speed=0.0, heading=0.0)

        # TTC > 10s with distance <= 100m -> level 1
        other_l1 = BSMCore(timestamp=0.0, msg_count=0, vehicle_id=2, latitude=95.0, longitude=0.0, speed=7.0, heading=180.0)
        l1, ttc1, _ = calculate_threat_level(ego, other_l1)
        self.assertGreater(ttc1, 10.0)
        self.assertEqual(l1, 1)

        # TTC = 7s -> level 2
        other_l2 = BSMCore(timestamp=0.0, msg_count=0, vehicle_id=3, latitude=70.0, longitude=0.0, speed=10.0, heading=180.0)
        l2, ttc2, _ = calculate_threat_level(ego, other_l2)
        self.assertTrue(5.0 < ttc2 <= 10.0)
        self.assertEqual(l2, 2)

        # TTC = 3s -> level 3
        other_l3 = BSMCore(timestamp=0.0, msg_count=0, vehicle_id=4, latitude=30.0, longitude=0.0, speed=10.0, heading=180.0)
        l3, ttc3, _ = calculate_threat_level(ego, other_l3)
        self.assertTrue(2.0 < ttc3 <= 5.0)
        self.assertEqual(l3, 3)

        # TTC = 1s -> level 4
        other_l4 = BSMCore(timestamp=0.0, msg_count=0, vehicle_id=5, latitude=10.0, longitude=0.0, speed=10.0, heading=180.0)
        l4, ttc4, _ = calculate_threat_level(ego, other_l4)
        self.assertTrue(ttc4 <= 2.0)
        self.assertEqual(l4, 4)

    def test_zero_distance_returns_imminent_collision(self):
        """Zero-distance pairs should be classified as imminent, not crash the model."""
        ego = BSMCore(timestamp=0.0, msg_count=0, vehicle_id=1, latitude=10.0, longitude=10.0, speed=0.0, heading=0.0)
        other = BSMCore(timestamp=0.0, msg_count=0, vehicle_id=2, latitude=10.0, longitude=10.0, speed=0.0, heading=0.0)

        level, ttc, distance = calculate_threat_level(ego, other)
        self.assertEqual(distance, 0.0)
        self.assertEqual(level, 4)
        self.assertTrue(math.isfinite(ttc))

    def test_bsm_fallback_when_actor_missing_from_snapshot(self):
        """If snapshot lookup fails, BSM generation should safely fall back to actor getters."""
        vehicle = MockVehicle(1, x=12.0, y=6.0, speed_x=8.0)
        empty_snapshot = MockWorldSnapshot([], elapsed_seconds=10.0)

        bsm = create_bsm_from_carla(vehicle, vehicle_id=1, msg_count=3, snapshot=empty_snapshot, prev_velocity=6.0, delta_time=0.5)
        self.assertEqual(bsm.vehicle_id, 1)
        self.assertAlmostEqual(bsm.latitude, 12.0)
        self.assertAlmostEqual(bsm.longitude, 6.0)
        self.assertGreaterEqual(bsm.speed, 0.0)

    def test_bsm_uses_snapshot_time_and_transform(self):
        vehicle = MockVehicle(1, x=0.0, y=0.0, speed_x=5.0)
        snapshot_transform = MockTransform(x=123.0, y=45.0, z=1.0, yaw=10.0)
        snapshot = MockWorldSnapshot([vehicle], elapsed_seconds=42.5, transform_overrides={1: snapshot_transform})

        bsm = create_bsm_from_carla(vehicle, vehicle_id=1, msg_count=7, snapshot=snapshot, prev_velocity=0.0, delta_time=0.5)

        self.assertAlmostEqual(bsm.timestamp, 42.5)
        self.assertAlmostEqual(bsm.latitude, 123.0)
        self.assertAlmostEqual(bsm.longitude, 45.0)
        self.assertAlmostEqual(bsm.heading, 10.0)

    def test_neighbor_distance_refreshes_each_update(self):
        world = MockWorld()
        v1 = MockVehicle(1, x=0.0)
        v2 = MockVehicle(2, x=10.0)
        net = V2VNetworkEnhanced(max_range=200.0, update_rate_hz=2.0, world=world)
        net.register(1, v1)
        net.register(2, v2)

        snap1 = MockWorldSnapshot([v1, v2], elapsed_seconds=0.5)
        world.set_snapshot(snap1)
        self.assertTrue(net.update(snapshot=snap1, force=True))
        self.assertAlmostEqual(net.get_distance(1, 2), 10.0)

        v2._transform = MockTransform(x=30.0, y=0.0, z=0.0)
        snap2 = MockWorldSnapshot([v1, v2], elapsed_seconds=1.0)
        world.set_snapshot(snap2)
        self.assertTrue(net.update(snapshot=snap2, force=True))
        self.assertAlmostEqual(net.get_distance(1, 2), 30.0)

    def test_non_forced_update_respects_2hz_sim_time(self):
        world = MockWorld()
        v1 = MockVehicle(1, x=0.0)
        net = V2VNetworkEnhanced(max_range=200.0, update_rate_hz=2.0, world=world)
        net.register(1, v1)

        snap0 = MockWorldSnapshot([v1], elapsed_seconds=0.0)
        self.assertTrue(net.update(snapshot=snap0, force=False))

        snap1 = MockWorldSnapshot([v1], elapsed_seconds=0.2)
        self.assertFalse(net.update(snapshot=snap1, force=False))

        snap2 = MockWorldSnapshot([v1], elapsed_seconds=0.5)
        self.assertTrue(net.update(snapshot=snap2, force=False))

    def test_threat_timestamp_uses_bsm_sim_time(self):
        """Threat entries should use simulation BSM time, not wall-clock time."""
        world = MockWorld()
        v1 = MockVehicle(1, x=0.0, y=0.0, speed_x=10.0)
        v2 = MockVehicle(2, x=20.0, y=0.0, speed_x=-10.0)
        net = V2VNetworkEnhanced(max_range=200.0, update_rate_hz=2.0, world=world)
        net.register(1, v1)
        net.register(2, v2)

        snap = MockWorldSnapshot([v1, v2], elapsed_seconds=12.5)
        self.assertTrue(net.update(snapshot=snap, force=True))

        threats = net.get_threats(1)
        self.assertTrue(len(threats) > 0)
        for threat in threats:
            self.assertAlmostEqual(threat['timestamp'], 12.5, places=6)


if __name__ == '__main__':
    unittest.main()
