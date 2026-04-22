"""
Unit tests for DENM and CAM protocol support.

Covers:
  - DENM dataclass construction and field validation
  - V2VNetworkEnhanced DENM store: publish / get / cancel
  - create_cam_from_bsm() field mapping
  - API endpoints: GET /cam, GET /cam/{id}, GET /denm, GET /denm/{id},
                   POST /denm, DELETE /denm/{station}/{action}

Level 1 (pure unit) — no CARLA runtime required.
"""

import pytest
import time
from fastapi.testclient import TestClient

# Re-use mock infrastructure from the existing basic test suite
from tests.v2v.test_v2v_basic import (
    MockVehicle,
    MockWorld,
    MockWorldSnapshot,
    MockActorSnapshot,
    _transparent_dsrc,
)

from src.v2v.messages import (
    BSMCore,
    DENM,
    DENMCauseCode,
    create_cam_from_bsm,
    BrakingStatus,
    VehicleType,
)
from src.v2v.network_enhanced import V2VNetworkEnhanced
from src.v2v.api import V2VAPI

# ── Helpers ──────────────────────────────────────────────────────────────────


def _make_bsm(vehicle_id: int = 1, x: float = 10.0, y: float = 20.0,
              speed: float = 5.0) -> BSMCore:
    """Construct a minimal BSMCore without a CARLA actor."""
    return BSMCore(
        timestamp=100.0,
        msg_count=1,
        vehicle_id=vehicle_id,
        latitude=x,
        longitude=y,
        speed=speed,
        heading=90.0,
    )


def _make_denm(station_id: int = 1, action_id: int = 42,
               cause: DENMCauseCode = DENMCauseCode.ACCIDENT) -> DENM:
    return DENM(
        station_id=station_id,
        action_id=action_id,
        detection_time=200.0,
        reference_time=200.0,
        event_position=(10.0, 20.0, 0.0),
        cause_code=cause,
        subcause_code=1,
        information_quality=5,
        relevance_distance=100.0,
    )


def _make_network() -> V2VNetworkEnhanced:
    return V2VNetworkEnhanced(
        max_range=200.0,
        update_rate_hz=2.0,
        dsrc_config=_transparent_dsrc(),
    )


def _make_api(network: V2VNetworkEnhanced) -> TestClient:
    api = V2VAPI(network)
    return TestClient(api.app)


# ── DENM dataclass tests ─────────────────────────────────────────────────────


class TestDENMDataclass:
    def test_fields_set_correctly(self):
        d = _make_denm()
        assert d.station_id == 1
        assert d.action_id == 42
        assert d.cause_code == DENMCauseCode.ACCIDENT
        assert d.event_position == (10.0, 20.0, 0.0)
        assert d.termination is None

    def test_cause_code_enum_values(self):
        assert int(DENMCauseCode.ACCIDENT) == 2
        assert int(DENMCauseCode.ROAD_WORKS) == 3
        assert int(DENMCauseCode.COLLISION_RISK) == 97
        assert int(DENMCauseCode.EMERGENCY_VEHICLE_APPROACHING) == 95

    def test_default_termination_is_none(self):
        d = _make_denm()
        assert d.termination is None

    def test_optional_fields_default_to_none(self):
        d = _make_denm()
        assert d.event_speed is None
        assert d.event_heading is None


# ── DENM network store tests ─────────────────────────────────────────────────


class TestDENMNetworkStore:
    def test_publish_adds_to_store(self):
        net = _make_network()
        d = _make_denm(station_id=1, action_id=10)
        net.publish_denm(d)
        assert len(net.get_denm()) == 1

    def test_get_denm_all(self):
        net = _make_network()
        net.publish_denm(_make_denm(station_id=1, action_id=1))
        net.publish_denm(_make_denm(station_id=2, action_id=1))
        assert len(net.get_denm()) == 2

    def test_get_denm_by_station(self):
        net = _make_network()
        net.publish_denm(_make_denm(station_id=1, action_id=1))
        net.publish_denm(_make_denm(station_id=1, action_id=2))
        net.publish_denm(_make_denm(station_id=2, action_id=1))
        result = net.get_denm(station_id=1)
        assert len(result) == 2
        assert all(d.station_id == 1 for d in result)

    def test_get_denm_unknown_station_returns_empty(self):
        net = _make_network()
        net.publish_denm(_make_denm(station_id=1, action_id=1))
        assert net.get_denm(station_id=99) == []

    def test_publish_same_key_updates_in_place(self):
        net = _make_network()
        net.publish_denm(_make_denm(station_id=1, action_id=1,
                                    cause=DENMCauseCode.ACCIDENT))
        updated = DENM(
            station_id=1, action_id=1,
            detection_time=300.0, reference_time=300.0,
            event_position=(0.0, 0.0, 0.0),
            cause_code=DENMCauseCode.ROAD_WORKS,
        )
        net.publish_denm(updated)
        stored = net.get_denm()
        assert len(stored) == 1
        assert stored[0].cause_code == DENMCauseCode.ROAD_WORKS

    def test_cancel_removes_denm(self):
        net = _make_network()
        net.publish_denm(_make_denm(station_id=1, action_id=5))
        removed = net.cancel_denm(station_id=1, action_id=5)
        assert removed is True
        assert len(net.get_denm()) == 0

    def test_cancel_nonexistent_returns_false(self):
        net = _make_network()
        assert net.cancel_denm(station_id=99, action_id=99) is False

    def test_cancel_only_removes_matching_entry(self):
        net = _make_network()
        net.publish_denm(_make_denm(station_id=1, action_id=1))
        net.publish_denm(_make_denm(station_id=1, action_id=2))
        net.cancel_denm(station_id=1, action_id=1)
        remaining = net.get_denm()
        assert len(remaining) == 1
        assert remaining[0].action_id == 2


# ── create_cam_from_bsm tests ────────────────────────────────────────────────


class TestCreateCamFromBsm:
    def test_station_id_maps_to_vehicle_id(self):
        bsm = _make_bsm(vehicle_id=7)
        cam = create_cam_from_bsm(bsm)
        assert cam.station_id == 7

    def test_reference_position_maps_correctly(self):
        bsm = _make_bsm(x=100.0, y=200.0)
        cam = create_cam_from_bsm(bsm)
        assert cam.reference_position == (100.0, 200.0, 0.0)

    def test_speed_and_heading_copied(self):
        bsm = _make_bsm(speed=12.5)
        bsm.heading = 45.0
        cam = create_cam_from_bsm(bsm)
        assert cam.speed == 12.5
        assert cam.heading == 45.0

    def test_forward_direction(self):
        bsm = _make_bsm()
        bsm.transmission_state = "forward"
        cam = create_cam_from_bsm(bsm)
        assert cam.drive_direction == "forward"

    def test_reverse_direction(self):
        bsm = _make_bsm()
        bsm.transmission_state = "reverse"
        cam = create_cam_from_bsm(bsm)
        assert cam.drive_direction == "reverse"

    def test_generation_time_equals_bsm_timestamp(self):
        bsm = _make_bsm()
        cam = create_cam_from_bsm(bsm)
        assert cam.generation_time == bsm.timestamp


# ── API endpoint tests ────────────────────────────────────────────────────────


def _setup_network_with_vehicles():
    """Two registered vehicles at known positions, after one forced update."""
    dsrc = _transparent_dsrc()
    v1 = MockVehicle(1, x=0, y=0)
    v2 = MockVehicle(2, x=50, y=0)
    world = MockWorld()
    world.add_vehicle(v1)
    world.add_vehicle(v2)
    net = V2VNetworkEnhanced(max_range=200.0, update_rate_hz=2.0,
                             dsrc_config=dsrc, world=world)
    net.register(1, v1)
    net.register(2, v2)
    snap = MockWorldSnapshot([v1, v2], elapsed_seconds=1.0)
    net.update(snapshot=snap, force=True)
    return net


class TestCAMEndpoints:
    def test_get_all_cam_returns_list(self):
        net = _setup_network_with_vehicles()
        client = _make_api(net)
        resp = client.get("/cam")
        assert resp.status_code == 200
        data = resp.json()
        assert isinstance(data, list)
        assert len(data) == 2

    def test_cam_response_shape(self):
        net = _setup_network_with_vehicles()
        client = _make_api(net)
        resp = client.get("/cam")
        item = resp.json()[0]
        for field in ("station_id", "generation_time", "reference_position",
                      "heading", "speed", "drive_direction", "vehicle_role",
                      "vehicle_length", "vehicle_width"):
            assert field in item, f"Missing field: {field}"

    def test_cam_reference_position_has_xyz(self):
        net = _setup_network_with_vehicles()
        client = _make_api(net)
        resp = client.get("/cam")
        pos = resp.json()[0]["reference_position"]
        assert "x" in pos and "y" in pos and "z" in pos

    def test_get_cam_by_station_id(self):
        net = _setup_network_with_vehicles()
        client = _make_api(net)
        resp = client.get("/cam/1")
        assert resp.status_code == 200
        assert resp.json()["station_id"] == 1

    def test_get_cam_unknown_station_returns_404(self):
        net = _setup_network_with_vehicles()
        client = _make_api(net)
        resp = client.get("/cam/999")
        assert resp.status_code == 404


class TestDENMEndpoints:
    def _client_with_denm(self):
        net = _make_network()
        net.publish_denm(_make_denm(station_id=1, action_id=1))
        net.publish_denm(_make_denm(station_id=2, action_id=1,
                                    cause=DENMCauseCode.ROAD_WORKS))
        return _make_api(net), net

    def test_get_all_denm_returns_list(self):
        client, _ = self._client_with_denm()
        resp = client.get("/denm")
        assert resp.status_code == 200
        assert len(resp.json()) == 2

    def test_denm_response_shape(self):
        client, _ = self._client_with_denm()
        resp = client.get("/denm")
        item = resp.json()[0]
        for field in ("station_id", "action_id", "detection_time", "reference_time",
                      "event_position", "cause_code", "cause_code_name",
                      "subcause_code", "information_quality", "relevance_distance",
                      "relevance_traffic_direction", "termination"):
            assert field in item, f"Missing field: {field}"

    def test_denm_event_position_has_xyz(self):
        client, _ = self._client_with_denm()
        resp = client.get("/denm")
        pos = resp.json()[0]["event_position"]
        assert "x" in pos and "y" in pos and "z" in pos

    def test_get_denm_by_station(self):
        client, _ = self._client_with_denm()
        resp = client.get("/denm/1")
        assert resp.status_code == 200
        data = resp.json()
        assert len(data) == 1
        assert data[0]["station_id"] == 1

    def test_get_denm_unknown_station_returns_404(self):
        client, _ = self._client_with_denm()
        resp = client.get("/denm/999")
        assert resp.status_code == 404

    def test_post_denm_creates_alert(self):
        net = _make_network()
        client = _make_api(net)
        payload = {
            "station_id": 5,
            "action_id": 10,
            "detection_time": 300.0,
            "reference_time": 300.0,
            "event_position": {"x": 1.0, "y": 2.0, "z": 0.0},
            "cause_code": int(DENMCauseCode.COLLISION_RISK),
            "subcause_code": 0,
            "information_quality": 7,
            "relevance_distance": 200.0,
            "relevance_traffic_direction": "allTrafficDirections",
        }
        resp = client.post("/denm", json=payload)
        assert resp.status_code == 201
        body = resp.json()
        assert body["station_id"] == 5
        assert body["action_id"] == 10
        assert body["cause_code_name"] == "COLLISION_RISK"
        assert body["event_position"] == {"x": 1.0, "y": 2.0, "z": 0.0}
        # Verify it is actually stored
        assert len(net.get_denm()) == 1

    def test_post_denm_invalid_cause_returns_422(self):
        net = _make_network()
        client = _make_api(net)
        payload = {
            "station_id": 1, "action_id": 1,
            "detection_time": 0.0, "reference_time": 0.0,
            "event_position": {"x": 0.0, "y": 0.0, "z": 0.0},
            "cause_code": 999,  # not a valid DENMCauseCode
        }
        resp = client.post("/denm", json=payload)
        assert resp.status_code == 422

    def test_delete_denm_removes_alert(self):
        net = _make_network()
        net.publish_denm(_make_denm(station_id=3, action_id=7))
        client = _make_api(net)
        resp = client.delete("/denm/3/7")
        assert resp.status_code == 204
        assert len(net.get_denm()) == 0

    def test_delete_nonexistent_denm_returns_404(self):
        net = _make_network()
        client = _make_api(net)
        resp = client.delete("/denm/99/99")
        assert resp.status_code == 404

    def test_root_lists_cam_and_denm_endpoints(self):
        net = _make_network()
        client = _make_api(net)
        resp = client.get("/")
        assert resp.status_code == 200
        endpoints = resp.json()["endpoints"]
        assert "/cam" in endpoints
        assert "/denm" in endpoints
