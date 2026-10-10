"""/api/status, /api/config(+schema), /api/map/geometry and the admin DENM endpoints (no CARLA)."""

import pytest

pytest.importorskip("httpx", reason="fastapi TestClient needs httpx")

from fastapi.testclient import TestClient  # noqa: E402

import src.server.app as app_mod
from tests.contract.fake_world import FakeWorld

TOKEN = "test-token"
ADMIN = {"X-Admin-Token": TOKEN}


@pytest.fixture(autouse=True)
def _env(monkeypatch):
    monkeypatch.setenv("ADMIN_TOKEN", TOKEN)
    yield
    app_mod.set_persistent_world(None)


@pytest.fixture
def client():
    return TestClient(app_mod.app)


@pytest.fixture
def world():
    w = FakeWorld()
    app_mod.set_persistent_world(w)
    return w


def test_status_before_connect_is_not_an_error(client):
    r = client.get("/api/status")
    assert r.status_code == 200 and r.json() == {"connected": False}


def test_status_when_connected(client, world):
    s = client.get("/api/status").json()
    assert s["connected"] is True and s["map"] == "Town01" and s["fixed_delta_seconds"] == 0.05
    assert s["observer_public"] is True and s["run_active"] is False


def test_config_and_schema(client, world):
    c = client.get("/api/config").json()
    assert c["fixed_delta_seconds"] == 0.05 and c["lidar_max_sensors"] == 4 and c["session_disconnect_grace_s"] == 30.0
    assert "admin_token" not in str(c).lower()
    schema = client.get("/api/config/schema").json()
    props = schema["properties"]
    assert props["fixed_delta_seconds"]["type"] == "number" and "lidar_max_sensors" in props
    assert "host" in schema["required"]


def test_config_503_without_world(client):
    assert client.get("/api/config").status_code == 503


def test_observer_gate(client, world):
    world.observer_public = False
    for path in ("/api/config", "/api/config/schema", "/api/map/geometry", "/api/v2x/denm"):
        assert client.get(path).status_code == 403, path
        assert client.get(path, headers=ADMIN).status_code == 200, path
    assert client.get("/api/config", headers={"X-Admin-Token": "bad"}).status_code == 403


def test_map_geometry_etag_and_gzip(client, world):
    r = client.get("/api/map/geometry")
    assert r.status_code == 200 and r.json()["lanes"][0][1] == [2.0, 0.0]
    etag = r.headers["etag"]
    assert etag == '"abc123"'
    assert client.get("/api/map/geometry", headers={"If-None-Match": etag}).status_code == 304
    assert client.get("/api/map/geometry", headers={"If-None-Match": '"other"'}).status_code == 200
    big = world.map_geometry["lanes"] = [[[float(i), 0.0] for i in range(500)]]
    gz = client.get("/api/map/geometry", headers={"Accept-Encoding": "gzip"})
    assert gz.headers.get("content-encoding") == "gzip" and len(gz.json()["lanes"][0]) == len(big[0])


# ── DENM ──────────────────────────────────────────────────────────────────────────────────────────

def test_denm_injection_needs_the_admin_token(client, world):
    body = {"x": 10, "y": 20}
    assert client.post("/api/v2x/denm", json=body).status_code == 422          # header missing
    assert client.post("/api/v2x/denm", json=body, headers={"X-Admin-Token": "bad"}).status_code == 403
    assert client.delete("/api/v2x/denm/1/1").status_code == 422
    assert world.denms == {}


def test_denm_inject_list_cancel(client, world):
    r = client.post("/api/v2x/denm", json={"x": 10, "y": 20, "cause_code": "obstacle_on_road",
                                           "relevance_distance_m": 80}, headers=ADMIN)
    assert r.status_code == 201
    d = r.json()
    assert d["action_id"] == 1 and d["station_id"] < -1_000_000
    listed = client.get("/api/v2x/denm").json()["denms"]
    assert listed[0]["cause_code"] == 10 and listed[0]["relevance_distance_m"] == 80
    assert client.delete(f"/api/v2x/denm/{d['station_id']}/{d['action_id']}", headers=ADMIN).json() == {"cancelled": True}
    assert client.delete(f"/api/v2x/denm/{d['station_id']}/{d['action_id']}", headers=ADMIN).status_code == 404
    assert client.get("/api/v2x/denm").json()["denms"] == []


@pytest.mark.parametrize("body", [{"x": 0, "y": 0, "cause_code": "teleport"}, {"x": 0, "y": 0, "cause_code": 12345},
                                  {"x": 0, "y": 0, "relevance_distance_m": 0}, {"x": 0, "y": 0, "information_quality": 9},
                                  {"x": 0, "y": 0, "duration_s": -1}, {"y": 0}, {"x": 0, "y": 0, "station_id": 99999}])
def test_bad_denm_requests_are_rejected(client, world, body):
    assert client.post("/api/v2x/denm", json=body, headers=ADMIN).status_code == 422
    assert world.denms == {}


def test_cause_codes_accept_numbers_and_names(client, world):
    for cause, num in (("ACCIDENT", 2), (2, 2), ("road_works", 3), (97, 97)):
        r = client.post("/api/v2x/denm", json={"x": 0, "y": 0, "cause_code": cause}, headers=ADMIN)
        assert r.status_code == 201, (cause, r.text)
    assert sorted(v["cause_code"] for v in world.denms.values()) == [2, 2, 3, 97]
