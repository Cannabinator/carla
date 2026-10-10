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


# ── hot configuration, schema marks, RSUs ─────────────────────────────────────────────────────────

def test_schema_marks_every_field_hot_or_restart(client, world):
    props = client.get("/api/config/schema").json()["properties"]
    assert props["awareness_range_m"]["x-apply"] == "hot" and props["awareness_range_m"]["x-effect"] == "next V2X update"
    assert props["fixed_delta_seconds"]["x-apply"] == "restart" and "x-effect" not in props["fixed_delta_seconds"]
    assert all(p["x-apply"] in ("hot", "restart") for p in props.values())


def test_patch_config_applies_hot_fields_and_reports_the_frame(client, world):
    r = client.patch("/api/config", json={"awareness_range_m": 120.0, "lidar_max_sensors": 2}, headers=ADMIN)
    assert r.status_code == 200
    body = r.json()
    assert body["applied"]["awareness_range_m"] == {"old": 75.0, "new": 120.0, "effective": "next V2X update"}
    assert body["applied"]["lidar_max_sensors"]["old"] == 4 and body["applied_at_frame"] == 42
    assert client.get("/api/config").json()["awareness_range_m"] == 120.0
    again = client.patch("/api/config", json={"awareness_range_m": 120.0}, headers=ADMIN).json()
    assert again["applied"] == {}, "an unchanged value is not a change"


def test_patch_config_is_all_or_nothing_and_explains_refusals(client, world):
    r = client.patch("/api/config", json={"awareness_range_m": 99.0, "fixed_delta_seconds": 0.1}, headers=ADMIN)
    assert r.status_code == 409 and r.json()["detail"]["error"] == "restart_required"
    assert "fixed_delta_seconds" in r.json()["detail"]["message"] and "awareness_range_m" in r.json()["detail"]["hot_fields"]
    assert client.get("/api/config").json()["awareness_range_m"] == 75.0, "nothing was applied"
    assert client.patch("/api/config", json={"nope": 1}, headers=ADMIN).status_code == 422
    r = client.patch("/api/config", json={"awareness_range_m": 5.0}, headers=ADMIN)             # below the allowed minimum
    assert r.status_code == 422 and "awareness_range_m" in str(r.json()["detail"])
    r = client.patch("/api/config", json={"world_state_rate_hz": 7.0}, headers=ADMIN)           # not a whole number of frames
    assert r.status_code == 422


def test_patch_config_needs_the_admin_token(client, world):
    assert client.patch("/api/config", json={"awareness_range_m": 99.0}).status_code == 422
    assert client.patch("/api/config", json={"awareness_range_m": 99.0}, headers={"X-Admin-Token": "bad"}).status_code == 403
    assert client.get("/api/config").json()["awareness_range_m"] == 75.0


def test_rsu_list_and_patch(client, world):
    world.publish_world_state([])
    rs = client.get("/api/v2x/rsus").json()["rsus"]
    assert rs[0]["station_id"] == -1 and rs[0]["enabled"] is True and rs[0]["tx_power_dbm"] == 33.0
    r = client.patch("/api/v2x/rsus/-1", json={"enabled": False, "tx_power_dbm": 20.5}, headers=ADMIN)
    assert r.status_code == 200 and r.json()["enabled"] is False
    rs = client.get("/api/v2x/rsus").json()["rsus"]
    assert rs[0]["enabled"] is False and rs[0]["tx_power_dbm"] == 20.5
    assert client.patch("/api/v2x/rsus/-99", json={"enabled": True}, headers=ADMIN).status_code == 404
    assert client.patch("/api/v2x/rsus/5", json={"enabled": True}, headers=ADMIN).status_code == 404       # vehicles are not RSUs
    assert client.patch("/api/v2x/rsus/-1", json={"tx_power_dbm": 99}, headers=ADMIN).status_code == 422
    assert client.patch("/api/v2x/rsus/-1", json={"enabled": True}).status_code == 422


# ── runs ──────────────────────────────────────────────────────────────────────────────────────────

def test_runs_need_the_admin_token(client, world):
    assert client.post("/api/runs", json={"name": "x"}).status_code == 422
    assert client.post("/api/runs", json={"name": "x"}, headers={"X-Admin-Token": "bad"}).status_code == 403
    assert client.get("/api/runs", headers={"X-Admin-Token": "bad"}).status_code == 403
    assert client.get("/api/runs/anything/manifest").status_code == 422
    assert client.post("/api/runs/current/stop").status_code == 422


def test_run_lifecycle_and_refusals(client, world):
    assert client.get("/api/status").json()["run_active"] is False
    r = client.post("/api/runs", json={"name": "cross", "record": ["links", "stations"]}, headers=ADMIN)
    assert r.status_code == 201 and r.json()["run_id"].endswith("_cross")
    assert client.get("/api/status").json()["run_active"] is True
    again = client.post("/api/runs", json={"name": "second"}, headers=ADMIN)
    assert again.status_code == 409 and again.json()["detail"]["error"] == "run_active"
    listing = client.get("/api/runs", headers=ADMIN).json()
    assert listing["active"]["name"] == "cross" and listing["runs"] == []
    assert client.post("/api/runs/current/stop", headers=ADMIN).json()["status"] == "finished"
    assert client.post("/api/runs/current/stop", headers=ADMIN).status_code == 404
    rid = r.json()["run_id"]
    assert client.get("/api/runs", headers=ADMIN).json()["runs"][0]["run_id"] == rid
    assert client.get(f"/api/runs/{rid}/manifest", headers=ADMIN).json()["schema_version"] == 1
    assert client.get("/api/runs/nope/manifest", headers=ADMIN).status_code == 404


def test_run_request_is_validated(client, world):
    assert client.post("/api/runs", json={"name": ""}, headers=ADMIN).status_code == 422
    assert client.post("/api/runs", json={"name": "x", "record": ["nonsense"]}, headers=ADMIN).status_code == 422
    r = client.post("/api/runs", json={"name": "x", "config": {"fixed_delta_seconds": 0.1}}, headers=ADMIN)
    assert r.status_code == 409 and r.json()["detail"]["error"] == "restart_required"
    ok = client.post("/api/runs", json={"name": "x", "config": {"awareness_range_m": 90.0}}, headers=ADMIN)
    assert ok.status_code == 201 and client.get("/api/config").json()["awareness_range_m"] == 90.0
