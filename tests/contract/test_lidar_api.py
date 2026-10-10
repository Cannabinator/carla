"""LiDAR REST + WebSocket contract (no CARLA): the real routes and LidarManager over a fake sensor backend."""

import numpy as np
import pytest

pytest.importorskip("httpx", reason="fastapi TestClient needs httpx")

from fastapi.testclient import TestClient  # noqa: E402

import src.server.app as app_mod
from src.lidar.protocol import decode_frame
from tests.contract import ws_helpers
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


def lconnect(client, world, path):
    return ws_helpers.connect(client, path, lambda: all(world.lidar.viewer_count(i) == 0 for i in range(1, 8)))


def join(client, name="car", **extra):
    r = client.post("/client/join", json={"client_name": name, **extra})
    assert r.status_code == 200, r.text
    return r.json()


def tick_until_active(world, sensor_id, start=1):
    f = start
    while world.lidar.state_of(sensor_id).value != "active":
        world.lidar_tick(f)
        f += 1
        assert f < start + 5
    return f


# ── read side (observers) ─────────────────────────────────────────────────────────────────────────

def test_schema_and_sensor_list_are_public(client, world):
    s = client.get("/api/lidar/schema")
    assert s.status_code == 200 and s.json()["protocol_version"] == 3
    assert s.json()["limits"]["max_sensors"] == 2
    assert "noise_stddev" not in {a["name"] for a in s.json()["sensor_types"]["ray_cast_semantic"]["attributes"]}
    lst = client.get("/api/lidar/sensors").json()
    assert lst["sensors"] == [] and lst["budget"]["max_sensors"] == 2


def test_503_without_a_world(client):
    for path in ("/api/lidar/schema", "/api/lidar/sensors"):
        assert client.get(path).status_code == 503


# ── creating sensors / roles (D9) ─────────────────────────────────────────────────────────────────

def test_observer_cannot_create(client, world):
    r = client.post("/api/lidar/sensors", json={"actor_id": 5})
    assert r.status_code == 403 and r.json()["error"] == "forbidden"


def test_admin_creates_on_any_actor_and_gets_ws_url(client, world):
    r = client.post("/api/lidar/sensors", json={"actor_id": 555, "pinned": True}, headers=ADMIN)
    assert r.status_code == 202
    body = r.json()
    assert body["state"] == "requested" and body["ws_url"] == f"/ws/lidar/{body['sensor_id']}"
    (info,) = client.get("/api/lidar/sensors").json()["sensors"]
    assert info["actor_id"] == 555 and info["owner"] == "admin" and info["pinned"] is True


def test_wrong_admin_token_is_403(client, world):
    r = client.post("/api/lidar/sensors", json={"actor_id": 1}, headers={"X-Admin-Token": "nope"})
    assert r.status_code == 403


def test_vehicle_creates_on_own_actor_only(client, world):
    j = join(client)
    h = {"X-Session-Token": j["session_token"]}
    assert client.post("/api/lidar/sensors", json={"actor_id": j["actor_id"] + 1}, headers=h).status_code == 403
    r = client.post("/api/lidar/sensors", json={}, headers=h)          # defaults to its own actor
    assert r.status_code == 202
    (info,) = client.get("/api/lidar/sensors").json()["sensors"]
    assert info["actor_id"] == j["actor_id"] and info["owner"] == "car"      # the client name, not the session id
    assert client.post("/api/lidar/sensors", json={"pinned": True}, headers=h).status_code == 403


def test_vehicle_session_must_exist(client, world):
    r = client.post("/api/lidar/sensors", json={}, headers={"X-Session-Token": "ghost"})
    assert r.status_code == 401


def test_session_convenience_endpoint(client, world):
    j = join(client)
    h = {"X-Session-Token": j["session_token"]}
    r = client.post(f"/api/sessions/{j['session_id']}/lidar", json={"channels": 16}, headers=h)
    assert r.status_code == 202
    (info,) = client.get("/api/lidar/sensors").json()["sensors"]
    assert info["config"]["channels"] == 16 and info["actor_id"] == j["actor_id"]
    assert client.post("/api/sessions/nope/lidar", headers=h).status_code == 403     # token of another session
    assert client.post(f"/api/sessions/{j['session_id']}/lidar").status_code == 403  # no token: observer is not the owner
    other = join(client, "other")
    assert client.post(f"/api/sessions/{j['session_id']}/lidar",
                       headers={"X-Session-Token": other["session_token"]}).status_code == 403


def test_invalid_config_is_422(client, world):
    r = client.post("/api/lidar/sensors", json={"actor_id": 1, "config": {"channels": 0}}, headers=ADMIN)
    assert r.status_code == 422
    r = client.post("/api/lidar/sensors", json={"actor_id": 1, "config": {"noise_stddev": 0.1}}, headers=ADMIN)
    assert r.status_code == 422                   # raycast-only attribute on the default semantic sensor
    assert client.get("/api/lidar/sensors").json()["sensors"] == []


def test_config_warning_when_not_one_sweep_per_frame(client, world):
    r = client.post("/api/lidar/sensors", json={"actor_id": 1, "config": {"rotation_frequency_hz": 10}}, headers=ADMIN)
    assert r.status_code == 202 and any("full sweep" in w for w in r.json()["warnings"])


# ── budget ────────────────────────────────────────────────────────────────────────────────────────

def test_budget_409_body_matches_the_spec(client, world):
    ok = client.post("/api/lidar/sensors", json={"actor_id": 1, "config": {"points_per_second": 700_000}}, headers=ADMIN)
    assert ok.status_code == 202
    r = client.post("/api/lidar/sensors", json={"actor_id": 2, "config": {"points_per_second": 400_000}}, headers=ADMIN)
    assert r.status_code == 409
    body = r.json()
    assert body["error"] == "lidar_budget_exceeded"
    assert body["requested_pps"] == 400_000 and body["available_pps"] == 300_000
    assert len(client.get("/api/lidar/sensors").json()["sensors"]) == 1


def test_join_with_lidar_over_budget_still_joins_and_reports(client, world):
    client.post("/api/lidar/sensors", json={"actor_id": 1, "config": {"points_per_second": 900_000}}, headers=ADMIN)
    j = join(client, enable_lidar=True)            # default 500k does not fit
    assert j["lidar_sensor_id"] is None and j["lidar_ws_url"] is None
    assert j["lidar_error"]["error"] == "lidar_budget_exceeded"


def test_join_with_lidar_requests_a_sensor_on_the_new_vehicle(client, world):
    j = join(client, enable_lidar=True, lidar_config={"channels": 16})
    assert j["lidar_ws_url"] == f"/ws/lidar/{j['lidar_sensor_id']}" and j["lidar_error"] is None
    (info,) = client.get("/api/lidar/sensors").json()["sensors"]
    assert info["actor_id"] == j["actor_id"] and info["config"]["channels"] == 16


# ── change / release ──────────────────────────────────────────────────────────────────────────────

def test_reconfigure_release_and_ownership(client, world):
    a, b = join(client, "a"), join(client, "b")
    sid = client.post(f"/api/sessions/{a['session_id']}/lidar",
                      headers={"X-Session-Token": a["session_token"]}).json()["sensor_id"]
    other = {"X-Session-Token": b["session_token"]}
    assert client.put(f"/api/lidar/sensors/{sid}", json={"channels": 64}, headers=other).status_code == 403
    assert client.delete(f"/api/lidar/sensors/{sid}", headers=other).status_code == 403
    assert client.delete(f"/api/lidar/sensors/{sid}").status_code == 403           # observer

    mine = {"X-Session-Token": a["session_token"]}
    world.lidar_tick(1)
    r = client.put(f"/api/lidar/sensors/{sid}", json={"channels": 64}, headers=mine)
    assert r.status_code == 202 and r.json()["state"] == "reconfiguring"
    world.lidar_tick(2)
    assert world.lidar_backend.handles[-1].config.channels == 64
    assert client.delete(f"/api/lidar/sensors/{sid}", headers=ADMIN).status_code == 202
    world.lidar_tick(3)
    assert world.lidar_backend.live == []
    assert client.delete(f"/api/lidar/sensors/{sid}", headers=ADMIN).status_code == 404


def test_leaving_the_session_releases_its_sensor(client, world):
    j = join(client, enable_lidar=True)
    world.lidar_tick(1)
    assert client.delete(f"/client/leave/{j['session_id']}").status_code == 200
    world.lidar_tick(2)
    assert world.lidar_backend.live == []


# ── WebSocket ─────────────────────────────────────────────────────────────────────────────────────

def recv_until_bytes(ws, max_msgs=20):
    texts = []
    for _ in range(max_msgs):
        m = ws_helpers.recv_raw(ws)
        if m.get("bytes") is not None:
            return texts, m["bytes"]
        texts.append(m["text"])
    raise AssertionError(f"no binary frame, got {texts}")


def test_ws_streams_decodable_frames_and_state(client, world):
    import json
    j = join(client, enable_lidar=True)
    sid = j["lidar_sensor_id"]
    with lconnect(client, world, j["lidar_ws_url"]) as ws:
        assert json.loads(ws_helpers.recv_raw(ws)['text'])["state"] == "requested"
        assert world.lidar.viewer_count(sid) == 1
        tick_until_active(world, sid)
        texts, data = recv_until_bytes(ws)
        states = [json.loads(t) for t in texts]
        assert any(s["type"] == "state" and s["state"] in ("warming_up", "active") for s in states)
        header, pts = decode_frame(data)
        assert header.sensor_id == sid and header.ego_actor_id == j["actor_id"] and header.num_points == 200
        assert header.frame == world.lidar.get_frame(sid).frame
        assert np.array_equal(pts["x"], world.lidar.get_frame(sid).points["x"])
    assert world.lidar.viewer_count(sid) == 0           # disconnect detaches the viewer


def test_ws_two_viewers_get_identical_bytes_and_one_encode(client, world):
    j = join(client, enable_lidar=True)
    sid = j["lidar_sensor_id"]
    with lconnect(client, world, j["lidar_ws_url"]) as w1, lconnect(client, world, j["lidar_ws_url"]) as w2:
        assert world.lidar.viewer_count(sid) == 2
        tick_until_active(world, sid)
        _, d1 = recv_until_bytes(w1)
        _, d2 = recv_until_bytes(w2)
        assert d1 == d2
        assert world.get_lidar_stream(sid).encodes >= 1


def test_ws_profile_message_subsamples_and_reports_it(client, world):
    import json
    j = join(client, enable_lidar=True)
    sid = j["lidar_sensor_id"]
    with lconnect(client, world, j["lidar_ws_url"]) as ws:
        ws.send_text(json.dumps({"op": "profile", "max_points": 100, "max_hz": 20}))   # 100 -> smallest step 10k > 200 pts
        tick_until_active(world, sid)
        texts, data = recv_until_bytes(ws)
        prof = [json.loads(t) for t in texts if json.loads(t)["type"] == "stream_profile"]
        assert prof and prof[-1]["max_points"] == 10_000 and prof[-1]["subsampled"] is False
        assert decode_frame(data)[0].num_points == 200


def test_ws_bad_profile_gets_an_error_not_a_disconnect(client, world):
    import json
    j = join(client, enable_lidar=True)
    with lconnect(client, world, j["lidar_ws_url"]) as ws:
        ws.send_text(json.dumps({"op": "profile", "max_hz": -3}))
        for _ in range(5):
            m = json.loads(ws_helpers.recv_raw(ws)['text'])
            if m["type"] == "error":
                assert "max_hz" in m["message"]
                break
        else:
            raise AssertionError("no error message")
        ws.send_text("ping")


def test_ws_unknown_sensor(client, world):
    import json
    with lconnect(client, world, "/ws/lidar/999") as ws:
        assert json.loads(ws_helpers.recv_raw(ws)['text'])["type"] == "error"


def test_ws_gets_destroyed_state_when_released(client, world):
    import json
    j = join(client, enable_lidar=True)
    sid = j["lidar_sensor_id"]
    with lconnect(client, world, j["lidar_ws_url"]) as ws:
        tick_until_active(world, sid)
        client.delete(f"/api/lidar/sensors/{sid}", headers=ADMIN)
        world.lidar_tick(10)
        seen = []
        for _ in range(20):
            m = ws_helpers.recv_raw(ws)
            if m.get("text") is None:
                continue
            seen.append(json.loads(m["text"]))
            if seen[-1].get("type") == "state" and seen[-1]["state"] == "destroyed":
                break
        assert seen[-1]["state"] == "destroyed"


def test_nothing_an_observer_can_read_contains_a_session_id_or_token(client, world):
    j = join(client, enable_lidar=True)
    world.lidar_tick(1)
    text = client.get("/api/lidar/sensors").text
    assert j["session_id"] not in text and j["session_token"] not in text


def test_admin_pins_and_unpins_a_sensor_others_cannot(client, world):
    j = join(client)
    mine = {"X-Session-Token": j["session_token"]}
    sid = client.post("/api/lidar/sensors", json={}, headers=mine).json()["sensor_id"]
    assert client.patch(f"/api/lidar/sensors/{sid}", json={"pinned": True}, headers=mine).status_code == 403
    assert client.patch(f"/api/lidar/sensors/{sid}", json={"pinned": True}).status_code == 403
    r = client.patch(f"/api/lidar/sensors/{sid}", json={"pinned": True, "record_raw": True}, headers=ADMIN)
    assert r.status_code == 202
    world.lidar_tick(1)
    (info,) = client.get("/api/lidar/sensors").json()["sensors"]
    assert info["pinned"] is True and info["record_raw"] is True
    assert client.patch("/api/lidar/sensors/99", json={"pinned": True}, headers=ADMIN).status_code == 404
