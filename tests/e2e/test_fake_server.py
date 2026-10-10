"""The real server process on the fake CARLA: everything the browser needs, over real HTTP and WebSockets."""

import json
import time

import pytest

from src.lidar.protocol import decode_frame

websockets = pytest.importorskip("websockets.sync.client")


def test_status_says_fake_and_connected(fake_server):
    status, s = fake_server.json("GET", "/api/status")
    assert status == 200 and s["connected"] and s["fake"] is True and s["map"].endswith("FakeTown")


def test_world_has_the_initial_population_and_rsus(fake_server):
    got = {}
    with websockets.connect(fake_server.ws_base + "/ws", open_timeout=10) as ws:
        for topic in ("world.state", "v2x.frame", "run.status", "v2x.stats"):
            ws.send(json.dumps({"op": "subscribe", "topic": topic, "max_hz": 2}))
        end = time.time() + 15
        while time.time() < end and len({k for k in got if k in ("world.state", "v2x.frame", "run.status", "v2x.stats")}) < 4:
            m = json.loads(ws.recv(timeout=5))
            got[m["topic"]] = m
    ws_data = got["world.state"]["data"]
    kinds = [s["kind"] for s in ws_data["stations"]]
    # the server is shared with the admin tests, which respawn vehicles: do not assume the exact initial count
    assert kinds.count("vehicle") >= 6 and kinds.count("pedestrian") == 6 and len(ws_data["rsus"]) == 9
    summary = got["v2x.frame"]["data"]
    assert {s["kind"] for s in summary["stations"]} == {"OBU", "RSU", "VRU"}
    assert summary["rsu_states"], "RSUs must publish SPAT"
    assert got["v2x.stats"]["data"]["window"]["updates"] >= 1
    assert got["run.status"]["data"]["tick_duration_ms"]["p50"] is not None


def test_map_geometry(fake_server):
    status, g = fake_server.json("GET", "/api/map/geometry", timeout=60)
    assert status == 200 and g["map_name"].endswith("FakeTown") and len(g["lanes"]) == 84 and len(g["junctions"]) == 9
    xs = [p[0] for lane in g["lanes"] for p in lane]
    assert min(xs) < -30 and max(xs) > 230


def test_vehicle_lidar_lifecycle_over_http_and_websocket(fake_server):
    status, j = fake_server.json("POST", "/client/join", {"client_name": "e2e-car"})
    assert status == 200, j
    h = {"X-Session-Token": j["session_token"]}
    try:
        status, c = fake_server.json("POST", f"/api/sessions/{j['session_id']}/lidar", {"points_per_second": 100000}, headers=h)
        assert status == 202, c
        sid = c["sensor_id"]
        frames, states = [], []
        with websockets.connect(fake_server.ws_base + c["ws_url"], open_timeout=10, max_size=None) as ws:
            end = time.time() + 10
            while time.time() < end and len(frames) < 5:
                m = ws.recv(timeout=5)
                if isinstance(m, str):
                    states.append(json.loads(m))
                else:
                    frames.append(decode_frame(m))
        assert len(frames) == 5, states
        header, pts = frames[-1]
        assert header.sensor_id == sid and header.ego_actor_id == j["actor_id"] and header.num_points > 1000
        assert [h.frame for h, _ in frames] == sorted({h.frame for h, _ in frames})
        assert (pts["object_idx"] == j["actor_id"]).any(), "own vehicle self-hits are expected"
        assert any(s.get("state") == "active" for s in states if s.get("type") == "state")
        # reconfigure while a viewer would auto-release otherwise
        status, _ = fake_server.json("PUT", f"/api/lidar/sensors/{sid}", {"channels": 16, "points_per_second": 100000}, headers=h)
        assert status == 202
    finally:
        fake_server.json("DELETE", f"/client/leave/{j['session_id']}")
    deadline = time.time() + 5
    while time.time() < deadline:
        _, lst = fake_server.json("GET", "/api/lidar/sensors")
        if not lst["sensors"]:
            break
        time.sleep(0.2)
    assert lst["sensors"] == [] and lst["budget"]["used_pps"] == 0, "the sensor must die with its session"


def test_denm_injection_reaches_a_vehicle(fake_server):
    status, j = fake_server.json("POST", "/client/join", {"client_name": "e2e-denm"})
    assert status == 200
    try:
        deadline = time.time() + 5
        tel = None
        while time.time() < deadline and tel is None:
            s, tel = fake_server.json("GET", f"/client/vehicle/{j['session_id']}")
            tel = tel if s == 200 else None
            time.sleep(0.1)
        assert tel is not None
        status, d = fake_server.json("POST", "/api/v2x/denm", {"x": tel["x"], "y": tel["y"], "cause_code": "ACCIDENT",
                                                                "relevance_distance_m": 300, "duration_s": 60}, admin=True)
        assert status == 201, d
        with websockets.connect(fake_server.ws_base + "/ws", open_timeout=10) as ws:
            ws.send(json.dumps({"op": "auth", "session_token": j["session_token"]}))
            ws.send(json.dumps({"op": "subscribe", "topic": "v2x.inbox", "max_hz": 2}))
            end, seen = time.time() + 10, False
            while time.time() < end and not seen:
                m = json.loads(ws.recv(timeout=5))
                if m["topic"] == "v2x.inbox":
                    seen = any(x["msg_type"] == "DENM" for x in m["data"]["messages"])
        assert seen, "the vehicle never received the DENM"
    finally:
        fake_server.json("DELETE", f"/client/leave/{j['session_id']}")


def test_a_vehicle_token_gets_no_ground_truth(fake_server):
    status, j = fake_server.json("POST", "/client/join", {"client_name": "e2e-priv"})
    try:
        with websockets.connect(fake_server.ws_base + "/ws", open_timeout=10) as ws:
            ws.send(json.dumps({"op": "auth", "session_token": j["session_token"]}))
            ws.send(json.dumps({"op": "subscribe", "topic": "world.state"}))
            end = time.time() + 5
            while time.time() < end:
                m = json.loads(ws.recv(timeout=3))
                assert m["topic"] != "world.state"
                if m["topic"] == "error":
                    assert m["code"] == "forbidden"
                    break
            else:
                pytest.fail("no refusal")
    finally:
        fake_server.json("DELETE", f"/client/leave/{j['session_id']}")
