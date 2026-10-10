"""The multiplexed WebSocket hub: auth, permissions per topic, filters, rates, latest-wins delivery."""

import asyncio
import json
import time

import pytest

pytest.importorskip("httpx", reason="fastapi TestClient needs httpx")

from fastapi.testclient import TestClient  # noqa: E402

import src.server.app as app_mod
import src.server.hub_routes as hub_routes
from src.server.topics import TOPICS
from src.server.ws_hub import Hub
from tests.contract import ws_helpers
from tests.contract.fake_world import FakeWorld
from tests.contract.v2x_samples import sample_frame

TOKEN = "test-token"


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
    w.publish_world_state([{"id": 101, "kind": "vehicle", "x": 0.0, "y": 0.0}, {"id": 300, "kind": "pedestrian", "x": 8.0, "y": 8.0}])
    w.publish_v2x(sample_frame())
    app_mod.set_persistent_world(w)
    return w


def connect(client):
    hub_done = lambda: hub_routes._hub is None or hub_routes._hub.connections == 0  # noqa: E731
    return ws_helpers.connect(client, "/ws", hub_done)


def join(client, name="car"):
    r = client.post("/client/join", json={"client_name": name})
    assert r.status_code == 200, r.text
    return r.json()


def send(ws, **msg):
    ws.send_text(json.dumps(msg))


def recv(ws, topic, limit=30):
    """Next message on ``topic`` (anything else seen on the way is returned too)."""
    seen = []
    for _ in range(limit):
        m = ws_helpers.recv_json(ws)
        seen.append(m)
        if m["topic"] == topic:
            return m, seen
    raise AssertionError(f"no {topic!r} message, got {[s['topic'] for s in seen]}")


def error_of(ws, limit=10):
    m, _ = recv(ws, "error", limit)
    return m


def auth(ws, **cred):
    send(ws, op="auth", **cred)
    return recv(ws, "auth")[0]


# ── basics ────────────────────────────────────────────────────────────────────────────────────────

def test_topic_catalogue_is_public(client, world):
    r = client.get("/api/topics").json()
    names = {t["name"] for t in r["topics"]}
    assert names == set(TOPICS) and r["endpoint"] == "/ws"
    links = next(t for t in r["topics"] if t["name"] == "v2x.links")
    assert links["filter_required"] is True and links["ground_truth"] is True


def test_envelope_and_ping(client, world):
    with connect(client) as ws:
        send(ws, op="ping")
        assert recv(ws, "pong")[0]["data"]["t"] > 0
        send(ws, op="subscribe", topic="world.state", max_hz=10)
        ack, _ = recv(ws, "subscribed")
        assert ack["data"] == {"topic": "world.state", "max_hz": 10.0, "filter": {}, "role": "observer"}
        m, _ = recv(ws, "world.state")
        assert set(m) == {"topic", "frame", "sim_time", "seq", "data"}
        assert m["frame"] == 42 and m["seq"] == 1 and [s["id"] for s in m["data"]["stations"]] == [101, 300]


def test_bad_messages_do_not_close_the_connection(client, world):
    with connect(client) as ws:
        ws.send_text("not json")
        assert error_of(ws)["code"] == "bad_message"
        ws.send_text("[1,2]")
        assert error_of(ws)["code"] == "bad_message"
        ws.send_text("x" * 20000)
        assert error_of(ws)["code"] == "bad_message"
        send(ws, op="teleport")
        assert error_of(ws)["code"] == "bad_request"
        send(ws, op="subscribe", topic="nope")
        assert error_of(ws)["code"] == "unknown_topic"
        send(ws, op="subscribe", topic="world.state", filter=[1])
        assert error_of(ws)["code"] == "bad_filter"
        for hz in (0, -1, "fast", True):
            send(ws, op="subscribe", topic="world.state", max_hz=hz)
            assert error_of(ws)["code"] == "bad_request"
        send(ws, op="ping")
        recv(ws, "pong")


def test_max_hz_is_capped_by_the_topic(client, world):
    with connect(client) as ws:
        send(ws, op="subscribe", topic="v2x.stats", max_hz=500)
        assert recv(ws, "subscribed")[0]["data"]["max_hz"] == TOPICS["v2x.stats"].max_hz


def test_world_state_kind_filter(client, world):
    with connect(client) as ws:
        send(ws, op="subscribe", topic="world.state", filter={"kinds": ["pedestrian"]})
        m, _ = recv(ws, "world.state")
        assert [s["id"] for s in m["data"]["stations"]] == [300]
        send(ws, op="subscribe", topic="world.state", filter={"kinds": ["boat"]})
        assert error_of(ws)["code"] == "bad_filter"


# ── auth ──────────────────────────────────────────────────────────────────────────────────────────

def test_bad_admin_token_closes_with_4401(client, world):
    with connect(client) as ws:
        send(ws, op="auth", admin_token="wrong")
        assert error_of(ws)["code"] == "auth_failed"
        with pytest.raises(Exception):
            for _ in range(5):
                ws_helpers.recv_json(ws)


def test_bad_session_token_is_refused_and_a_session_id_is_not_a_token(client, world):
    j = join(client)
    for bad in ("nope", j["session_id"]):
        with connect(client) as ws:
            send(ws, op="auth", session_token=bad)
            assert error_of(ws)["code"] == "auth_failed"


def test_auth_without_credentials_is_a_request_error(client, world):
    with connect(client) as ws:
        send(ws, op="auth")
        assert error_of(ws)["code"] == "bad_request"


def test_admin_may_subscribe_to_every_topic(client, world):
    with connect(client) as ws:
        assert auth(ws, admin_token=TOKEN)["data"]["role"] == "admin"
        for name in TOPICS:
            if name == "session.self":
                continue                                     # needs a session
            filt = {"v2x.links": {"outcome": "all"}, "v2x.inbox": {"station_id": 102}}.get(name, {})
            send(ws, op="subscribe", topic=name, filter=filt)
            assert recv(ws, "subscribed")[0]["data"]["topic"] == name, name
        send(ws, op="subscribe", topic="session.self")
        assert error_of(ws)["code"] == "forbidden"           # admin has no vehicle


GROUND_TRUTH = [n for n, t in TOPICS.items() if t.ground_truth]


def test_the_ground_truth_set_is_what_the_spec_says():
    assert set(GROUND_TRUTH) == {"world.state", "v2x.frame", "v2x.links", "v2x.stats", "events", "run.status"}


@pytest.mark.parametrize("name", GROUND_TRUTH)
def test_every_ground_truth_topic_is_refused_for_a_vehicle(client, world, name):
    j = join(client)
    with connect(client) as ws:
        assert auth(ws, session_token=j["session_token"])["data"]["role"] == "vehicle"
        send(ws, op="subscribe", topic=name, filter={"outcome": "all"} if name == "v2x.links" else {})
        e = error_of(ws)
        assert e["code"] == "forbidden" and e["request"]["topic"] == name


def test_vehicle_gets_its_own_telemetry_and_inbox_only(client, world):
    j = join(client)
    sid = j["actor_id"]
    with connect(client) as ws:
        a = auth(ws, session_token=j["session_token"])
        assert a["data"]["station_id"] == sid and a["data"]["client_name"] == "car"
        send(ws, op="subscribe", topic="session.self")
        m, _ = recv(ws, "session.self")
        assert m["data"]["telemetry"]["actor_id"] == sid and m["data"]["remaining_seconds"] is None
        # the inbox of another station is ground truth of that station
        send(ws, op="subscribe", topic="v2x.inbox", filter={"station_id": 102})
        assert error_of(ws)["code"] == "forbidden"
        # without a filter it defaults to the vehicle's own station
        send(ws, op="subscribe", topic="v2x.inbox")
        ack = recv(ws, "subscribed")[0]
        assert ack["data"]["filter"] == {"station_id": sid}
        send(ws, op="subscribe", topic="lidar.sensors")
        assert recv(ws, "lidar.sensors")[0]["data"]["budget"]["max_sensors"] == 2


def test_vehicle_inbox_shows_what_it_received(client, world):
    j = join(client)
    world.publish_v2x(_frame_with_vehicle(j["actor_id"]))
    with connect(client) as ws:
        auth(ws, session_token=j["session_token"])
        send(ws, op="subscribe", topic="v2x.inbox")
        m, _ = recv(ws, "v2x.inbox")
        assert [x["msg_type"] for x in m["data"]["messages"]] == ["BSM"] and m["data"]["messages"][0]["tx_id"] == 101
        assert m["data"]["messages"][0]["link"]["delivered"] is True
        assert m["data"]["messages"][0]["message"]["type"] == "BSMCore"


def _frame_with_vehicle(actor_id):
    """sample_frame where the joined vehicle (actor_id) is the receiver of 101's BSM."""
    from dataclasses import replace
    from src.v2x.frame import ReceivedMessage, StationState, frozen_map
    f = sample_frame(frame_no=11)
    bsm = f.sent[101][0]
    link = f.links[0]._replace(rx_id=actor_id)
    return replace(f, stations=frozen_map({**f.stations, actor_id: StationState(actor_id, "OBU", 1, 1, 0)}),
                   inbox=frozen_map({actor_id: (ReceivedMessage(101, "BSM", bsm, link),)}))


def test_observer_cannot_get_session_self(client, world):
    with connect(client) as ws:
        send(ws, op="subscribe", topic="session.self")
        assert error_of(ws)["code"] == "forbidden"


def test_observer_access_can_be_disabled(client, world):
    world.observer_public = False
    j = join(client)
    with connect(client) as ws:
        send(ws, op="subscribe", topic="world.state")
        e = error_of(ws)
        assert e["code"] == "forbidden" and "disabled" in e["message"]
        auth(ws, admin_token=TOKEN)
        send(ws, op="subscribe", topic="world.state")
        recv(ws, "world.state")
    with connect(client) as ws:
        auth(ws, session_token=j["session_token"])
        send(ws, op="subscribe", topic="session.self")
        recv(ws, "session.self")


def test_auth_after_subscribing_drops_subscriptions_the_new_role_may_not_have(client, world):
    j = join(client)
    with connect(client) as ws:
        send(ws, op="subscribe", topic="world.state")
        recv(ws, "subscribed")
        send(ws, op="auth", session_token=j["session_token"])
        e = error_of(ws)
        assert e["code"] == "forbidden" and e["request"]["topic"] == "world.state"
        recv(ws, "auth")


def test_vehicle_connection_counts_as_session_presence(client, world):
    j = join(client)
    s = world.sessions.get_session(j["session_id"])
    assert s.ws_connections == 0
    with connect(client) as ws:
        auth(ws, session_token=j["session_token"])
        assert s.ws_connections == 1
    for _ in range(50):
        if s.ws_connections == 0:
            break
        time.sleep(0.02)
    assert s.ws_connections == 0


# ── v2x.links ─────────────────────────────────────────────────────────────────────────────────────

def test_links_without_a_filter_are_refused_for_everyone(client, world):
    for cred in ({}, {"admin_token": TOKEN}):
        with connect(client) as ws:
            if cred:
                auth(ws, **cred)
            for filt in (None, {}, {"role": "tx"}):
                msg = {"op": "subscribe", "topic": "v2x.links"}
                if filt is not None:
                    msg["filter"] = filt
                ws.send_text(json.dumps(msg))
                assert error_of(ws)["code"] == "filter_required"


def test_links_filters(client, world):
    def rows(**f):
        with connect(client) as ws:
            send(ws, op="subscribe", topic="v2x.links", filter=f)
            m, _ = recv(ws, "v2x.links")
            d = m["data"]
            return {**d, "links": [dict(zip(d["columns"], r)) for r in d["rows"]]}

    assert [(l["tx_id"], l["rx_id"]) for l in rows(outcome="dropped")["links"]] == [(102, 101)]
    assert len(rows(outcome="delivered")["links"]) == 3
    assert [l["msg_type"] for l in rows(msg_types=["DENM"])["links"]] == ["DENM"]
    assert {l["link_type"] for l in rows(link_types=["I2V", "P2V"])["links"]} == {"I2V", "P2V"}
    assert len(rows(station_ids=[102])["links"]) == 2                         # as tx and as rx
    assert [(l["tx_id"], l["rx_id"]) for l in rows(station_ids=[102], role="tx")["links"]] == [(102, 101)]
    assert [(l["tx_id"], l["rx_id"]) for l in rows(station_ids=[102], role="rx")["links"]] == [(101, 102)]
    d = rows(outcome="all")
    assert d["matching"] == 4 and d["total_links"] == 4 and d["truncated"] is False
    link = d["links"][0]
    assert {"tx_id", "rx_id", "rx_power_dbm", "p_total", "drop_cause", "los", "frame"} <= set(link)


def test_links_are_capped_and_say_so(client, world):
    world.v2x_links_max_rows = 3
    world.publish_v2x(sample_frame(frame_no=12, extra_links=10))
    with connect(client) as ws:
        send(ws, op="subscribe", topic="v2x.links", filter={"outcome": "all"})
        d = recv(ws, "v2x.links")[0]["data"]
        assert len(d["rows"]) == 3 and d["truncated"] is True and d["matching"] == 14 and d["max_rows"] == 3


@pytest.mark.parametrize("bad", [{"station_ids": "101"}, {"station_ids": [1.5]}, {"msg_types": ["XYZ"]},
                                 {"link_types": ["V2X"]}, {"outcome": "some"}, {"role": "both", "outcome": "all"},
                                 {"foo": 1, "outcome": "all"}, {"station_ids": list(range(501))}])
def test_bad_links_filters(client, world, bad):
    with connect(client) as ws:
        send(ws, op="subscribe", topic="v2x.links", filter=bad)
        assert error_of(ws)["code"] == "bad_filter"


def test_inbox_needs_a_station(client, world):
    with connect(client) as ws:
        send(ws, op="subscribe", topic="v2x.inbox")
        assert error_of(ws)["code"] == "filter_required"
        send(ws, op="subscribe", topic="v2x.inbox", filter={"station_id": 102})
        m = recv(ws, "v2x.inbox")[0]
        assert [x["tx_id"] for x in m["data"]["messages"]] == [101] and m["data"]["threats"][0]["other_id"] == 101
        assert [x["type"] for x in m["data"]["sent"]] == ["BSMCore"] and m["data"]["sent"][0]["vehicle_id"] == 102


# ── other topics ──────────────────────────────────────────────────────────────────────────────────

def test_v2x_frame_and_stats_topics(client, world):
    with connect(client) as ws:
        send(ws, op="subscribe", topic="v2x.frame")
        d = recv(ws, "v2x.frame")[0]["data"]
        assert {s["station_id"] for s in d["stations"]} == {101, 102, -1, 300} and d["denms"][0]["action_id"] == 7
        send(ws, op="subscribe", topic="v2x.stats")
        s = recv(ws, "v2x.stats")[0]["data"]
        assert s["window"]["updates"] == 1 and s["run"]["updates"] == 1 and "V2V/LOS" in s["window"]["prr_by_distance"]
        send(ws, op="subscribe", topic="run.status")
        assert recv(ws, "run.status")[0]["data"]["frame"] == 42


def test_lidar_sensors_topic_pushes_on_change_only(client, world):
    with connect(client) as ws:
        send(ws, op="subscribe", topic="lidar.sensors", max_hz=2)
        first = recv(ws, "lidar.sensors")[0]
        assert first["data"]["sensors"] == []
        r = client.post("/api/lidar/sensors", json={"actor_id": 9}, headers={"X-Admin-Token": TOKEN})
        assert r.status_code == 202
        second = recv(ws, "lidar.sensors")[0]
        assert second["seq"] > first["seq"] and second["data"]["sensors"][0]["actor_id"] == 9


def test_events_topic_starts_with_the_tail_then_streams_new_entries(client, world):
    world.events.add("a")
    world.events.add("b")
    with connect(client) as ws:
        send(ws, op="subscribe", topic="events", max_hz=2)
        first = recv(ws, "events")[0]
        assert [e["kind"] for e in first["data"]["events"]] == ["a", "b"]
        world.events.add("c", detail=1)
        second = recv(ws, "events")[0]
        assert [e["kind"] for e in second["data"]["events"]] == ["c"] and second["seq"] == 3
        assert second["data"]["events"][0]["data"] == {"detail": 1}


def test_unchanged_state_is_not_resent(client, world):
    with connect(client) as ws:
        send(ws, op="subscribe", topic="world.state", max_hz=10)
        recv(ws, "world.state")
        time.sleep(0.4)
        world.publish_world_state([{"id": 5, "kind": "vehicle"}])
        m = recv(ws, "world.state")[0]
        assert m["seq"] == 2                                   # nothing was sent for the unchanged seq 1 in between


def test_nan_in_published_data_becomes_an_error_not_invalid_json(client, world):
    world.publish_world_state([{"id": 1, "kind": "vehicle", "x": float("nan")}])
    with connect(client) as ws:
        send(ws, op="subscribe", topic="world.state")
        e = error_of(ws)
        assert e["code"] == "internal"


def test_no_world_gives_a_not_ready_error(client):
    with connect(client) as ws:
        assert ws_helpers.recv_json(ws)["code"] == "not_ready"


# ── delivery: latest wins, slow clients ───────────────────────────────────────────────────────────

def test_slow_client_gets_the_latest_state_and_seq_gaps_show_the_skips():
    world = FakeWorld()

    async def scenario():
        hub = Hub(world)
        ws = ws_helpers.SlowSocket(send_delay=0.25)
        task = asyncio.create_task(hub.serve(ws))
        ws.feed(op="subscribe", topic="world.state", max_hz=10)
        stop = time.monotonic() + 1.6
        n = 0
        while time.monotonic() < stop:           # the simulation publishes at 50 Hz regardless of the reader
            n += 1
            world.publish_world_state([{"id": n}])
            await asyncio.sleep(0.02)
        await asyncio.sleep(0.4)
        task.cancel()
        await asyncio.gather(task, return_exceptions=True)
        return ws, n

    ws, published = asyncio.run(scenario())
    got = [m for m in ws.sent if m["topic"] == "world.state"]
    seqs = [m["seq"] for m in got]
    assert seqs == sorted(seqs) and len(seqs) < published / 4, (len(seqs), published)
    assert max(seqs) >= published - 15, "the client must end up with a recent state, not a backlog"
    assert any(b - a > 1 for a, b in zip(seqs, seqs[1:])), "seq gaps tell the client it skipped updates"
    assert got[-1]["data"]["stations"][0]["id"] == seqs[-1], "payload belongs to the seq it is labelled with"


def test_a_stalled_client_is_disconnected_instead_of_piling_up():
    world = FakeWorld()
    world.publish_world_state([{"id": 1}])

    async def scenario():
        hub = Hub(world, send_timeout_s=0.2)
        ws = ws_helpers.SlowSocket(send_delay=5.0)
        task = asyncio.create_task(hub.serve(ws))
        ws.feed(op="subscribe", topic="world.state")
        await asyncio.wait_for(task, timeout=3)
        return ws, hub

    ws, hub = asyncio.run(scenario())
    assert ws.closed_code == 1013 and hub.connections == 0


def test_admin_can_see_who_is_subscribed_to_what(client, world):
    j = join(client)
    with connect(client) as ws, connect(client) as ws2:
        auth(ws, session_token=j["session_token"])
        send(ws, op="subscribe", topic="session.self")
        recv(ws, "session.self")
        send(ws2, op="subscribe", topic="world.state")
        recv(ws2, "world.state")
        r = client.get("/api/hub/connections", headers={"X-Admin-Token": TOKEN}).json()
        by_role = {c["role"]: c for c in r["connections"]}
        assert set(by_role) == {"vehicle", "observer"}
        assert set(by_role["vehicle"]["topics"]) == {"session.self"} and by_role["vehicle"]["client_name"] == "car"
        assert set(by_role["observer"]["topics"]) == {"world.state"}
        assert r["messages_sent"] > 0 and by_role["observer"]["bytes_sent"] > 0
    assert client.get("/api/hub/connections").status_code == 403
    assert client.get("/api/hub/connections", headers={"X-Admin-Token": "bad"}).status_code == 403
