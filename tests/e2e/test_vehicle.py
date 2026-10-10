import json
import time

import pytest

pytest.importorskip("playwright")
websockets = pytest.importorskip("websockets.sync.client")

ALLOWED_VEHICLE_TOPICS = {"session.self", "v2x.inbox", "lidar.sensors"}


def join(new_page, fake_server, name="e2e-vehicle", lidar=False, **kw):
    pg = new_page(**kw)
    pg.goto(fake_server.base + "/vehicle?debug=1")
    pg.fill("#j-name", name)
    if lidar:
        pg.check("#j-lidar")
    pg.click("#join-btn")
    pg.wait_for_selector("#me dd", timeout=20000)
    return pg


def leave_if_joined(pg, fake_server):
    sid = pg.evaluate("(window.carlaDebug && window.carlaDebug.session && window.carlaDebug.session.id) || null")
    if sid:
        fake_server.json("DELETE", f"/client/leave/{sid}")


def sensors(fake_server):
    return fake_server.json("GET", "/api/lidar/sensors")[1]


def test_join_shows_my_vehicle_and_my_radio_with_frames(new_page, fake_server):
    pg = join(new_page, fake_server)
    try:
        assert "e2e-vehicle" in pg.inner_text("#me")
        pg.wait_for_selector("#radio h3:has-text('Inbox')", timeout=20000)
        assert "My telemetry" in pg.inner_text("#radio")
        assert pg.get_attribute("#map", "data-mode") == "perspective"
        assert "frame" in pg.inner_text("#radio-stamp")
        assert pg.get_attribute("#lidar-badge", "data-state") == "off"
        pg.assert_clean()
    finally:
        leave_if_joined(pg, fake_server)


def test_the_vehicle_page_subscribes_to_no_ground_truth_topic_as_seen_by_the_server(new_page, fake_server):
    pg = join(new_page, fake_server, name="e2e-privacy")
    try:
        pg.wait_for_selector("#radio h3:has-text('Inbox')", timeout=20000)
        status, r = fake_server.json("GET", "/api/hub/connections", admin=True)
        assert status == 200
        mine = [c for c in r["connections"] if c["client_name"] == "e2e-privacy"]
        assert len(mine) == 1 and mine[0]["role"] == "vehicle"
        assert set(mine[0]["topics"]) <= ALLOWED_VEHICLE_TOPICS, mine[0]["topics"]
        assert set(mine[0]["topics"]) >= {"session.self", "v2x.inbox"}
        # and the map must not have been drawn from ground truth: there is no world.state in this page's store
        assert pg.evaluate("window.carlaDebug.store.get('world.state')") is None
    finally:
        leave_if_joined(pg, fake_server)


def test_lidar_request_reconfigure_and_leave_follow_the_state_machine(new_page, fake_server):
    pg = join(new_page, fake_server, name="e2e-lidar")
    try:
        pg.evaluate("""() => { window.__states = []; const b = document.getElementById('lidar-badge');
            new MutationObserver(() => window.__states.push(b.dataset.state)).observe(b, {attributes: true, attributeFilter: ['data-state']}); }""")
        pg.fill("#lidar-points_per_second", "100000")
        pg.click("#lidar-request")
        pg.wait_for_function("(() => { const c = document.querySelector('#lidar-view canvas'); return c && Number(c.dataset.displayed) > 500; })()", timeout=30000)
        assert pg.get_attribute("#lidar-badge", "data-state") == "active"
        seen = pg.evaluate("window.__states")
        order = ["off", "requested", "spawning", "warming_up", "active"]
        idx = [order.index(s) for s in seen]
        assert idx == sorted(idx) and seen[-1] == "active", seen            # only forward steps, ending active
        # the server's event log has the complete, ordered state machine with frames
        with websockets.connect(fake_server.ws_base + "/ws", open_timeout=10) as ws:
            ws.send(json.dumps({"op": "auth", "admin_token": fake_server.token}))
            ws.send(json.dumps({"op": "subscribe", "topic": "events"}))
            kinds = []
            end = time.time() + 10
            while time.time() < end and "lidar_active" not in kinds:
                m = json.loads(ws.recv(timeout=5))
                if m["topic"] == "events":
                    kinds += [e["kind"] for e in m["data"]["events"] if e["kind"].startswith("lidar_")]
        want = ["lidar_requested", "lidar_spawning", "lidar_warming_up", "lidar_active"]
        assert [k for k in kinds if k in want][-4:] == want, kinds

        # reconfigure 32 -> 16 channels: the sensor is respawned and the view keeps rendering
        pg.fill("#lidar-channels", "16")
        pg.click("#lidar-apply")
        pg.wait_for_function("window.carlaDebug.store.get('lidar.sensors').data.sensors.some(s => s.config.channels === 16 && s.state === 'active')", timeout=20000)
        f0 = pg.get_attribute("#lidar-view canvas", "data-frame")
        pg.wait_for_function(f"document.querySelector('#lidar-view canvas').dataset.frame !== '{f0}' && Number(document.querySelector('#lidar-view canvas').dataset.displayed) > 100", timeout=20000)
        assert len(sensors(fake_server)["sensors"]) == 1

        # leave: the sensor and the vehicle are gone
        pg.click("#leave")
        pg.wait_for_selector("#join-panel:not([hidden])")
        deadline = time.time() + 5
        while time.time() < deadline and sensors(fake_server)["sensors"]:
            time.sleep(0.2)
        assert sensors(fake_server)["sensors"] == [] and sensors(fake_server)["budget"]["used_pps"] == 0
        pg.assert_clean()
    finally:
        leave_if_joined(pg, fake_server)


def test_request_lidar_on_join(new_page, fake_server):
    pg = join(new_page, fake_server, name="e2e-join-lidar", lidar=True)
    try:
        pg.wait_for_function("document.querySelector('#lidar-badge').dataset.state === 'active'", timeout=30000)
        pg.wait_for_selector("#lidar-view canvas")
        assert not pg.is_visible("#lidar-request")
    finally:
        leave_if_joined(pg, fake_server)


def test_over_budget_request_explains_itself(new_page, fake_server):
    pg = join(new_page, fake_server, name="e2e-budget")
    try:
        pg.fill("#lidar-points_per_second", "10000000")             # more than the budget (2,000,000 points/s)
        pg.click("#lidar-request")
        pg.wait_for_selector(".toast-error:has-text('budget exceeded')", timeout=10000)
        assert pg.get_attribute("#lidar-badge", "data-state") == "off"
        assert sensors(fake_server)["sensors"] == []
    finally:
        leave_if_joined(pg, fake_server)


def test_reload_keeps_the_session_and_a_forgotten_session_returns_to_the_join_form(new_page, fake_server):
    pg = join(new_page, fake_server, name="e2e-reload")
    try:
        pg.reload()
        pg.wait_for_selector("#me dd", timeout=20000)
        assert "e2e-reload" in pg.inner_text("#me")
        # the server forgets the session (an admin kicks it)
        status, sessions = fake_server.json("GET", "/admin/sessions", admin=True)
        sid = next(s["session_id"] for s in sessions if s["client_name"] == "e2e-reload")
        fake_server.json("DELETE", f"/admin/sessions/{sid}", admin=True)
        pg.reload()
        pg.wait_for_selector(".toast:has-text('has ended on the server')", timeout=20000)
        assert pg.is_visible("#join-panel") and pg.is_hidden("#app-right")
        assert pg.evaluate("sessionStorage.getItem('carla.session')") is None
    finally:
        leave_if_joined(pg, fake_server)


def test_connection_loss_shows_the_grace_countdown_and_recovers(new_page, fake_server):
    pg = join(new_page, fake_server, name="e2e-grace")
    try:
        pg.wait_for_selector("#radio h3:has-text('Inbox')", timeout=20000)
        pg.evaluate("(() => { window.carlaDebug.hub.WebSocketImpl = class { constructor() { throw new Error('offline'); } }; })()")
        pg.evaluate("window.carlaDebug.hub.sock.close()")
        pg.wait_for_selector("#conn-banner:not([hidden])", timeout=10000)
        text = pg.inner_text("#conn-banner")
        assert "keeps your vehicle" in text and "s while this page reconnects" in text
        pg.evaluate("(() => { window.carlaDebug.hub.WebSocketImpl = WebSocket; })()")
        pg.wait_for_selector("#conn-banner", state="hidden", timeout=20000)
        assert pg.get_attribute("#conn", "data-state") == "open"
    finally:
        leave_if_joined(pg, fake_server)
