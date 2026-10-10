import json
import time

import pytest

pytest.importorskip("playwright")
websockets = pytest.importorskip("websockets.sync.client")


def open_admin(new_page, fake_server):
    pg = new_page(viewport=(1500, 1000))
    pg.goto(fake_server.base + "/admin?debug=1")
    pg.fill("#token", fake_server.token)
    pg.click("#login-btn")
    pg.wait_for_selector("#admin:not([hidden])", timeout=15000)
    pg.wait_for_function("document.querySelector('#map').dataset.stations && Number(document.querySelector('#map').dataset.stations) > 5", timeout=20000)
    return pg


def test_login_refuses_a_wrong_token_and_remembers_a_right_one_for_the_tab(new_page, fake_server):
    pg = new_page()
    pg.goto(fake_server.base + "/admin")
    assert pg.is_visible("#login") and pg.is_hidden("#admin")
    pg.fill("#token", "wrong")
    pg.click("#login-btn")
    pg.wait_for_selector(".toast-error:has-text('refused')")
    assert pg.is_hidden("#admin")
    pg.fill("#token", fake_server.token)
    pg.click("#login-btn")
    pg.wait_for_selector("#admin:not([hidden])")
    pg.reload()
    pg.wait_for_selector("#admin:not([hidden])", timeout=15000)               # sessionStorage keeps it for this tab
    assert "token" not in pg.url
    # the failed attempt above made the browser log a 403: that is the expected outcome here
    pg.assert_clean(allow=("403",))


def test_admin_page_shows_every_card_and_runs_clean(new_page, fake_server):
    pg = open_admin(new_page, fake_server)
    for card in ("connection", "run", "population", "config", "rsus", "denm", "sessions", "lidar", "hub"):
        assert pg.is_visible(f"#card-{card}"), card
    pg.wait_for_function("document.querySelector('#conn-state').textContent.includes('running')")
    assert pg.is_enabled("#run-start") and "Record" in pg.inner_text("#card-run")
    pg.wait_for_selector("#rsu-table tbody tr", timeout=15000)
    assert pg.locator("#rsu-table tbody tr").count() == 9
    pg.assert_clean(allow=("403",))


def test_population_card_spawns_and_clears(new_page, fake_server):
    pg = open_admin(new_page, fake_server)

    def veh():
        return fake_server.json("GET", "/admin/status", admin=True)[1]["background_vehicles"]
    before = veh()
    pg.fill("#pop-veh", "4")
    pg.click("#pop-veh-add")
    pg.wait_for_selector(".toast-ok:has-text('spawned')")
    assert veh() == before + 4
    pg.wait_for_function(f"document.querySelector('#pop-counts').textContent.startsWith('{before + 4} vehicles')", timeout=10000)
    pg.click("#pop-veh-clear")
    pg.wait_for_function("document.querySelector('#pop-counts').textContent.startsWith('0 vehicles')", timeout=10000)
    pg.fill("#pop-veh", str(before))
    pg.click("#pop-veh-add")                                                  # put the world back for the other tests
    pg.wait_for_function("Number(document.querySelector('#pop-counts').textContent.split(' ')[0]) >= 8", timeout=10000)   # spawn points may be taken


def test_config_card_applies_hot_fields_with_the_frame_and_disables_restart_fields(new_page, fake_server):
    pg = open_admin(new_page, fake_server)
    pg.wait_for_selector("#cfg-form .field", timeout=15000)
    assert pg.is_disabled("#cfg-fixed_delta_seconds")
    assert pg.locator("#cfg-form .badge-restart").count() > 5 and pg.locator("#cfg-form .badge-hot").count() >= 5
    assert not pg.is_disabled("#cfg-awareness_range_m")
    pg.fill("#cfg-awareness_range_m", "120")
    pg.click("#cfg-apply")
    pg.wait_for_function("document.querySelector('#cfg-result').textContent.includes('applied at frame')", timeout=10000)
    text = pg.inner_text("#cfg-result")
    assert "awareness_range_m: 75 → 120" in text and "next V2X update" in text
    assert fake_server.json("GET", "/api/config")[1]["awareness_range_m"] == 120.0
    pg.evaluate("window.carlaDebug.map.requestRender()")
    # the change is an event with the frame
    with websockets.connect(fake_server.ws_base + "/ws", open_timeout=10) as ws:
        ws.send(json.dumps({"op": "subscribe", "topic": "events"}))
        got = []
        end = time.time() + 10
        while time.time() < end and not any(e["kind"] == "config_changed" for e in got):
            m = json.loads(ws.recv(timeout=5))
            if m["topic"] == "events":
                got += m["data"]["events"]
    ev = next(e for e in got if e["kind"] == "config_changed" and e["data"]["field"] == "awareness_range_m")
    assert ev["data"]["old"] == 75.0 and ev["data"]["new"] == 120.0 and ev["data"]["applied_at_frame"] > 0
    # emptying a nullable field means "no awareness filter"; an invalid value is refused with a reason
    pg.fill("#cfg-awareness_range_m", "")
    pg.click("#cfg-apply")
    pg.wait_for_function("document.querySelector('#cfg-result').textContent.includes('null')", timeout=10000)
    assert fake_server.json("GET", "/api/config")[1]["awareness_range_m"] is None
    pg.fill("#cfg-awareness_range_m", "75")
    pg.click("#cfg-apply")
    pg.wait_for_function("document.querySelector('#cfg-result').textContent.includes('75')", timeout=10000)
    pg.fill("#cfg-lidar_max_sensors", "999")
    pg.click("#cfg-apply")
    pg.wait_for_selector(".toast-error:has-text('at most 64')")
    assert fake_server.json("GET", "/api/config")[1]["awareness_range_m"] == 75.0


def test_rsu_card_toggles_and_sets_power(new_page, fake_server):
    pg = open_admin(new_page, fake_server)
    pg.wait_for_selector("#rsu-table tbody tr", timeout=15000)
    row = pg.locator("#rsu-table tbody tr").first
    sid = row.get_attribute("data-station")
    row.locator("input[type=checkbox]").uncheck()
    pg.wait_for_function(f"window.carlaDebug.app.store.get('world.state').data.rsus.find(r => r.station_id === {sid}).enabled === false", timeout=10000)
    pg.wait_for_function(f"!window.carlaDebug.app.store.get('v2x.frame').data.stations.some(s => s.station_id === {sid})", timeout=15000)
    row = pg.locator("#rsu-table tbody tr").first
    row.locator("input[type=number]").fill("27.5")
    row.locator("button:has-text('Set')").click()
    pg.wait_for_function(f"window.carlaDebug.app.store.get('world.state').data.rsus.find(r => r.station_id === {sid}).tx_power_dbm === 27.5", timeout=10000)
    pg.locator("#rsu-table tbody tr").first.locator("input[type=checkbox]").check()
    pg.wait_for_function(f"window.carlaDebug.app.store.get('v2x.frame').data.stations.some(s => s.station_id === {sid})", timeout=15000)
    fake_server.json("PATCH", f"/api/v2x/rsus/{sid}", {"tx_power_dbm": 33.0}, admin=True)


def test_denm_injection_by_clicking_the_map_and_cancelling(new_page, fake_server):
    pg = open_admin(new_page, fake_server)
    pg.select_option("#dn-cause", "10")
    pg.fill("#dn-rad", "200")
    pg.click("#dn-pick")
    box = pg.locator("#map").bounding_box()
    sp = pg.evaluate("(() => { const m = window.carlaDebug.map; const [x, y] = [100, 100]; return [(x - m.vp.cx) * m.vp.scale + m.vp.width / 2, (y - m.vp.cy) * m.vp.scale + m.vp.height / 2]; })()")
    pg.mouse.click(box["x"] + sp[0], box["y"] + sp[1])
    pg.wait_for_function("document.querySelector('#dn-pos').textContent.startsWith('x=')")
    pos = pg.inner_text("#dn-pos")
    assert abs(float(pos.split("x=")[1].split(" m")[0]) - 100) < 3 and abs(float(pos.split("y=")[1].split(" m")[0]) - 100) < 3
    pg.click("#dn-inject")
    pg.wait_for_selector(".toast-ok:has-text('injected')")
    pg.wait_for_selector("#dn-active table tbody tr", timeout=10000)
    assert "10" in pg.inner_text("#dn-active")
    # the map draws it
    pg.wait_for_function("window.carlaDebug.app.store.get('v2x.frame').data.denms.some(d => d.cause_code === 'OBSTACLE_ON_ROAD' && d.relevance_distance === 200)")
    mine = pg.locator("#dn-active tbody tr", has_text="OBSTACLE_ON_ROAD")
    mine.locator("button:has-text('Cancel')").click()
    pg.wait_for_function("!window.carlaDebug.app.store.get('v2x.frame').data.denms.some(d => d.cause_code === 'OBSTACLE_ON_ROAD')", timeout=15000)


def test_sessions_card_lists_and_kicks(new_page, fake_server):
    status, j = fake_server.json("POST", "/client/join", {"client_name": "e2e-kickme"})
    assert status == 200
    try:
        pg = open_admin(new_page, fake_server)
        pg.wait_for_selector(f"#sess-table tr[data-session='{j['session_id']}']", timeout=15000)
        assert "e2e-kickme" in pg.inner_text("#sess-table")
        pg.once("dialog", lambda d: d.accept())
        pg.locator(f"#sess-table tr[data-session='{j['session_id']}'] button:has-text('Kick')").click()
        pg.wait_for_selector(f"#sess-table tr[data-session='{j['session_id']}']", state="detached", timeout=15000)
        status, again = fake_server.json("POST", "/client/join", {"client_name": "e2e-after-kick"})
        assert status == 200
        fake_server.json("DELETE", f"/client/leave/{again['session_id']}")
    finally:
        fake_server.json("DELETE", f"/client/leave/{j['session_id']}")


def test_lidar_card_attach_pin_detach_and_budget_message(new_page, fake_server):
    pg = open_admin(new_page, fake_server)
    pg.wait_for_function("document.querySelector('#lid-veh').options.length > 0", timeout=15000)
    pg.click("#card-lidar summary")
    pg.fill("#lidar-points_per_second", "100000")
    pg.check("#lid-pin")
    pg.click("#lid-attach")
    pg.wait_for_selector("#lid-table tbody tr", timeout=15000)
    sid = pg.locator("#lid-table tbody tr").first.get_attribute("data-sensor")
    pg.wait_for_function("document.querySelector('#lid-table tbody tr .state-active')", timeout=15000)
    assert "100,000" in pg.inner_text("#lid-table")
    assert "points/s" in pg.inner_text("#lid-budget")
    assert fake_server.json("GET", "/api/lidar/sensors")[1]["sensors"][0]["pinned"] is True
    pg.locator("#lid-table tbody tr input[type=checkbox]").uncheck()
    pg.wait_for_function("window.carlaDebug.app.store.get('lidar.sensors').data.sensors[0].pinned === false", timeout=10000)
    # over budget: the explanation carries the numbers
    pg.fill("#lidar-points_per_second", "10000000")
    pg.click("#lid-attach")
    pg.wait_for_selector(".toast-error:has-text('budget exceeded')")
    # view and detach
    pg.locator("#lid-table tbody tr button:has-text('View')").click()
    pg.wait_for_selector("#lidar canvas", timeout=20000)
    pg.locator("#lid-table tbody tr button:has-text('Detach')").click()
    pg.wait_for_selector("#lid-table tbody tr", state="detached", timeout=15000)
    assert sid and fake_server.json("GET", "/api/lidar/sensors")[1]["sensors"] == []


def test_admin_creates_a_sensor_from_the_inspector(new_page, fake_server):
    pg = open_admin(new_page, fake_server)
    vid = pg.evaluate("window.carlaDebug.app.store.get('world.state').data.stations.find(s => s.kind === 'vehicle' && s.role === 'background').id")
    pg.evaluate(f"window.carlaDebug.select({vid})")
    pg.wait_for_selector("#show-lidar:has-text('Create LiDAR')", timeout=15000)
    pg.click("#show-lidar")
    pg.wait_for_selector("#lidar canvas", timeout=30000)
    pg.wait_for_function("(() => { const c = document.querySelector('#lidar canvas'); return c && Number(c.dataset.displayed) > 100; })()", timeout=30000)
    for s in fake_server.json("GET", "/api/lidar/sensors")[1]["sensors"]:
        fake_server.json("DELETE", f"/api/lidar/sensors/{s['sensor_id']}", admin=True)


def test_hub_card_shows_clients_with_their_topics(new_page, fake_server):
    pg = open_admin(new_page, fake_server)
    pg.wait_for_function("document.querySelector('#hub-table tbody').textContent.includes('admin')", timeout=15000)
    assert "world.state" in pg.inner_text("#hub-table")
    pg.wait_for_function("document.querySelector('#hub-rate').textContent.includes('outbound')", timeout=15000)
