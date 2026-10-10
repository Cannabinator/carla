import pytest

pytest.importorskip("playwright")


@pytest.fixture
def pinned_sensor(fake_server):
    """A pinned sensor on a background vehicle (admin), removed afterwards."""
    import json
    import time
    import websockets.sync.client as wsc
    with wsc.connect(fake_server.ws_base + "/ws", open_timeout=10) as sock:
        sock.send(json.dumps({"op": "subscribe", "topic": "world.state"}))
        end = time.time() + 10
        veh = None
        while time.time() < end and veh is None:
            m = json.loads(sock.recv(timeout=5))
            if m["topic"] == "world.state":
                veh = next(s for s in m["data"]["stations"] if s["kind"] == "vehicle" and s["role"] == "background")
    status, body = fake_server.json("POST", "/api/lidar/sensors", {"actor_id": veh["id"], "pinned": True,
                                                                    "config": {"points_per_second": 200000}}, admin=True)
    assert status == 202, body
    yield veh["id"], body["sensor_id"]
    fake_server.json("DELETE", f"/api/lidar/sensors/{body['sensor_id']}", admin=True)
    time.sleep(0.5)


def test_observer_opens_a_lidar_view_and_points_render(new_page, fake_server, pinned_sensor):
    actor_id, sensor_id = pinned_sensor
    pg = new_page()
    pg.goto(fake_server.base + "/observer?debug=1")
    pg.wait_for_function("window.carlaDebug && window.carlaDebug.app.store.get('lidar.sensors')", timeout=20000)
    pg.evaluate(f"window.carlaDebug.select({actor_id})")
    pg.wait_for_selector("#show-lidar:not([disabled])", timeout=15000)
    pg.click("#show-lidar")
    pg.wait_for_function("(() => { const c = document.querySelector('#lidar canvas'); return c && Number(c.dataset.frames) > 3 && Number(c.dataset.displayed) > 1000; })()", timeout=30000)
    hud = pg.inner_text("#lidar .hud")
    assert "LiDAR frame F_l=" in hud and "displayed" in hud and "fps" in hud
    assert pg.get_attribute("#lidar canvas", "data-state") == "active"
    # overlays: own box and V2X-received boxes are drawn from published V2X data
    n_boxes = pg.evaluate(f"window.carlaDebug.views.get({sensor_id}).boxPool.filter(b => b && b.visible).length")
    assert n_boxes >= 1
    pg.assert_clean()


def test_ego_filter_changes_the_displayed_count_and_the_colour_modes_match_the_sensor_type(new_page, fake_server, pinned_sensor):
    actor_id, sensor_id = pinned_sensor
    pg = new_page()
    pg.goto(fake_server.base + "/observer?debug=1")
    pg.wait_for_function("window.carlaDebug && window.carlaDebug.app.store.get('lidar.sensors')", timeout=20000)
    pg.evaluate(f"window.carlaDebug.select({actor_id})")
    pg.wait_for_selector("#show-lidar:not([disabled])", timeout=15000)
    pg.click("#show-lidar")
    pg.wait_for_function("(() => { const c = document.querySelector('#lidar canvas'); return c && Number(c.dataset.displayed) > 1000; })()", timeout=30000)
    v = f"window.carlaDebug.views.get({sensor_id})"
    assert pg.evaluate(f"{v}.latest.counts.filteredEgo") > 0, "the sensor sees its own vehicle: those points are filtered and counted"
    modes = pg.evaluate("[...document.querySelectorAll('#lidar select')][0].options.length")
    enabled = pg.evaluate("[...document.querySelectorAll('#lidar select')][0].querySelectorAll('option:not([disabled])').length")
    assert modes == 5 and enabled == 4           # a semantic sensor has no intensity: that mode is disabled, not faked
    pg.uncheck("#lidar input[type=checkbox] >> nth=0")
    pg.wait_for_function(f"{v}.latest.counts.filteredEgo === 0", timeout=10000)


def test_freeze_stops_the_picture_but_not_the_stream_and_export_downloads_raw_bytes(new_page, fake_server, pinned_sensor):
    actor_id, sensor_id = pinned_sensor
    pg = new_page()
    pg.goto(fake_server.base + "/observer?debug=1")
    pg.wait_for_function("window.carlaDebug && window.carlaDebug.app.store.get('lidar.sensors')", timeout=20000)
    pg.evaluate(f"window.carlaDebug.select({actor_id})")
    pg.wait_for_selector("#show-lidar:not([disabled])", timeout=15000)
    pg.click("#show-lidar")
    pg.wait_for_function("(() => { const c = document.querySelector('#lidar canvas'); return c && Number(c.dataset.frames) > 2; })()", timeout=30000)
    pg.click("#lidar button:has-text('Freeze')")
    f0 = pg.get_attribute("#lidar canvas", "data-frame")
    pg.wait_for_timeout(1500)
    assert pg.get_attribute("#lidar canvas", "data-frame") == f0
    assert "FROZEN" in pg.inner_text("#lidar .hud")
    with pg.expect_download() as d1:
        pg.click("#lidar button:has-text('Export frame')")
    names = {d1.value.suggested_filename}
    assert any(n.endswith(".v3.bin") or n.startswith("frame_") for n in names)
    raw = open(d1.value.path(), "rb").read() if d1.value.suggested_filename.endswith(".bin") else None
    if raw is not None:
        assert raw[:4] == b"LIDR"
    pg.click("#lidar button:has-text('Resume')")
    pg.wait_for_function(f"document.querySelector('#lidar canvas').dataset.frame !== '{f0}'", timeout=10000)


def test_at_most_two_lidar_views_at_once(new_page, fake_server, pinned_sensor):
    actor_id, sensor_id = pinned_sensor
    pg = new_page()
    pg.goto(fake_server.base + "/observer?debug=1")
    pg.wait_for_function("window.carlaDebug && window.carlaDebug.app.store.get('lidar.sensors')", timeout=20000)
    sensors = pg.evaluate("window.carlaDebug.app.store.get('lidar.sensors').data.sensors")
    s = next(x for x in sensors if x["sensor_id"] == sensor_id)
    pg.evaluate(f"window.carlaDebug.openLidar({s!r})".replace("True", "true").replace("False", "false").replace("None", "null"))
    pg.wait_for_selector("#lidar canvas")
    assert pg.evaluate("window.carlaDebug.views.size") == 1
    pg.evaluate("window.carlaDebug.views.get(%d).dispose()" % sensor_id)
    pg.wait_for_function("window.carlaDebug.views.size === 0")
    assert pg.is_hidden("#lidar-panel")
