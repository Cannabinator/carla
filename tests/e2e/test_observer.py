import json
import time

import pytest

pytest.importorskip("playwright")


def open_observer(new_page, fake_server, **kw):
    pg = new_page(**kw)
    pg.goto(fake_server.base + "/observer?debug=1")
    pg.wait_for_function("document.querySelector('#map').dataset.stations && Number(document.querySelector('#map').dataset.stations) > 10",
                         timeout=20000)
    return pg


def test_observer_loads_without_errors_and_shows_the_world(new_page, fake_server):
    pg = open_observer(new_page, fake_server)
    assert pg.get_attribute("#map", "data-mode") == "truth"
    assert pg.inner_text("#conn").startswith("open")
    pg.wait_for_selector("#health dd")
    assert "real-time factor" in pg.inner_text("#health")
    pg.assert_clean()


def test_map_handedness_in_the_real_page(new_page, fake_server):
    """A station's screen position follows the not-mirrored projection: larger world y is lower on the screen."""
    pg = open_observer(new_page, fake_server)
    pts = pg.evaluate("""() => { const m = window.carlaDebug.map; const w = m.store.get('world.state').data.stations;
        return w.map(s => ({id: s.id, x: s.x, y: s.y, sp: m.screenPosOf(s.id)})).filter(s => s.sp); }""")
    assert len(pts) > 10
    a = min(pts, key=lambda p: p["y"])
    b = max(pts, key=lambda p: p["y"])
    assert a["sp"][1] < b["sp"][1], "larger CARLA y must be lower on the screen"
    c = min(pts, key=lambda p: p["x"])
    d = max(pts, key=lambda p: p["x"])
    assert c["sp"][0] < d["sp"][0]


def test_clicking_a_station_shows_its_inbox(new_page, fake_server):
    pg = open_observer(new_page, fake_server)
    sp = pg.evaluate("""() => { const m = window.carlaDebug.map; const w = m.store.get('world.state').data.stations
        .filter(s => s.kind === 'vehicle'); const s = w[0]; return {id: s.id, p: m.screenPosOf(s.id)}; }""")
    box = pg.locator("#map").bounding_box()
    pg.mouse.click(box["x"] + sp["p"][0], box["y"] + sp["p"][1])
    pg.wait_for_function(f"window.carlaDebug.selected === {sp['id']}")
    pg.wait_for_selector("#inspector h3:has-text('Inbox')", timeout=15000)
    text = pg.inner_text("#inspector")
    assert f"#{sp['id']}" in text and "Ground truth" in text and "Last transmitted" in text
    pg.assert_clean()


def test_inbox_table_lists_received_messages_with_link_fields(new_page, fake_server):
    pg = open_observer(new_page, fake_server)
    # pick a vehicle that has neighbours: the first station with received > 0 in the V2X summary
    sid = pg.evaluate("""async () => { for (let i = 0; i < 60; i++) {
        const v = window.carlaDebug.app.store.get('v2x.frame'); if (v) { const s = v.data.stations.find(s => s.kind === 'OBU' && s.received > 0); if (s) return s.station_id; }
        await new Promise(r => setTimeout(r, 250)); } return null; }""")
    assert sid is not None
    pg.evaluate(f"window.carlaDebug.select({sid})")
    pg.wait_for_selector("#inspector table tbody tr", timeout=15000)
    header = pg.inner_text("#inspector table thead").lower()
    for col in ("tx", "link", "msg", "age", "dist", "los", "rx power", "margin", "p_total"):
        assert col in header
    pg.assert_clean()


def test_link_log_filter_and_link_details(new_page, fake_server):
    pg = open_observer(new_page, fake_server)
    sid = pg.evaluate("""async () => { for (let i = 0; i < 60; i++) {
        const v = window.carlaDebug.app.store.get('v2x.frame'); if (v) { const s = v.data.stations.find(s => s.kind === 'OBU' && s.received > 0); if (s) return s.station_id; }
        await new Promise(r => setTimeout(r, 250)); } return null; }""")
    pg.evaluate(f"window.carlaDebug.select({sid})")
    pg.wait_for_selector("#log table tbody tr", timeout=15000)
    n = pg.locator("#log table tbody tr").count()
    assert n > 0
    pg.locator("#log table tbody tr").first.click()
    pg.wait_for_selector("#link-details dt:has-text('rx_power_dbm')")
    fields = {t.strip() for t in pg.locator("#link-details dt").all_inner_texts()}
    assert {"frame", "tx_id", "rx_id", "msg_type", "link_type", "distance_m", "los", "path_loss_db", "shadowing_db",
            "rx_power_dbm", "rx_margin_db", "p_channel", "p_mac", "p_total", "delivered", "drop_cause"} <= fields
    assert "filter" in pg.inner_text("#log-filter-text")


def test_without_a_filter_the_log_asks_for_one(new_page, fake_server):
    pg = open_observer(new_page, fake_server)
    assert "select a station or set a filter" in pg.inner_text("#log-filter-text")
    pg.select_option("#f-outcome", "dropped")
    pg.wait_for_function("document.querySelector('#log-filter-text').textContent.includes('outcome')")


def test_perspective_mode_shows_only_received_data_with_ages(new_page, fake_server):
    pg = open_observer(new_page, fake_server)
    sid = pg.evaluate("""async () => { for (let i = 0; i < 60; i++) {
        const v = window.carlaDebug.app.store.get('v2x.frame'); if (v) { const s = v.data.stations.find(s => s.kind === 'OBU' && s.received > 3); if (s) return s.station_id; }
        await new Promise(r => setTimeout(r, 250)); } return null; }""")
    assert sid is not None
    pg.evaluate(f"window.carlaDebug.select({sid})")
    pg.select_option("#mode", "perspective")
    pg.wait_for_function("document.querySelector('#map').dataset.mode === 'perspective' && Number(document.querySelector('#map').dataset.stations) >= 2", timeout=15000)
    n_truth = pg.evaluate("window.carlaDebug.app.store.get('world.state').data.stations.length")
    n_persp = int(pg.get_attribute("#map", "data-stations"))
    assert 2 <= n_persp < n_truth + 12, (n_persp, n_truth)
    assert "PERSPECTIVE" in pg.evaluate("document.querySelector('#map').dataset.mode").upper()


def test_channel_charts_render_and_window_switch_works(new_page, fake_server):
    pg = open_observer(new_page, fake_server)
    pg.wait_for_selector("#chart-prr canvas", timeout=20000)
    assert pg.locator("#chart-margin canvas").count() >= 1
    assert "V2V" in pg.inner_text("#chart-prr .legend")
    pg.select_option("#stats-window", "run")
    pg.wait_for_function("document.querySelector('#chart-prr .stamp').textContent.includes('V2X updates')")
    assert "awareness" in pg.inner_text("#chart-prr .stamp")


def test_dark_theme_loads_cleanly(new_page, fake_server):
    pg = open_observer(new_page, fake_server, dark=True)
    bg = pg.evaluate("getComputedStyle(document.body).backgroundColor")
    assert bg != "rgb(246, 247, 249)"
    pg.assert_clean()


def test_connection_state_is_visible_and_recovers_after_a_dropped_socket(new_page, fake_server):
    pg = open_observer(new_page, fake_server)
    assert pg.get_attribute("#conn", "data-state") == "open"
    pg.evaluate("window.carlaDebug.app.hub.sock.close()")        # drop the socket under the page's feet
    pg.wait_for_function("document.querySelector('#conn').dataset.state === 'reconnecting'", timeout=5000)
    pg.wait_for_function("document.querySelector('#conn').dataset.state === 'open'", timeout=15000)
    frame = pg.evaluate("window.carlaDebug.app.store.get('world.state').frame")
    time.sleep(1.0)
    assert pg.evaluate("window.carlaDebug.app.store.get('world.state').frame") > frame, "subscriptions must resume"
    _ = json
