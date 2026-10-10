import json
import time

import pytest

pytest.importorskip("playwright")


def open_admin(new_page, fake_server):
    pg = new_page(viewport=(1500, 1000))
    pg.goto(fake_server.base + "/admin?debug=1")
    pg.fill("#token", fake_server.token)
    pg.click("#login-btn")
    pg.wait_for_selector("#admin:not([hidden])", timeout=15000)
    return pg


def test_rest_run_lifecycle_on_the_real_server(fake_server):
    status, started = fake_server.json("POST", "/api/runs", {"name": "e2e rest", "record": ["links", "stations", "lidar_meta"]}, admin=True)
    assert status == 201, started
    try:
        status, again = fake_server.json("POST", "/api/runs", {"name": "second"}, admin=True)
        assert status == 409 and again["detail"]["error"] == "run_active"
        assert fake_server.json("GET", "/api/status")[1]["run_active"] is True
        deadline = time.time() + 15
        while time.time() < deadline:
            active = fake_server.json("GET", "/api/runs", admin=True)[1]["active"]
            if active and active["v2x_updates"] >= 3:
                break
            time.sleep(0.3)
        assert active["v2x_updates"] >= 3 and active["bytes_written"] > 0 and active["error"] is None
    finally:
        status, stopped = fake_server.json("POST", "/api/runs/current/stop", admin=True)
    assert status == 200 and stopped["status"] == "finished"
    status, m = fake_server.json("GET", f"/api/runs/{started['run_id']}/manifest", admin=True)
    assert status == 200 and m["status"] == "finished" and m["carla"]["fake"] is True
    assert m["files"]["v2x_links.jsonl"]["lines"] > 0 and m["files"]["stations.jsonl"]["lines"] == m["frames_recorded"]
    assert fake_server.json("GET", "/api/runs/..%2F..%2Fetc/manifest", admin=True)[0] == 404
    assert fake_server.json("GET", "/api/runs/does-not-exist/manifest", admin=True)[0] == 404
    assert any(r["run_id"] == started["run_id"] for r in fake_server.json("GET", "/api/runs", admin=True)[1]["runs"])


def test_admin_run_card_starts_shows_progress_and_stops(new_page, fake_server):
    pg = open_admin(new_page, fake_server)
    assert pg.is_enabled("#run-start") and pg.is_disabled("#run-stop")
    pg.fill("#run-name", "ui run")
    pg.click("#run-start")
    pg.wait_for_selector(".toast-ok:has-text('started')")
    pg.wait_for_function("document.querySelector('#run-badge').textContent === 'recording'")
    assert pg.is_disabled("#run-start") and pg.is_enabled("#run-stop")
    pg.wait_for_function("document.querySelector('#run-status').textContent.includes('V2X updates written')", timeout=10000)
    assert "writer queue" in pg.inner_text("#run-status") and "disk free" in pg.inner_text("#run-status")
    pg.click("#run-stop")
    pg.wait_for_selector(".toast-ok:has-text('finished')")
    pg.wait_for_function("document.querySelector('#run-badge').textContent === 'no run'")
    pg.wait_for_selector("#run-table tbody tr:has-text('ui-run')", timeout=10000)
    pg.locator("#run-table tbody tr", has_text="ui-run").first.locator("button:has-text('Manifest')").click()
    pg.wait_for_selector("#run-manifest:not([hidden])")
    m = json.loads(pg.inner_text("#run-manifest"))
    assert m["name"] == "ui run" and m["status"] == "finished" and m["recorded"] == ["lidar_meta", "links", "messages", "stations"]
    pg.assert_clean(allow=("403",))
