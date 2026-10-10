import pytest

pytest.importorskip("playwright")


def test_index_page_shows_status_and_the_fake_banner(new_page, fake_server):
    pg = new_page()
    pg.goto(fake_server.base + "/")
    pg.wait_for_selector("#status dd:has-text('connected')")
    assert "FAKE world" in pg.inner_text("#status")
    assert pg.inner_text(".fake-banner").startswith("FAKE WORLD")
    for href in ("/observer", "/vehicle", "/admin"):
        assert pg.locator(f"a[href='{href}']").count() >= 1
    assert "FakeTown" in pg.inner_text("#status")
    pg.assert_clean()


def test_static_files_and_vendored_libraries_are_served(fake_server):
    for path in ("/static/css/app.css", "/static/js/coords.js", "/static/js/ws.js",
                 "/static/vendor/three/three.module.min.js", "/static/vendor/three/OrbitControls.js",
                 "/static/vendor/uplot/uPlot.esm.js", "/static/vendor/uplot/uPlot.min.css", "/static/vendor/VERSIONS.md"):
        status, raw = fake_server.request("GET", path)
        assert status == 200 and raw, path
    assert fake_server.request("GET", "/static/../../etc/passwd")[0] in (404, 400)
