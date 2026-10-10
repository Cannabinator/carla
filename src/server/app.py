"""FastAPI app for the persistent CARLA server: REST routes, the WebSocket hub and the web pages (src/web)."""

import sys
from pathlib import Path

from fastapi import FastAPI
from fastapi.middleware.gzip import GZipMiddleware
from fastapi.responses import HTMLResponse, RedirectResponse
from fastapi.staticfiles import StaticFiles

import src.server.admin_routes as _admin_mod
import src.server.api_routes as _api_mod
import src.server.client_routes as _client_mod
import src.server.hub_routes as _hub_mod
import src.server.lidar_routes as _lidar_mod

WEB_DIR = Path(__file__).parent.parent / "web"

app = FastAPI(title="CARLA V2X + LiDAR Server")
app.add_middleware(GZipMiddleware, minimum_size=1000)      # the map geometry is large; WebSockets are not affected
app.include_router(_admin_mod.router)
app.include_router(_client_mod.router)
app.include_router(_lidar_mod.router)
app.include_router(_hub_mod.router)
app.include_router(_api_mod.router)

app.mount("/static", StaticFiles(directory=str(WEB_DIR / "static")), name="static")


def set_persistent_world(world) -> None:
    """Register the PersistentCARLAServer with the admin and client routes."""
    _admin_mod.set_persistent_world(world)
    _client_mod.set_persistent_world(world)
    _lidar_mod.set_persistent_world(world)
    _hub_mod.set_persistent_world(world)
    _api_mod.set_persistent_world(world)


# /admin/connect creates a world after startup and propagates it through this module.
_admin_mod.set_server_module(sys.modules[__name__])


def _page(name: str) -> HTMLResponse:
    page = WEB_DIR / name
    if page.exists():
        return HTMLResponse(content=page.read_text())
    return HTMLResponse(content=f"<h1>{name} not found</h1>", status_code=404)


@app.get("/", include_in_schema=False)
async def root():
    # Docker healthcheck requests "/" and expects 200.
    return _page("index.html")


@app.get("/observer", include_in_schema=False)
async def observer_page():
    return _page("observer.html")


@app.get("/vehicle", include_in_schema=False)
async def vehicle_page():
    return _page("vehicle.html")


@app.get("/admin", include_in_schema=False)
async def admin_panel():
    return _page("admin.html")


@app.get("/client", include_in_schema=False)
async def client_join():
    # the old join page is gone; keep the URL working for devices that have it bookmarked
    return RedirectResponse("/vehicle")


def run_server(host: str = "0.0.0.0", port: int = 8000):
    import uvicorn

    uvicorn.run(app, host=host, port=port)
