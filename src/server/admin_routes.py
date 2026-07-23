"""
Admin-only FastAPI router.

All endpoints require the ``X-Admin-Token`` header to match the value of
the ``ADMIN_TOKEN`` environment variable set at server startup.

Mount on the main app with::

    from src.server.admin_routes import router as admin_router
    app.include_router(admin_router)
"""

import os
import logging
import threading
from typing import Optional

from fastapi import APIRouter, HTTPException, Header, Depends
from pydantic import BaseModel, Field

logger = logging.getLogger(__name__)

router = APIRouter(prefix="/admin", tags=["admin"])

# Injected once at startup by start_server.py
_world = None
_server_module = None  # reference back to lidar.server for set_persistent_world propagation


def set_persistent_world(world) -> None:
    global _world
    _world = world


def set_server_module(mod) -> None:
    global _server_module
    _server_module = mod


# ------------------------------------------------------------------ #
# Auth dependency                                                     #
# ------------------------------------------------------------------ #

def _require_admin(x_admin_token: str = Header(..., alias="X-Admin-Token")):
    """Raise 403 if the token does not match ADMIN_TOKEN env var."""
    expected = os.environ.get("ADMIN_TOKEN", "")
    if not expected:
        raise HTTPException(status_code=500, detail="ADMIN_TOKEN is not configured on the server")
    if x_admin_token != expected:
        raise HTTPException(status_code=403, detail="Invalid admin token")


# ------------------------------------------------------------------ #
# Request models                                                      #
# ------------------------------------------------------------------ #

class TrafficRequest(BaseModel):
    count: int = Field(default=20, ge=1, le=200)

class PedestrianRequest(BaseModel):
    count: int = Field(default=15, ge=1, le=100)

class ConnectRequest(BaseModel):
    carla_host: str = Field(default="192.168.1.110")
    carla_port: int = Field(default=2000, ge=1, le=65535)
    v2v_enabled: bool = True
    v2v_range: float = Field(default=75.0, ge=10.0, le=500.0)
    tm_port: int = Field(default=8001, ge=1024, le=65535)


# ------------------------------------------------------------------ #
# Endpoints                                                           #
# ------------------------------------------------------------------ #

@router.get("/status", summary="World status", dependencies=[Depends(_require_admin)])
async def admin_status():
    """Return world-level status — works even before connect (world=None)."""
    if _world is None:
        return {
            "running": False,
            "connected": False,
            "frame": 0,
            "elapsed_seconds": 0,
            "background_vehicles": 0,
            "pedestrians": 0,
            "active_sessions": 0,
            "client_vehicles": 0,
            "v2v_enabled": False,
            "map": None,
        }
    return {**_world.get_status(), "connected": True}


@router.post("/connect", summary="Connect to CARLA and start world", dependencies=[Depends(_require_admin)])
async def connect_world(req: ConnectRequest):
    """
    Start (or restart) the persistent CARLA world.

    Call this from the admin panel to connect to a CARLA server at any IP.
    Safe to call again if the world is already running — stops the old one first.
    """
    global _world

    def _start():
        global _world
        from src.server.persistent_world import PersistentCARLAServer
        # Stop previous world if any
        if _world is not None:
            try:
                _world.stop()
            except Exception as e:
                logger.warning("Error stopping previous world: %s", e)

        world = PersistentCARLAServer(
            host=req.carla_host,
            port=req.carla_port,
            v2v_enabled=req.v2v_enabled,
            v2v_range=req.v2v_range,
            tm_port=req.tm_port,
        )
        world.start()
        _world = world

        # Propagate to client routes and lidar server
        if _server_module is not None:
            _server_module.set_persistent_world(world)

        logger.info("Persistent world started via admin /connect (%s:%s)", req.carla_host, req.carla_port)

    threading.Thread(target=_start, daemon=True, name="admin-connect").start()
    return {
        "ok": True,
        "message": f"Connecting to CARLA at {req.carla_host}:{req.carla_port} (async)…",
    }


@router.get("/sessions", summary="All client sessions", dependencies=[Depends(_require_admin)])
async def list_sessions():
    """Return all sessions (active and ended). Returns [] when world is not started."""
    if _world is None:
        return []
    return [s.to_dict() for s in _world.sessions.all_sessions()]


@router.delete("/sessions/{session_id}", summary="Kick a client", dependencies=[Depends(_require_admin)])
async def kick_session(session_id: str):
    """Force-end a session and despawn the client vehicle."""
    if _world is None:
        raise HTTPException(status_code=404, detail="Session not found — world not started")
    session = _world.sessions.get_session(session_id)
    if session is None:
        raise HTTPException(status_code=404, detail="Session not found")
    _world.despawn_client_vehicle(session_id)
    _world.sessions.end_session(session_id)
    return {"ok": True, "kicked": session_id}


@router.post("/traffic", summary="Spawn background traffic", dependencies=[Depends(_require_admin)])
async def spawn_traffic(req: TrafficRequest):
    """Spawn autopilot background vehicles."""
    _assert_ready()
    _assert_running()
    spawned = _world.spawn_background_traffic(req.count)
    return {"requested": req.count, "spawned": spawned}


@router.delete("/traffic", summary="Clear background traffic", dependencies=[Depends(_require_admin)])
async def clear_traffic():
    """Destroy all admin-spawned background vehicles. Returns 0 when world not started."""
    if _world is None:
        return {"removed": 0}
    removed = _world.despawn_background_traffic()
    return {"removed": removed}


@router.post("/pedestrians", summary="Spawn pedestrians", dependencies=[Depends(_require_admin)])
async def spawn_pedestrians(req: PedestrianRequest):
    """Spawn AI-controlled pedestrians."""
    _assert_ready()
    _assert_running()
    spawned = _world.spawn_pedestrians(req.count)
    return {"requested": req.count, "spawned": spawned}


@router.delete("/pedestrians", summary="Clear pedestrians", dependencies=[Depends(_require_admin)])
async def clear_pedestrians():
    """Destroy all admin-spawned pedestrians. Returns 0 when world not started."""
    if _world is None:
        return {"removed": 0}
    removed = _world.despawn_pedestrians()
    return {"removed": removed}


@router.post("/stop", summary="Stop the CARLA server", dependencies=[Depends(_require_admin)])
async def stop_world():
    """Gracefully stop the persistent world (tick loop + actor cleanup)."""
    if _world is None:
        return {"ok": False, "message": "World not running"}
    threading.Thread(target=_world.stop, daemon=True, name="admin-stop").start()
    return {"ok": True, "message": "Shutdown initiated"}


# ------------------------------------------------------------------ #
# Internal helpers                                                    #
# ------------------------------------------------------------------ #

def _assert_ready():
    if _world is None:
        raise HTTPException(status_code=503, detail="Persistent world not initialised")


def _assert_running():
    if not _world._running:
        raise HTTPException(status_code=503, detail="CARLA simulation is not running")
