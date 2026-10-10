# pyright: reportOptionalMemberAccess=false
"""Read-only status/config/map endpoints and the admin DENM injection (REFACTOR_HANDOFF 5.7.2, decision D11).

Public ones follow ``observer_public``: when it is false they need the admin token like everything else.
"""

import logging
import sys
from typing import Optional, Union

from fastapi import APIRouter, Header, HTTPException, Request, Response
from fastapi.responses import JSONResponse
from pydantic import BaseModel, Field, field_validator

from src.server.admin_routes import check_admin_token
from src.server.route_utils import on_sim_thread
from src.server.world_config import WorldConfig
from src.v2x.messages.denm import DENMCauseCode

logger = logging.getLogger(__name__)

router = APIRouter(tags=["api"])

_world = None


def set_persistent_world(world) -> None:
    global _world
    _world = world


def _read_gate(x_admin_token: Optional[str]) -> None:
    """Observer reads are allowed unless the server switched them off; the admin token always works."""
    if x_admin_token:
        check_admin_token(x_admin_token)
        return
    if _world is not None and not _world.observer_public:
        raise HTTPException(status_code=403, detail="observer access is disabled; send X-Admin-Token")


def _ready():
    if _world is None:
        raise HTTPException(status_code=503, detail="CARLA server not started")
    if not _world._running:
        raise HTTPException(status_code=503, detail="Simulation not active yet")
    return _world


@router.get("/api/status", summary="Server status (public)")
async def get_status(x_admin_token: Optional[str] = Header(default=None)):
    """Is CARLA connected, which map, which frame; ``connected: false`` before /admin/connect."""
    if _world is None or not _world._running:
        return {"connected": False}
    _read_gate(x_admin_token)
    status = _world.get_status()
    run = _world.get_run_status()
    return {
        "connected": True, "map": status["map"], "frame": status["frame"],
        "sim_time": run.sim_time if run else None,
        "real_time_factor": run.data.get("real_time_factor") if run else None,
        "fixed_delta_seconds": _world.fixed_delta_seconds, "v2x_enabled": status["v2x_enabled"],
        "active_sessions": status["active_sessions"], "run_active": False,     # recording: Phase 7
        "observer_public": _world.observer_public,
        "fake": bool(getattr(sys.modules.get("carla"), "IS_FAKE", False)),   # a synthetic world, not CARLA
    }


@router.get("/api/config", summary="The resolved configuration of the running world")
async def get_config(x_admin_token: Optional[str] = Header(default=None)):
    world = _ready()
    _read_gate(x_admin_token)
    return world.config.model_dump()


@router.get("/api/config/schema", summary="JSON schema of the configuration (for forms)")
async def get_config_schema(x_admin_token: Optional[str] = Header(default=None)):
    _read_gate(x_admin_token)
    return WorldConfig.model_json_schema()


@router.get("/api/map/geometry", summary="Road network: lane polylines and junction boxes")
async def get_map_geometry(request: Request, x_admin_token: Optional[str] = Header(default=None)):
    world = _ready()
    _read_gate(x_admin_token)
    geom = world.cached_map_geometry()
    if geom is None:
        geom = await on_sim_thread(world, world.get_map_geometry)
    etag = f'"{geom["version"]}"'
    if request.headers.get("if-none-match") == etag:
        return Response(status_code=304, headers={"ETag": etag})
    return JSONResponse(geom, headers={"ETag": etag, "Cache-Control": "no-cache"})


# ── DENM injection ────────────────────────────────────────────────────────────────────────────────

class DenmRequest(BaseModel):
    x: float = Field(description="CARLA world x of the event, metres")
    y: float = Field(description="CARLA world y of the event, metres")
    z: float = 0.0
    cause_code: Union[int, str] = Field(
        default="ACCIDENT", description="ETSI cause code number or name: " + ", ".join(c.name for c in DENMCauseCode))
    subcause_code: int = Field(default=0, ge=0, le=255)
    relevance_distance_m: float = Field(default=150.0, gt=0, le=2000)
    information_quality: int = Field(default=7, ge=0, le=7)
    station_id: Optional[int] = Field(default=None, description="Originating station (a V2X vehicle's actor id or "
                                      "-intersection_id of an RSU); null = a fixed roadside transmitter at the event")
    duration_s: Optional[float] = Field(default=None, gt=0, description="Simulation seconds until it is cancelled; "
                                        "null = until cancelled")

    @field_validator("cause_code")
    @classmethod
    def _cause(cls, v):
        try:
            return int(DENMCauseCode[v.upper()] if isinstance(v, str) else DENMCauseCode(v))
        except (KeyError, ValueError):
            raise ValueError(f"unknown DENM cause code {v!r}")


@router.post("/api/v2x/denm", status_code=201, summary="Inject a DENM at a map position (admin)")
async def inject_denm(req: DenmRequest, x_admin_token: str = Header(...)):
    check_admin_token(x_admin_token)
    world = _ready()
    try:
        return await on_sim_thread(world, lambda: world.inject_denm(
            req.x, req.y, req.z, req.cause_code, req.subcause_code, req.relevance_distance_m,
            req.information_quality, req.station_id, req.duration_s))
    except ValueError as exc:
        raise HTTPException(status_code=422, detail=str(exc))


@router.get("/api/v2x/denm", summary="DENMs in the last V2X update")
async def list_denm(x_admin_token: Optional[str] = Header(default=None)):
    world = _ready()
    _read_gate(x_admin_token)
    return {"denms": world.list_denm()}


@router.delete("/api/v2x/denm/{station_id}/{action_id}", summary="Cancel a DENM (admin)")
async def cancel_denm(station_id: int, action_id: int, x_admin_token: str = Header(...)):
    check_admin_token(x_admin_token)
    world = _ready()
    if not await on_sim_thread(world, lambda: world.cancel_denm(station_id, action_id)):
        raise HTTPException(status_code=404, detail="no such DENM")
    return {"cancelled": True}
