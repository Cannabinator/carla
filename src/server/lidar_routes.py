# pyright: reportOptionalMemberAccess=false
"""LiDAR API (REFACTOR_HANDOFF 5.5): sensors exist only on request, viewers are separate from sensors.

Who may do what (D9, D12): the *admin* (``X-Admin-Token``) may create a sensor on any actor, pin it and record
its raw frames; a *vehicle* session (``X-Session-Id``) only on its own vehicle; anyone else is an *observer*
and may only read the sensor list and view active sensors. Phase 5 replaces this header handling by the roles
of the WebSocket hub; the permission rules themselves live in ``LidarManager``.
"""

import asyncio
import json
import logging
import uuid
from typing import Any, Dict, Optional

from fastapi import APIRouter, Header, HTTPException, WebSocket, WebSocketDisconnect
from fastapi.responses import JSONResponse
from pydantic import BaseModel

from src.lidar.config import LidarConfig, lidar_schema
from src.lidar.sensor import (ADMIN, LidarBudgetExceeded, LidarError, LidarForbidden, LidarNotFound, Principal,
                              SensorState)
from src.lidar.stream import StreamProfile, quantize_profile
from src.server.admin_routes import check_admin_token
from src.server.route_utils import on_sim_thread

logger = logging.getLogger(__name__)

router = APIRouter(tags=["lidar"])

_world = None

_END_STATES = (SensorState.DESTROYED.value, SensorState.ERROR.value)


def set_persistent_world(world) -> None:
    global _world
    _world = world


def _ready():
    if _world is None:
        raise HTTPException(status_code=503, detail="CARLA server not started")
    if not _world._running or _world.lidar is None:
        raise HTTPException(status_code=503, detail="Simulation not active yet")
    return _world


def principal_from_headers(x_admin_token: Optional[str], x_session_id: Optional[str]) -> Principal:
    if x_admin_token:
        check_admin_token(x_admin_token)
        return ADMIN
    if x_session_id:
        session = _world.sessions.get_session(x_session_id) if _world is not None else None
        if session is None or not session.is_active:
            raise HTTPException(status_code=404, detail="Session not found or ended")
        return Principal("vehicle", session.session_id, session.vehicle_actor_id)
    return Principal("observer")


def _lidar_errors(exc: LidarError) -> JSONResponse:
    if isinstance(exc, LidarBudgetExceeded):
        return JSONResponse(status_code=409, content=exc.detail)
    if isinstance(exc, LidarForbidden):
        return JSONResponse(status_code=403, content={"error": "forbidden", "message": str(exc)})
    if isinstance(exc, LidarNotFound):
        return JSONResponse(status_code=404, content={"error": "not_found", "message": str(exc)})
    return JSONResponse(status_code=409, content={"error": "lidar_state", "message": str(exc)})


class SensorRequest(BaseModel):
    actor_id: Optional[int] = None          # a vehicle may omit it (own actor)
    config: Optional[LidarConfig] = None
    record_raw: bool = False
    pinned: bool = False


async def request_sensor(world, principal: Principal, actor_id: Optional[int], config: Optional[LidarConfig],
                         record_raw: bool = False, pinned: bool = False) -> int:
    """Create a sensor from the simulation thread; raises LidarError for the caller to map."""
    if actor_id is None:
        actor_id = principal.actor_id
    if actor_id is None:
        raise LidarForbidden("actor_id is required")
    return await on_sim_thread(world, lambda: world.lidar_request(
        principal, actor_id, config, record_raw=record_raw, pinned=pinned))


def _accepted(world, sensor_id: int, config: LidarConfig) -> JSONResponse:
    return JSONResponse(status_code=202, content={
        "sensor_id": sensor_id, "state": SensorState.REQUESTED.value, "ws_url": f"/ws/lidar/{sensor_id}",
        "warnings": config.warnings(world.fixed_delta_seconds)})


@router.get("/api/lidar/schema", summary="What can be configured, per sensor type, plus the semantic tag table")
async def get_schema():
    world = _ready()
    lim = world.lidar.limits
    return lidar_schema({"max_sensors": lim.max_sensors, "max_points_per_second": lim.max_points_per_second,
                         "keep_alive_s": lim.keep_alive_s, "fixed_delta_seconds": world.fixed_delta_seconds})


@router.get("/api/lidar/sensors", summary="Sensors and the budget")
async def list_sensors():
    world = _ready()
    return {"sensors": world.lidar.list_sensors(), "budget": world.lidar.budget()}


@router.post("/api/lidar/sensors", summary="Request a sensor")
async def create_sensor(req: SensorRequest, x_admin_token: Optional[str] = Header(default=None),
                        x_session_id: Optional[str] = Header(default=None)):
    world = _ready()
    principal = principal_from_headers(x_admin_token, x_session_id)
    config = req.config or LidarConfig()
    try:
        sensor_id = await request_sensor(world, principal, req.actor_id, config, req.record_raw, req.pinned)
    except LidarError as exc:
        return _lidar_errors(exc)
    return _accepted(world, sensor_id, config)


@router.put("/api/lidar/sensors/{sensor_id}", summary="Reconfigure (destroy and respawn at a tick boundary)")
async def reconfigure_sensor(sensor_id: int, config: LidarConfig, x_admin_token: Optional[str] = Header(default=None),
                             x_session_id: Optional[str] = Header(default=None)):
    world = _ready()
    principal = principal_from_headers(x_admin_token, x_session_id)
    try:
        await on_sim_thread(world, lambda: world.lidar_reconfigure(principal, sensor_id, config))
    except LidarError as exc:
        return _lidar_errors(exc)
    return JSONResponse(status_code=202, content={"sensor_id": sensor_id, "state": SensorState.RECONFIGURING.value,
                                                  "warnings": config.warnings(world.fixed_delta_seconds)})


@router.delete("/api/lidar/sensors/{sensor_id}", summary="Release a sensor")
async def release_sensor(sensor_id: int, x_admin_token: Optional[str] = Header(default=None),
                         x_session_id: Optional[str] = Header(default=None)):
    world = _ready()
    principal = principal_from_headers(x_admin_token, x_session_id)
    try:
        await on_sim_thread(world, lambda: world.lidar_release(principal, sensor_id))
    except LidarError as exc:
        return _lidar_errors(exc)
    return JSONResponse(status_code=202, content={"sensor_id": sensor_id, "state": SensorState.STOPPING.value})


@router.post("/api/sessions/{session_id}/lidar", summary="A vehicle asks for a sensor on its own actor")
async def create_session_sensor(session_id: str, config: Optional[LidarConfig] = None):
    world = _ready()
    principal = principal_from_headers(None, session_id)
    config = config or LidarConfig()
    try:
        sensor_id = await request_sensor(world, principal, None, config)
    except LidarError as exc:
        return _lidar_errors(exc)
    session = world.sessions.get_session(session_id)
    if session is not None:
        session.lidar_enabled = True
    return _accepted(world, sensor_id, config)


# ------------------------------------------------------------------ #
# WebSocket — binary protocol v3 frames + JSON state messages          #
# ------------------------------------------------------------------ #

POLL_S = 0.01


def _parse_profile(msg: Dict[str, Any], sim_rate_hz: float) -> StreamProfile:
    max_points = msg.get("max_points")
    max_hz = msg.get("max_hz") or sim_rate_hz
    if not isinstance(max_hz, (int, float)) or max_hz <= 0:
        raise ValueError("max_hz must be a positive number")
    if max_points is not None and not isinstance(max_points, int):
        raise ValueError("max_points must be an integer or null")
    return quantize_profile(max_points, float(max_hz), bool(msg.get("compress", False)), sim_rate_hz)


@router.websocket("/ws/lidar/{sensor_id}")
async def lidar_websocket(websocket: WebSocket, sensor_id: int):
    """
    View one sensor (observers may view). Binary messages are protocol v3 frames; text messages are JSON::

        {"type": "state", "state": "active", "frame": 1234}
        {"type": "stream_profile", "max_points": 25000, "max_hz": 10.0, "compress": false,
         "points_total": 41000, "points_sent": 25000, "subsampled": true}
        {"type": "error", "message": "..."}

    The client may send ``{"op": "profile", "max_points": 25000, "max_hz": 10, "compress": true}`` (quantised
    server side so viewers share encoded frames) and ``"ping"``.
    """
    await websocket.accept()
    if _world is None or _world.lidar is None:
        await websocket.send_json({"type": "error", "message": "Server not initialised"})
        await websocket.close()
        return
    mgr = _world.lidar
    if mgr.info_of(sensor_id) is None:
        await websocket.send_json({"type": "error", "message": f"no LiDAR sensor {sensor_id}"})
        await websocket.close(code=4404)
        return

    sim_rate = 1.0 / _world.fixed_delta_seconds
    profile = quantize_profile(None, sim_rate, False, sim_rate)
    stream = _world.get_lidar_stream(sensor_id)
    viewer_id = uuid.uuid4().hex
    mgr.attach_viewer(sensor_id, viewer_id)
    profile_box = {"profile": profile, "changed": True}
    loop = asyncio.get_running_loop()

    async def _receive_loop():
        while True:
            msg = await websocket.receive_text()
            if msg == "ping":
                await websocket.send_json({"type": "pong"})
                continue
            try:
                data = json.loads(msg)
                if data.get("op") == "profile":
                    profile_box["profile"] = _parse_profile(data, sim_rate)
                    profile_box["changed"] = True
            except (ValueError, AttributeError) as exc:
                await websocket.send_json({"type": "error", "message": f"bad message: {exc}"})

    receiver = asyncio.create_task(_receive_loop())
    last_state: Optional[str] = None
    last_sent_frame = -1
    last_info: Optional[tuple] = None
    try:
        while True:
            if receiver.done():
                receiver.result()
                break
            info = mgr.info_of(sensor_id)
            state = info["state"] if info else SensorState.DESTROYED.value
            frame = mgr.get_frame(sensor_id)
            if state != last_state:
                last_state = state
                await websocket.send_json({"type": "state", "state": state,
                                           "frame": info["last_frame"] if info else None,
                                           "error": info["error"] if info else None})
                if state in _END_STATES:
                    break
            prof: StreamProfile = profile_box["profile"]
            if frame is not None and frame.frame != last_sent_frame:
                last_sent_frame = frame.frame
                enc = await loop.run_in_executor(None, stream.encode, frame, prof)
                if enc is not None:
                    key = (prof, enc.points_total, enc.points_sent)
                    if key != last_info or profile_box["changed"]:
                        last_info, profile_box["changed"] = key, False
                        await websocket.send_json({
                            "type": "stream_profile", "max_points": prof.max_points, "max_hz": prof.max_hz,
                            "compress": prof.compress, "points_total": enc.points_total,
                            "points_sent": enc.points_sent, "subsampled": enc.subsampled})
                    await websocket.send_bytes(enc.data)
            await asyncio.sleep(POLL_S)
    except WebSocketDisconnect:
        logger.info("LiDAR WS disconnected: sensor=%d", sensor_id)
    except RuntimeError as exc:
        if "close message" in str(exc):
            logger.info("LiDAR WS closed by peer: sensor=%d", sensor_id)
        else:
            logger.error("LiDAR WS error for sensor %d: %s", sensor_id, exc)
    except Exception as exc:
        logger.error("LiDAR WS error for sensor %d: %s", sensor_id, exc)
    finally:
        receiver.cancel()
        mgr.detach_viewer(sensor_id, viewer_id)
