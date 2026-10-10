# TODO: CARLA handles / the module-global world are Optional until a connected-world type exists (WorldController state machine)
# pyright: reportOptionalMemberAccess=false
"""
Client-facing FastAPI router.

Any device on the network (laptop, raspi, workstation…) can:
  1. POST /client/join   → register + spawn their leading vehicle
  2. WS   /ws/client/{session_id}  → receive real-time telemetry
  3. DELETE /client/leave/{session_id}  → graceful exit + despawn

No authentication is required — any device on the LAN can connect.

Mount on the main app with::

    from src.server.client_routes import router as client_router
    app.include_router(client_router)
"""

import asyncio
import logging
from typing import Optional

from fastapi import APIRouter, HTTPException, WebSocket, WebSocketDisconnect, Request
from pydantic import BaseModel, Field

from src.lidar.config import LidarConfig
from src.lidar.sensor import LidarBudgetExceeded, LidarError, Principal
from src.server.lidar_routes import request_sensor
from src.server.route_utils import on_sim_thread

logger = logging.getLogger(__name__)

router = APIRouter(tags=["client"])

TELEMETRY_PERIOD_S = 0.2  # 5 Hz

# Injected once at startup by start_server.py
_world = None

# Vehicle blueprints a client may request (validated to be always-available)
ALLOWED_BLUEPRINTS = [
    "vehicle.tesla.model3",
    "vehicle.audi.a2",
    "vehicle.bmw.grandtourer",
    "vehicle.citroen.c3",
    "vehicle.dodge.charger_2020",
    "vehicle.ford.mustang",
    "vehicle.jeep.wrangler_rubicon",
    "vehicle.lincoln.mkz_2020",
    "vehicle.mercedes.coupe",
    "vehicle.mini.cooperst",
    "vehicle.nissan.micra",
    "vehicle.seat.leon",
    "vehicle.toyota.prius",
    "vehicle.volkswagen.t2_2021",
]
_DEFAULT_BLUEPRINT = "vehicle.tesla.model3"


def set_persistent_world(world) -> None:
    global _world
    _world = world


# ------------------------------------------------------------------ #
# Request / response models                                           #
# ------------------------------------------------------------------ #

class JoinRequest(BaseModel):
    client_name: str = Field(..., min_length=1, max_length=64,
                             description="Friendly name shown in the admin panel (e.g. 'raspi-1')")
    vehicle_blueprint: str = Field(default=_DEFAULT_BLUEPRINT,
                                   description="CARLA blueprint ID for the leading vehicle")
    duration_seconds: Optional[float] = Field(
        default=None, ge=10, le=86400,
        description="Session lifetime in seconds. Omit or set null to run until disconnect.",
    )
    enable_lidar: bool = Field(
        default=False,
        description="Request a LiDAR sensor on the vehicle (stream it from /ws/lidar/{sensor_id}).",
    )
    lidar_config: Optional[LidarConfig] = Field(default=None, description="Sensor configuration (defaults apply)")

class JoinResponse(BaseModel):
    session_id: str
    session_token: str = Field(description="Secret for the WebSocket hub (auth message) and the X-Session-Token "
                                           "header; shown only to the client that joined")
    actor_id: int
    ws_url: str
    lidar_ws_url: Optional[str] = None
    lidar_sensor_id: Optional[int] = None
    lidar_error: Optional[dict] = None
    message: str


# ------------------------------------------------------------------ #
# REST endpoints                                                      #
# ------------------------------------------------------------------ #

@router.get("/client/blueprints", summary="Available vehicle blueprints")
async def get_blueprints():
    return {"blueprints": ALLOWED_BLUEPRINTS, "default": _DEFAULT_BLUEPRINT}


@router.post("/client/join", response_model=JoinResponse, summary="Join the live server")
async def join_server(req: JoinRequest, request: Request):
    """
    Register a client device.  Spawns a leading vehicle immediately and
    returns the session_id and the WebSocket URL for telemetry streaming.
    """
    _assert_world_ready()

    # Sanitise blueprint choice
    blueprint = req.vehicle_blueprint if req.vehicle_blueprint in ALLOWED_BLUEPRINTS else _DEFAULT_BLUEPRINT

    client_ip = request.client.host if request.client else "unknown"

    session = _world.sessions.create_session(
        client_name=req.client_name,
        client_ip=client_ip,
        vehicle_blueprint=blueprint,
        duration_seconds=req.duration_seconds,
    )

    try:
        actor_id = await on_sim_thread(
            _world, _world.spawn_client_vehicle, session.session_id, blueprint
        )
    except HTTPException:
        _world.sessions.remove_session(session.session_id)
        raise
    except Exception as exc:
        _world.sessions.remove_session(session.session_id)
        logger.error("Vehicle spawn failed for session %s: %s", session.session_id, exc)
        raise HTTPException(status_code=500, detail=f"Vehicle spawn failed: {exc}")

    if actor_id is None:
        _world.sessions.remove_session(session.session_id)
        raise HTTPException(status_code=500, detail="Could not find a free spawn point")

    duration_msg = (
        f"{int(req.duration_seconds)}s" if req.duration_seconds else "until you disconnect"
    )

    lidar_ws_url: Optional[str] = None
    lidar_sensor_id: Optional[int] = None
    lidar_error: Optional[dict] = None
    if req.enable_lidar:
        try:
            lidar_sensor_id = await request_sensor(
                _world, Principal("vehicle", session.session_id, actor_id, session.client_name), None, req.lidar_config or LidarConfig())
            session.lidar_enabled = True
            lidar_ws_url = f"/ws/lidar/{lidar_sensor_id}"
        except LidarBudgetExceeded as exc:       # the join itself succeeded
            lidar_error = exc.detail
        except LidarError as exc:
            lidar_error = {"error": "lidar_unavailable", "message": str(exc)}

    return JoinResponse(
        session_id=session.session_id,
        session_token=session.session_token,
        actor_id=actor_id,
        ws_url=f"/ws/client/{session.session_id}",
        lidar_ws_url=lidar_ws_url,
        lidar_sensor_id=lidar_sensor_id,
        lidar_error=lidar_error,
        message=f"Welcome {req.client_name}! Vehicle actor={actor_id} spawned. Session runs {duration_msg}.",
    )


@router.delete("/client/leave/{session_id}", summary="Leave the server")
async def leave_server(session_id: str):
    """Gracefully end a session and despawn the leading vehicle."""
    _assert_world_ready()
    session = _world.sessions.get_session(session_id)
    if session is None:
        raise HTTPException(status_code=404, detail="Session not found")

    await on_sim_thread(_world, _world.despawn_client_vehicle, session_id)
    _world.sessions.end_session(session_id)
    return {"ok": True, "message": f"Goodbye {session.client_name}!"}


@router.get("/client/vehicle/{session_id}", summary="Get vehicle telemetry (REST)")
async def get_vehicle_state(session_id: str):
    """One-shot telemetry poll (for clients that can't use WebSocket)."""
    _assert_world_ready()
    session = _world.sessions.get_session(session_id)
    if session is None or not session.is_active:
        raise HTTPException(status_code=404, detail="Session not found or ended")

    _world.sessions.touch(session_id)      # a REST-polling client is alive too
    telemetry = _world.get_vehicle_telemetry(session_id)
    if telemetry is None:
        raise HTTPException(status_code=404, detail="Vehicle not found")

    return {**telemetry, "session": session.to_dict()}


# ------------------------------------------------------------------ #
# WebSocket — per-client telemetry stream                             #
# ------------------------------------------------------------------ #

@router.websocket("/ws/client/{session_id}")
async def client_websocket(websocket: WebSocket, session_id: str):
    """
    Stream real-time telemetry to a specific client at ~5 Hz.

    The server sends JSON frames::

        {
          "type": "telemetry",
          "data": { "actor_id": …, "x": …, "y": …, "z": …,
                    "yaw": …, "speed_kmh": …, "frame": … },
          "remaining_seconds": 120.5   # null if no duration limit
        }

    The client can send ``"ping"`` to keep the connection alive.
    When the session expires the server sends ``{"type": "session_expired"}``
    and closes the connection.
    """
    await websocket.accept()

    if _world is None:
        await websocket.send_json({"type": "error", "message": "Server not initialised"})
        await websocket.close()
        return

    session = _world.sessions.get_session(session_id)
    if session is None or not session.is_active:
        await websocket.send_json({"type": "error", "message": "Invalid or expired session"})
        await websocket.close()
        return

    _world.sessions.mark_connected(session_id)
    logger.info("Client WS connected: session=%s (%s)", session_id, session.client_name)

    async def _receive_loop():
        # Separate task: a client "ping" is answered at once and never delays telemetry.
        while True:
            msg = await websocket.receive_text()
            if msg == "ping":
                await websocket.send_json({"type": "pong"})

    receiver = asyncio.create_task(_receive_loop())
    try:
        while True:
            # Session expired?
            if not session.is_active or session.is_expired:
                await websocket.send_json({"type": "session_expired"})
                break
            if receiver.done():  # client closed or the socket failed
                receiver.result()
                break

            _world.sessions.touch(session_id)

            telemetry = _world.get_vehicle_telemetry(session_id)
            if telemetry:
                v2x_data = _world.get_vehicle_v2x_data(session_id)
                await websocket.send_json({
                    "type": "telemetry",
                    "data": {**telemetry, "v2x": v2x_data},
                    "remaining_seconds": session.remaining_seconds,
                })

            await asyncio.sleep(TELEMETRY_PERIOD_S)

    except WebSocketDisconnect:
        logger.info("Client WS disconnected: session=%s", session_id)
    except RuntimeError as exc:
        if "close message" in str(exc):  # peer closed while we were sending: a normal disconnect
            logger.info("Client WS closed by peer: session=%s", session_id)
        else:
            logger.error("Client WS error for session %s: %s", session_id, exc)
    except Exception as exc:
        logger.error("Client WS error for session %s: %s", session_id, exc)
    finally:
        receiver.cancel()
        if _world is not None:
            _world.sessions.mark_disconnected(session_id)
            grace = getattr(_world, "session_disconnect_grace_s", 0.0)
            if grace > 0:
                # D4: keep the vehicle; the simulation thread removes the session if nobody comes back in time
                logger.info("Client WS closed: session=%s kept for %.0f s", session_id, grace)
            else:
                try:
                    await on_sim_thread(_world, _world.despawn_client_vehicle, session_id)
                except Exception as exc:  # world may have stopped meanwhile
                    logger.warning("Cleanup of session %s skipped: %s", session_id, exc)
                _world.sessions.end_session(session_id)
                logger.info("Auto-cleaned session %s after WS close", session_id)


# ------------------------------------------------------------------ #
# Internal helpers                                                    #
# ------------------------------------------------------------------ #

def _assert_world_ready():
    if _world is None:
        raise HTTPException(status_code=503, detail="CARLA server not started")
    if not _world._running:
        raise HTTPException(status_code=503, detail="Simulation not active yet")
