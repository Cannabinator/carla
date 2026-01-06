#!/usr/bin/env python3
"""
FastAPI WebSocket Server for Real-time V2V LiDAR Visualization
Streams semantic LiDAR data from multiple vehicles to web browser.
"""

import asyncio
import json
import logging
from pathlib import Path
from typing import List, Optional, Dict, Any
import threading

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse
from pydantic import BaseModel

from .collector import LiDARDataCollector

logger = logging.getLogger(__name__)

# Forward declaration for run_simulation_headless (imported lazily to avoid circular import)
_run_simulation_headless = None

app = FastAPI(title="V2V LiDAR Visualizer")

# Global collector instance (set by scenario)
_collector: Optional[LiDARDataCollector] = None
_v2v_network: Optional[object] = None
_streaming_task: Optional[asyncio.Task] = None
_event_loop: Optional[asyncio.AbstractEventLoop] = None  # Store event loop reference

# Simulation control
_simulation_thread: Optional[threading.Thread] = None
_simulation_status: Dict[str, Any] = {
    "running": False,
    "status": "idle",
    "frame": 0,
    "elapsed": 0,
    "vehicles": 0,
    "v2v_messages": 0,
    "error": None
}
_simulation_stop_flag = False


class SimulationConfig(BaseModel):
    """Simulation configuration model."""
    duration: int = 120
    vehicles: int = 10
    v2v_range: int = 75
    lidar_quality: str = "high"
    csv_logging: bool = True
    console_output: bool = True


def set_collector(collector: LiDARDataCollector):
    """Set the global LiDAR collector and start streaming."""
    global _collector, _streaming_task, _event_loop
    _collector = collector
    logger.info("LiDAR collector registered with server")
    
    # If there are already WebSocket connections waiting, start streaming now!
    if len(manager.active_connections) > 0 and (_streaming_task is None or _streaming_task.done()):
        if _event_loop is not None:
            # Schedule the task in the main event loop from worker thread
            future = asyncio.run_coroutine_threadsafe(
                stream_lidar_data(_collector, update_rate=0.1),
                _event_loop
            )
            logger.info(f"LiDAR streaming task scheduled for {len(manager.active_connections)} waiting clients")
        else:
            logger.warning("Event loop not available - streaming will start on next WebSocket connection")
    else:
        logger.info("LiDAR streaming will start automatically when clients connect")


def set_v2v_network(v2v_network):
    """Set the global V2V network."""
    global _v2v_network
    _v2v_network = v2v_network
    logger.info("V2V network registered with server")


class ConnectionManager:
    """Manages WebSocket connections."""
    
    def __init__(self):
        self.active_connections: List[WebSocket] = []
    
    async def connect(self, websocket: WebSocket):
        """Accept new WebSocket connection."""
        await websocket.accept()
        self.active_connections.append(websocket)
        logger.info(f"New WebSocket connection. Total: {len(self.active_connections)}")
    
    def disconnect(self, websocket: WebSocket):
        """Remove WebSocket connection."""
        if websocket in self.active_connections:
            self.active_connections.remove(websocket)
            logger.info(f"WebSocket disconnected. Remaining: {len(self.active_connections)}")
    
    async def broadcast(self, message: str):
        """Broadcast message to all connected clients."""
        if not self.active_connections:
            return
            
        disconnected = []
        for connection in self.active_connections:
            try:
                await connection.send_text(message)
            except Exception as e:
                logger.error(f"Error sending to client: {e}")
                disconnected.append(connection)
        
        # Remove disconnected clients
        for conn in disconnected:
            if conn in self.active_connections:
                self.active_connections.remove(conn)


manager = ConnectionManager()


@app.get("/")
async def root():
    """Serve unified viewer with LiDAR and V2V tabs."""
    unified_viewer_path = Path(__file__).parent.parent / 'web' / 'unified_viewer.html'
    if unified_viewer_path.exists():
        return HTMLResponse(content=unified_viewer_path.read_text())
    # Fallback to LiDAR only
    return HTMLResponse(content="<h1>Visualization Server</h1><p><a href='/lidar'>LiDAR Viewer</a></p>")


@app.get("/lidar")
async def lidar_viewer():
    """Serve LiDAR 3D viewer."""
    viewer_path = Path(__file__).parent.parent / 'web' / 'viewer.html'
    if viewer_path.exists():
        return HTMLResponse(content=viewer_path.read_text())
    return HTMLResponse(content="<h1>LiDAR viewer not found</h1>", status_code=404)


@app.get("/control")
async def control_panel():
    """Serve control panel."""
    control_path = Path(__file__).parent.parent / 'web' / 'control_panel.html'
    if control_path.exists():
        return HTMLResponse(content=control_path.read_text())
    return HTMLResponse(content="<h1>Control panel not found</h1>", status_code=404)


@app.get("/v2v")
async def v2v_dashboard():
    """Serve V2V dashboard."""
    # Import V2V dashboard HTML
    v2v_dashboard_path = Path(__file__).parent.parent.parent / 'v2v' / 'dashboard.html'
    if v2v_dashboard_path.exists():
        # Replace API endpoint URLs to match server routes
        content = v2v_dashboard_path.read_text()
        # Fix all fetch URLs to use correct API endpoints
        content = content.replace('${window.location.hostname}:8001/network/stats', '${window.location.hostname}:8000/api/v2v/network/stats')
        content = content.replace('${window.location.hostname}:8001/vehicles/0/neighbors', '${window.location.hostname}:8000/api/v2v/vehicles/0/neighbors')
        content = content.replace('${window.location.hostname}:8001/vehicles/0/threats', '${window.location.hostname}:8000/api/v2v/vehicles/0/threats')
        content = content.replace('${window.location.hostname}:8001/vehicles/0', '${window.location.hostname}:8000/api/v2v/vehicles/0')
        
        # Replace WebSocket connection logic with REST API polling
        ws_connect = '''// Connect to WebSocket
        function connect() {
            const wsUrl = `ws://${window.location.hostname}:8001/ws/v2v`;
            ws = new WebSocket(wsUrl);
            
            ws.onopen = () => {
                console.log('Connected to V2V WebSocket');
                statusEl.textContent = 'Connected';
                statusEl.className = 'status connected';
                if (reconnectInterval) {
                    clearInterval(reconnectInterval);
                    reconnectInterval = null;
                }
            };
            
            ws.onmessage = (event) => {
                const data = JSON.parse(event.data);
                updateDashboard(data);
            };
            
            ws.onerror = (error) => {
                console.error('WebSocket error:', error);
                statusEl.textContent = 'Error';
                statusEl.className = 'status disconnected';
            };
            
            ws.onclose = () => {
                console.log('Disconnected from V2V WebSocket');
                statusEl.textContent = 'Disconnected';
                statusEl.className = 'status disconnected';
                
                // Try to reconnect
                if (!reconnectInterval) {
                    reconnectInterval = setInterval(() => {
                        console.log('Attempting to reconnect...');
                        connect();
                    }, 2000);
                }
            };
        }'''
        
        rest_polling = '''// Use REST API polling instead of WebSocket
        function connect() {
            console.log('Using REST API polling');
            statusEl.textContent = 'Connected';
            statusEl.className = 'status connected';
            // Initial fetch
            updateDashboard({});
        }'''
        
        content = content.replace(ws_connect, rest_polling)
        
        # Update the periodic refresh to not check WebSocket state
        ws_check = '''if (ws && ws.readyState === WebSocket.OPEN) {
                fetchNetworkStats();
                updateEgoInfo();
                fetchNeighbors();
                fetchThreats();
            }'''
        
        rest_refresh = '''fetchNetworkStats();
            updateEgoInfo();
            fetchNeighbors();
            fetchThreats();'''
        
        content = content.replace(ws_check, rest_refresh)
        
        return HTMLResponse(content=content)
    return HTMLResponse(content="<h1>V2V dashboard not found</h1>", status_code=404)


@app.on_event("startup")
async def startup_event():
    """Store event loop reference when FastAPI starts."""
    global _streaming_task, _event_loop
    
    # Store event loop reference for cross-thread scheduling
    _event_loop = asyncio.get_running_loop()
    logger.info(f"Event loop stored: {id(_event_loop)}")
    
    if _collector is not None:
        _streaming_task = asyncio.create_task(stream_lidar_data(_collector, update_rate=0.1))
        logger.info("Streaming task started in FastAPI event loop")
    else:
        logger.warning("No collector set - call set_collector() before starting server")


@app.get("/")
async def get_viewer():
    """Serve the LiDAR viewer HTML page."""
    html_path = Path(__file__).parent.parent / "web" / "viewer.html"
    if html_path.exists():
        return HTMLResponse(content=html_path.read_text())
    else:
        return HTMLResponse(content="""
        <html>
            <head><title>V2V LiDAR Viewer</title></head>
            <body>
                <h1>V2V LiDAR Viewer</h1>
                <p>HTML viewer not found at {}</p>
                <p>Please ensure viewer.html exists in src/visualization/web/</p>
            </body>
        </html>
        """.format(html_path))


@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    """WebSocket endpoint for streaming LiDAR data."""
    global _streaming_task
    
    await manager.connect(websocket)
    
    # Start streaming task if collector is now available and task isn't running
    if _collector is not None and (_streaming_task is None or _streaming_task.done()):
        _streaming_task = asyncio.create_task(stream_lidar_data(_collector, update_rate=0.1))
        logger.info("LiDAR streaming task started on WebSocket connection")
    
    try:
        while True:
            # Keep connection alive and receive pings
            await websocket.receive_text()
    except WebSocketDisconnect:
        manager.disconnect(websocket)
    except Exception as e:
        logger.error(f"WebSocket error: {e}")
        manager.disconnect(websocket)
        logger.error(f"WebSocket error: {e}")
        manager.disconnect(websocket)


async def stream_lidar_data(collector: Optional[LiDARDataCollector] = None, update_rate: float = 0.1):
    """Background task to stream LiDAR data to clients.
    
    Args:
        collector: LiDARDataCollector instance (uses global if None)
        update_rate: Update interval in seconds (10 Hz default)
    """
    if collector is None:
        collector = _collector
    
    if collector is None:
        logger.error("No collector available for streaming")
        return
    
    logger.info("Streaming loop started")
    while True:
        try:
            if len(manager.active_connections) > 0:
                data = collector.get_combined_pointcloud()
                if data and data.get('num_points', 0) > 0:
                    logger.info(f"📡 Broadcasting {data['num_points']} points to {len(manager.active_connections)} clients")
                    await manager.broadcast(json.dumps(data))
                else:
                    logger.warning(f"❌ No data: data={data is not None}, points={data.get('num_points', 0) if data else 0}")
            await asyncio.sleep(update_rate)
        except asyncio.CancelledError:
            logger.info("Streaming task cancelled")
            break
        except Exception as e:
            logger.error(f"Error streaming data: {e}", exc_info=True)
            await asyncio.sleep(1.0)


# V2V API Endpoints
@app.get("/api/v2v/vehicles")
async def get_vehicles():
    """Get list of all vehicle IDs in V2V network."""
    if _v2v_network is None:
        return []
    return list(_v2v_network.vehicles.keys())


@app.get("/api/v2v/network/stats")
async def get_network_stats():
    """Get V2V network statistics."""
    if _v2v_network is None:
        return {"error": "V2V network not available"}
    
    stats = _v2v_network.get_network_stats()
    return {
        "total_vehicles": len(_v2v_network.vehicles),
        "total_messages_sent": stats['total_messages_sent'],
        "average_neighbors": stats['average_neighbors'],
        "max_neighbors": stats['max_neighbors'],
        "cooperative_shares": stats['cooperative_shares'],
        "update_rate_hz": _v2v_network.update_rate_hz,
        "max_range_m": _v2v_network.max_range
    }


@app.get("/api/v2v/vehicles/{vehicle_id}/neighbors")
async def get_neighbors(vehicle_id: int):
    """Get neighbors for specific vehicle."""
    if _v2v_network is None:
        return []
    
    if vehicle_id not in _v2v_network.vehicles:
        return []
    
    neighbor_bsms = _v2v_network.get_neighbors(vehicle_id)
    ego_bsm = _v2v_network.get_bsm(vehicle_id)
    
    if not ego_bsm:
        return []
    
    neighbors = []
    for neighbor_bsm in neighbor_bsms:
        distance = _v2v_network.get_distance(vehicle_id, neighbor_bsm.vehicle_id)
        rel_speed = abs(neighbor_bsm.speed - ego_bsm.speed)
        
        neighbors.append({
            "vehicle_id": neighbor_bsm.vehicle_id,
            "distance": distance if distance else 0.0,
            "relative_speed": rel_speed,
            "bsm": {
                "speed": neighbor_bsm.speed,
                "heading": neighbor_bsm.heading,
                "position": {
                    "x": neighbor_bsm.latitude,
                    "y": neighbor_bsm.longitude,
                    "z": neighbor_bsm.elevation
                },
                "acceleration": {
                    "longitudinal": neighbor_bsm.longitudinal_accel,
                    "lateral": neighbor_bsm.lateral_accel,
                    "vertical": neighbor_bsm.vertical_accel
                }
            }
        })
    
    return neighbors


@app.get("/api/v2v/vehicles/{vehicle_id}/threats")
async def get_threats(vehicle_id: int):
    """Get threat assessment for vehicle."""
    if _v2v_network is None:
        return []
    
    if vehicle_id not in _v2v_network.vehicles:
        return []
    
    threats = _v2v_network.get_threats(vehicle_id)
    import math
    return [{
        "other_vehicle_id": t['other_vehicle_id'],
        "threat_level": t['level'],
        "time_to_collision": None if (math.isinf(t['ttc']) or math.isnan(t['ttc'])) else t['ttc'],
        "distance": t['distance'],
        "timestamp": t['timestamp']
    } for t in threats]


@app.get("/api/v2v/vehicles/{vehicle_id}")
async def get_vehicle_bsm(vehicle_id: int):
    """Get BSM for specific vehicle."""
    if _v2v_network is None:
        return {"error": "V2V network not available"}
    
    bsm = _v2v_network.get_bsm(vehicle_id)
    if not bsm:
        return {"error": "Vehicle not found"}
    
    return {
        "vehicle_id": bsm.vehicle_id,
        "speed": bsm.speed,
        "heading": bsm.heading,
        "position": {
            "x": bsm.latitude,
            "y": bsm.longitude,
            "z": bsm.elevation
        },
        "acceleration": {
            "longitudinal": bsm.longitudinal_accel,
            "lateral": bsm.lateral_accel,
            "vertical": bsm.vertical_accel
        }
    }


# Simulation Control API Endpoints
@app.post("/api/simulation/start")
async def start_simulation(config: SimulationConfig):
    """Start CARLA simulation with specified configuration."""
    global _simulation_thread, _simulation_status, _simulation_stop_flag
    
    if _simulation_status["running"]:
        return {"error": "Simulation already running"}
    
    # Reset stop flag
    _simulation_stop_flag = False
    
    # Update status
    _simulation_status = {
        "running": True,
        "status": "starting",
        "frame": 0,
        "elapsed": 0,
        "vehicles": config.vehicles,
        "v2v_messages": 0,
        "error": None
    }
    
    # Run simulation in background thread
    def run_simulation():
        try:
            from src.scenarios.v2v_complete_demo import run_simulation_headless
            # Pass server module reference to avoid thread isolation
            import sys
            server_module = sys.modules[__name__]
            run_simulation_headless(
                duration=config.duration,
                vehicles=config.vehicles,
                v2v_range=config.v2v_range,
                lidar_quality=config.lidar_quality,
                csv_logging=config.csv_logging,
                console_output=config.console_output,
                status_callback=update_simulation_status,
                server_module=server_module
            )
            _simulation_status["running"] = False
            _simulation_status["status"] = "completed"
        except Exception as e:
            logger.error(f"Simulation error: {e}", exc_info=True)
            _simulation_status["running"] = False
            _simulation_status["status"] = "error"
            _simulation_status["error"] = str(e)
    
    _simulation_thread = threading.Thread(target=run_simulation, daemon=True)
    _simulation_thread.start()
    
    return {"status": "started", "config": config.dict()}


@app.post("/api/simulation/stop")
async def stop_simulation():
    """Stop running simulation."""
    global _simulation_stop_flag, _simulation_status
    
    if not _simulation_status["running"]:
        return {"error": "No simulation running"}
    
    _simulation_stop_flag = True
    _simulation_status["status"] = "stopping"
    
    return {"status": "stopping"}


@app.get("/api/simulation/status")
async def get_simulation_status():
    """Get current simulation status."""
    return _simulation_status


def update_simulation_status(frame: int, elapsed: float, v2v_msgs: int):
    """Callback to update simulation status from running thread."""
    global _simulation_status
    _simulation_status["frame"] = frame
    _simulation_status["elapsed"] = int(elapsed)
    _simulation_status["v2v_messages"] = v2v_msgs
    _simulation_status["status"] = "running"


def should_stop_simulation() -> bool:
    """Check if simulation should stop."""
    return _simulation_stop_flag


def run_server(host: str = '0.0.0.0', port: int = 8000):
    """Run the FastAPI server.
    
    Args:
        host: Server host
        port: Server port
    """
    import uvicorn
    uvicorn.run(app, host=host, port=port)


if __name__ == "__main__":
    import argparse
    
    parser = argparse.ArgumentParser(description='V2V LiDAR Visualization Server')
    parser.add_argument('--host', default='0.0.0.0', help='Server host')
    parser.add_argument('--port', type=int, default=8000, help='Server port')
    
    args = parser.parse_args()
    
    logger.info(f"Starting V2V LiDAR server on {args.host}:{args.port}")
    logger.info(f"Open browser at http://localhost:{args.port}")
    
    run_server(host=args.host, port=args.port)
