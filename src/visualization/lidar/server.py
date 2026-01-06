#!/usr/bin/env python3
"""
FastAPI WebSocket Server for Real-time V2V LiDAR Visualization
Streams semantic LiDAR data from multiple vehicles to web browser.
"""

import asyncio
import json
import logging
from pathlib import Path
from typing import List

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse

from .collector import LiDARDataCollector

logger = logging.getLogger(__name__)

app = FastAPI(title="V2V LiDAR Visualizer")

# Global collector instance (set by scenario)
_collector: LiDARDataCollector = None
_streaming_task = None


def set_collector(collector: LiDARDataCollector):
    """Set the global LiDAR collector."""
    global _collector
    _collector = collector
    logger.info("LiDAR collector registered with server")


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


@app.on_event("startup")
async def startup_event():
    """Start streaming task when FastAPI starts."""
    global _streaming_task
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
    await manager.connect(websocket)
    try:
        while True:
            # Keep connection alive and receive pings
            await websocket.receive_text()
    except WebSocketDisconnect:
        manager.disconnect(websocket)
    except Exception as e:
        logger.error(f"WebSocket error: {e}")
        manager.disconnect(websocket)


async def stream_lidar_data(collector: LiDARDataCollector = None, update_rate: float = 0.1):
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
                    await manager.broadcast(json.dumps(data))
            await asyncio.sleep(update_rate)
        except asyncio.CancelledError:
            logger.info("Streaming task cancelled")
            break
        except Exception as e:
            logger.error(f"Error streaming data: {e}", exc_info=True)
            await asyncio.sleep(1.0)


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
