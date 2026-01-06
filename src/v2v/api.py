"""
V2V REST API
FastAPI-based REST API for accessing V2V network data.
"""

from fastapi import FastAPI, HTTPException, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import HTMLResponse
from typing import List, Optional, Dict
from pydantic import BaseModel
import asyncio
import json
from datetime import datetime
from pathlib import Path

from .messages import BSMCore, VehicleType, BrakingStatus
from .network_enhanced import V2VNetworkEnhanced


# Pydantic models for API responses
class BSMResponse(BaseModel):
    """BSM message response model"""
    vehicle_id: int
    timestamp: float
    msg_count: int
    vehicle_type: str
    position: Dict[str, float]  # {x, y, z}
    speed: float
    heading: float
    steering_angle: float
    acceleration: Dict[str, float]  # {longitudinal, lateral, vertical}
    dimensions: Dict[str, float]  # {length, width, height}
    brake_status: str
    brake_pressure: float
    transmission_state: str


class NeighborInfo(BaseModel):
    """Neighbor vehicle information"""
    vehicle_id: int
    distance: float
    relative_speed: float
    bsm: BSMResponse


class ThreatInfo(BaseModel):
    """Threat assessment information"""
    other_vehicle_id: int
    threat_level: int  # 0-4
    time_to_collision: float
    distance: float
    timestamp: float


class NetworkStats(BaseModel):
    """Network statistics"""
    total_vehicles: int
    total_messages_sent: int
    average_neighbors: float
    max_neighbors: int
    cooperative_shares: int
    update_rate_hz: float
    max_range_m: float


class V2VAPI:
    """V2V Network REST API"""
    
    def __init__(self, v2v_network: V2VNetworkEnhanced, port: int = 8001):
        """
        Initialize V2V API.
        
        Args:
            v2v_network: V2VNetworkEnhanced instance
            port: API port (default 8001)
        """
        self.v2v = v2v_network
        self.port = port
        self.app = FastAPI(title="V2V Network API", version="1.0.0")
        
        # Enable CORS
        self.app.add_middleware(
            CORSMiddleware,
            allow_origins=["*"],
            allow_credentials=True,
            allow_methods=["*"],
            allow_headers=["*"],
        )
        
        # WebSocket connections for real-time updates
        self.websocket_clients: List[WebSocket] = []
        
        self._setup_routes()
    
    def _setup_routes(self):
        """Setup API routes"""
        
        @self.app.get("/")
        async def root():
            return {
                "service": "V2V Network API",
                "version": "1.0.0",
                "endpoints": [
                    "/dashboard",
                    "/vehicles",
                    "/vehicles/{vehicle_id}",
                    "/vehicles/{vehicle_id}/neighbors",
                    "/vehicles/{vehicle_id}/threats",
                    "/bsm",
                    "/bsm/{vehicle_id}",
                    "/network/stats",
                    "/ws/v2v"
                ]
            }
        
        @self.app.get("/dashboard", response_class=HTMLResponse)
        async def dashboard():
            """Serve V2V Dashboard HTML"""
            dashboard_path = Path(__file__).parent / "dashboard.html"
            if dashboard_path.exists():
                return HTMLResponse(content=dashboard_path.read_text())
            return HTMLResponse(content="<h1>Dashboard not found</h1>", status_code=404)
        
        @self.app.get("/vehicles", response_model=List[int])
        async def get_vehicles():
            """Get list of all vehicle IDs in network"""
            return list(self.v2v.vehicles.keys())
        
        @self.app.get("/vehicles/{vehicle_id}", response_model=BSMResponse)
        async def get_vehicle(vehicle_id: int):
            """Get BSM data for specific vehicle"""
            bsm = self.v2v.get_bsm(vehicle_id)
            if not bsm:
                raise HTTPException(status_code=404, detail="Vehicle not found")
            return self._bsm_to_response(bsm)
        
        @self.app.get("/vehicles/{vehicle_id}/neighbors", response_model=List[NeighborInfo])
        async def get_neighbors(vehicle_id: int):
            """Get neighboring vehicles and their BSM data"""
            if vehicle_id not in self.v2v.vehicles:
                raise HTTPException(status_code=404, detail="Vehicle not found")
            
            neighbor_bsms = self.v2v.get_neighbors(vehicle_id)
            ego_bsm = self.v2v.get_bsm(vehicle_id)
            
            if not ego_bsm:
                return []
            
            neighbors = []
            for neighbor_bsm in neighbor_bsms:
                distance = self.v2v.get_distance(vehicle_id, neighbor_bsm.vehicle_id)
                
                # Calculate relative speed
                rel_speed = abs(neighbor_bsm.speed - ego_bsm.speed)
                
                neighbors.append(NeighborInfo(
                    vehicle_id=neighbor_bsm.vehicle_id,
                    distance=distance if distance else 0.0,
                    relative_speed=rel_speed,
                    bsm=self._bsm_to_response(neighbor_bsm)
                ))
            
            return neighbors
        
        @self.app.get("/vehicles/{vehicle_id}/threats", response_model=List[ThreatInfo])
        async def get_threats(vehicle_id: int):
            """Get threat assessment for vehicle"""
            if vehicle_id not in self.v2v.vehicles:
                raise HTTPException(status_code=404, detail="Vehicle not found")
            
            threats = self.v2v.get_threats(vehicle_id)
            # Map field names from internal format to API format
            return [ThreatInfo(
                other_vehicle_id=t['other_vehicle_id'],
                threat_level=t['level'],
                time_to_collision=t['ttc'],
                distance=t['distance'],
                timestamp=t['timestamp']
            ) for t in threats]
        
        @self.app.get("/bsm", response_model=List[BSMResponse])
        async def get_all_bsm():
            """Get all BSM messages in network"""
            all_bsm = self.v2v.get_all_bsm()
            return [self._bsm_to_response(bsm) for bsm in all_bsm.values()]
        
        @self.app.get("/bsm/{vehicle_id}", response_model=BSMResponse)
        async def get_bsm(vehicle_id: int):
            """Get BSM message for specific vehicle"""
            bsm = self.v2v.get_bsm(vehicle_id)
            if not bsm:
                raise HTTPException(status_code=404, detail="Vehicle not found")
            return self._bsm_to_response(bsm)
        
        @self.app.get("/network/stats", response_model=NetworkStats)
        async def get_network_stats():
            """Get network statistics"""
            stats = self.v2v.get_network_stats()
            return NetworkStats(
                total_vehicles=len(self.v2v.vehicles),
                total_messages_sent=stats['total_messages_sent'],
                average_neighbors=stats['average_neighbors'],
                max_neighbors=stats['max_neighbors'],
                cooperative_shares=stats['cooperative_shares'],
                update_rate_hz=self.v2v.update_rate_hz,
                max_range_m=self.v2v.max_range
            )
        
        @self.app.websocket("/ws/v2v")
        async def websocket_v2v(websocket: WebSocket):
            """WebSocket endpoint for real-time V2V updates"""
            await websocket.accept()
            self.websocket_clients.append(websocket)
            
            try:
                while True:
                    # Send V2V data every update
                    await asyncio.sleep(self.v2v.update_interval)
                    
                    data = {
                        "timestamp": datetime.now().isoformat(),
                        "vehicles": len(self.v2v.vehicles),
                        "bsm_messages": [
                            self._bsm_to_dict(bsm) 
                            for bsm in self.v2v.get_all_bsm().values()
                        ]
                    }
                    
                    await websocket.send_json(data)
            
            except WebSocketDisconnect:
                self.websocket_clients.remove(websocket)
    
    def _bsm_to_response(self, bsm: BSMCore) -> BSMResponse:
        """Convert BSMCore to BSMResponse"""
        return BSMResponse(
            vehicle_id=bsm.vehicle_id,
            timestamp=bsm.timestamp,
            msg_count=bsm.msg_count,
            vehicle_type=VehicleType(bsm.vehicle_type).name,
            position={
                "x": bsm.latitude,
                "y": bsm.longitude,
                "z": bsm.elevation
            },
            speed=bsm.speed,
            heading=bsm.heading,
            steering_angle=bsm.steering_angle,
            acceleration={
                "longitudinal": bsm.longitudinal_accel,
                "lateral": bsm.lateral_accel,
                "vertical": bsm.vertical_accel
            },
            dimensions={
                "length": bsm.vehicle_length,
                "width": bsm.vehicle_width,
                "height": bsm.vehicle_height
            },
            brake_status=BrakingStatus(bsm.brake_status).name,
            brake_pressure=bsm.brake_pressure,
            transmission_state=bsm.transmission_state
        )
    
    def _bsm_to_dict(self, bsm: BSMCore) -> dict:
        """Convert BSMCore to dictionary"""
        response = self._bsm_to_response(bsm)
        return response.dict()
    
    async def broadcast_update(self, data: dict):
        """Broadcast update to all WebSocket clients"""
        if not self.websocket_clients:
            return
        
        disconnected = []
        for client in self.websocket_clients:
            try:
                await client.send_json(data)
            except:
                disconnected.append(client)
        
        # Remove disconnected clients
        for client in disconnected:
            self.websocket_clients.remove(client)


def create_v2v_api(v2v_network: V2VNetworkEnhanced, port: int = 8001) -> V2VAPI:
    """
    Create V2V API instance.
    
    Args:
        v2v_network: V2VNetworkEnhanced instance
        port: API port
    
    Returns:
        V2VAPI instance
    """
    return V2VAPI(v2v_network, port)
