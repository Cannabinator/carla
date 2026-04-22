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
import math
from datetime import datetime
from pathlib import Path

from .messages import (
    BSMCore, VehicleType, BrakingStatus,
    DENM, DENMCauseCode,
    create_cam_from_bsm,
)
from .network_enhanced import V2VNetworkEnhanced


# Pydantic models for API responses
class BSMResponse(BaseModel):
    """BSM message response model"""
    vehicle_id: int
    timestamp: float
    msg_count: int
    vehicle_type: str
    position: Dict[str, float]  # {x, y, z}
    position_accuracy: float
    speed: float
    heading: float
    steering_angle: float
    acceleration: Dict[str, float]  # {longitudinal, lateral, vertical}
    yaw_rate: float
    dimensions: Dict[str, float]  # {length, width, height}
    brake_status: str
    brake_pressure: float
    transmission_state: str
    throttle_confidence: float
    brake_confidence: float
    steering_confidence: float


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


class EnhancedMetadata(BaseModel):
    """Optional enhanced metadata for V2V messages"""
    transmission_time: Optional[float] = None
    reception_time: Optional[float] = None
    link_quality: Optional[float] = None
    hop_count: Optional[int] = None
    priority: Optional[int] = None


class CAMResponse(BaseModel):
    """ETSI ITS-G5 Cooperative Awareness Message response model"""
    station_id: int
    generation_time: float
    reference_position: Dict[str, float]   # {x, y, z}
    heading: float
    speed: float
    drive_direction: str
    vehicle_role: str
    vehicle_length: float
    vehicle_width: float


class DENMResponse(BaseModel):
    """ETSI EN 302 637-3 Decentralized Environmental Notification Message response"""
    station_id: int
    action_id: int
    detection_time: float
    reference_time: float
    event_position: Dict[str, float]   # {x, y, z}
    cause_code: int
    cause_code_name: str
    subcause_code: int
    information_quality: int
    relevance_distance: float
    relevance_traffic_direction: str
    event_speed: Optional[float]
    event_heading: Optional[float]
    termination: Optional[str]


class DENMPublish(BaseModel):
    """Request body for publishing a DENM"""
    station_id: int
    action_id: int
    detection_time: float
    reference_time: float
    event_position: Dict[str, float]   # {x, y, z}
    cause_code: int = 0
    subcause_code: int = 0
    information_quality: int = 0
    relevance_distance: float = 150.0
    relevance_traffic_direction: str = "allTrafficDirections"
    event_speed: Optional[float] = None
    event_heading: Optional[float] = None


class ThreatOverview(BaseModel):
    """Threat assessment with safe TTC handling"""
    other_vehicle_id: int
    threat_level: int  # 0-4
    time_to_collision: Optional[float]
    distance: float
    timestamp: float


class VehicleOverview(BaseModel):
    """Aggregated per-vehicle overview"""
    vehicle_id: int
    bsm: BSMResponse
    neighbors: List[NeighborInfo]
    threats: List[ThreatOverview]
    enhanced: EnhancedMetadata


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
                    "/vehicles/overview",
                    "/vehicles/{vehicle_id}",
                    "/vehicles/{vehicle_id}/neighbors",
                    "/vehicles/{vehicle_id}/threats",
                    "/bsm",
                    "/bsm/{vehicle_id}",
                    "/cam",
                    "/cam/{station_id}",
                    "/denm",
                    "/denm/{station_id}",
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

        @self.app.get("/vehicles/overview", response_model=List[VehicleOverview])
        async def get_vehicle_overview():
            """Get aggregated data for all vehicles in the network"""
            overview = []
            for vehicle_id in sorted(self.v2v.vehicles.keys()):
                bsm = self.v2v.get_bsm(vehicle_id)
                if not bsm:
                    continue

                neighbors = self._build_neighbors(vehicle_id, bsm)
                threats = self._build_threats_overview(vehicle_id)
                enhanced = self._build_enhanced_metadata(vehicle_id)

                overview.append(VehicleOverview(
                    vehicle_id=vehicle_id,
                    bsm=self._bsm_to_response(bsm),
                    neighbors=neighbors,
                    threats=threats,
                    enhanced=enhanced
                ))
            return overview
        
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

        # ── CAM endpoints ────────────────────────────────────────────────────

        @self.app.get("/cam", response_model=List[CAMResponse])
        async def get_all_cam():
            """Get ETSI ITS-G5 CAM derived from all current BSMs in the network"""
            return [
                self._cam_to_response(create_cam_from_bsm(bsm))
                for bsm in self.v2v.get_all_bsm().values()
            ]

        @self.app.get("/cam/{station_id}", response_model=CAMResponse)
        async def get_cam(station_id: int):
            """Get CAM for a specific ITS station (vehicle)"""
            bsm = self.v2v.get_bsm(station_id)
            if not bsm:
                raise HTTPException(status_code=404, detail="Station not found")
            return self._cam_to_response(create_cam_from_bsm(bsm))

        # ── DENM endpoints ───────────────────────────────────────────────────

        @self.app.get("/denm", response_model=List[DENMResponse])
        async def get_all_denm():
            """Get all active DENM alerts in the network"""
            return [self._denm_to_response(d) for d in self.v2v.get_denm()]

        @self.app.get("/denm/{station_id}", response_model=List[DENMResponse])
        async def get_denm_by_station(station_id: int):
            """Get all active DENMs published by a specific station"""
            denms = self.v2v.get_denm(station_id)
            if not denms:
                raise HTTPException(status_code=404, detail="No DENMs found for station")
            return [self._denm_to_response(d) for d in denms]

        @self.app.post("/denm", response_model=DENMResponse, status_code=201)
        async def publish_denm(body: DENMPublish):
            """Publish a new DENM alert (or update an existing one)"""
            try:
                cause = DENMCauseCode(body.cause_code)
            except ValueError:
                raise HTTPException(
                    status_code=422,
                    detail=f"Unknown cause_code: {body.cause_code}"
                )
            pos = body.event_position
            denm = DENM(
                station_id=body.station_id,
                action_id=body.action_id,
                detection_time=body.detection_time,
                reference_time=body.reference_time,
                event_position=(pos.get("x", 0.0), pos.get("y", 0.0), pos.get("z", 0.0)),
                cause_code=cause,
                subcause_code=body.subcause_code,
                information_quality=body.information_quality,
                relevance_distance=body.relevance_distance,
                relevance_traffic_direction=body.relevance_traffic_direction,
                event_speed=body.event_speed,
                event_heading=body.event_heading,
            )
            self.v2v.publish_denm(denm)
            return self._denm_to_response(denm)

        @self.app.delete("/denm/{station_id}/{action_id}", status_code=204)
        async def cancel_denm(station_id: int, action_id: int):
            """Cancel (terminate) a DENM alert"""
            removed = self.v2v.cancel_denm(station_id, action_id)
            if not removed:
                raise HTTPException(status_code=404, detail="DENM not found")

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
    
    def _cam_to_response(self, cam) -> CAMResponse:
        """Convert CooperativeAwarenessMessage to CAMResponse"""
        pos = cam.reference_position
        return CAMResponse(
            station_id=cam.station_id,
            generation_time=cam.generation_time,
            reference_position={"x": pos[0], "y": pos[1], "z": pos[2]},
            heading=cam.heading,
            speed=cam.speed,
            drive_direction=cam.drive_direction,
            vehicle_role=cam.vehicle_role,
            vehicle_length=cam.vehicle_length,
            vehicle_width=cam.vehicle_width,
        )

    def _denm_to_response(self, denm: DENM) -> DENMResponse:
        """Convert DENM dataclass to DENMResponse"""
        pos = denm.event_position
        return DENMResponse(
            station_id=denm.station_id,
            action_id=denm.action_id,
            detection_time=denm.detection_time,
            reference_time=denm.reference_time,
            event_position={"x": pos[0], "y": pos[1], "z": pos[2]},
            cause_code=int(denm.cause_code),
            cause_code_name=denm.cause_code.name,
            subcause_code=denm.subcause_code,
            information_quality=denm.information_quality,
            relevance_distance=denm.relevance_distance,
            relevance_traffic_direction=denm.relevance_traffic_direction,
            event_speed=denm.event_speed,
            event_heading=denm.event_heading,
            termination=denm.termination,
        )

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
            position_accuracy=bsm.position_accuracy,
            speed=bsm.speed,
            heading=bsm.heading,
            steering_angle=bsm.steering_angle,
            acceleration={
                "longitudinal": bsm.longitudinal_accel,
                "lateral": bsm.lateral_accel,
                "vertical": bsm.vertical_accel
            },
            yaw_rate=bsm.yaw_rate,
            dimensions={
                "length": bsm.vehicle_length,
                "width": bsm.vehicle_width,
                "height": bsm.vehicle_height
            },
            brake_status=BrakingStatus(bsm.brake_status).name,
            brake_pressure=bsm.brake_pressure,
            transmission_state=bsm.transmission_state,
            throttle_confidence=bsm.throttle_confidence,
            brake_confidence=bsm.brake_confidence,
            steering_confidence=bsm.steering_confidence
        )

    def _build_neighbors(self, vehicle_id: int, ego_bsm: BSMCore) -> List[NeighborInfo]:
        """Build NeighborInfo list for a vehicle"""
        neighbor_bsms = self.v2v.get_neighbors(vehicle_id)
        neighbors = []

        for neighbor_bsm in neighbor_bsms:
            distance = self.v2v.get_distance(vehicle_id, neighbor_bsm.vehicle_id)
            rel_speed = abs(neighbor_bsm.speed - ego_bsm.speed)

            neighbors.append(NeighborInfo(
                vehicle_id=neighbor_bsm.vehicle_id,
                distance=distance if distance else 0.0,
                relative_speed=rel_speed,
                bsm=self._bsm_to_response(neighbor_bsm)
            ))

        return neighbors

    def _build_threats_overview(self, vehicle_id: int) -> List[ThreatOverview]:
        """Build a safe threat list with non-finite TTC handled"""
        threats = []
        for threat in self.v2v.get_threats(vehicle_id):
            ttc = threat.get('ttc')
            if ttc is None or not math.isfinite(ttc):
                ttc_value = None
            else:
                ttc_value = ttc

            threats.append(ThreatOverview(
                other_vehicle_id=threat['other_vehicle_id'],
                threat_level=threat['level'],
                time_to_collision=ttc_value,
                distance=threat['distance'],
                timestamp=threat['timestamp']
            ))

        return threats

    def _build_enhanced_metadata(self, vehicle_id: int) -> EnhancedMetadata:
        """Extract enhanced metadata when available"""
        enhanced_msg = None
        if hasattr(self.v2v, 'enhanced_messages'):
            enhanced_msg = self.v2v.enhanced_messages.get(vehicle_id)

        if not enhanced_msg:
            return EnhancedMetadata()

        return EnhancedMetadata(
            transmission_time=enhanced_msg.transmission_time,
            reception_time=enhanced_msg.reception_time,
            link_quality=enhanced_msg.link_quality,
            hop_count=enhanced_msg.hop_count,
            priority=enhanced_msg.priority
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
