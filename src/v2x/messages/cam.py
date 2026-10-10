"""CAM (Cooperative Awareness Message), ETSI EN 302 637-2."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import List

from .bsm import BSMCore


@dataclass
class CooperativeAwarenessMessage:
    """
    CAM (Cooperative Awareness Message) - European standard
    Similar to BSM but with different structure (ETSI ITS-G5)
    """
    station_id: int  # Unique ITS station ID
    generation_time: float  # Timestamp
    
    # Reference position
    reference_position: tuple  # (lat, lon, alt) or (x, y, z)
    
    # High frequency container
    heading: float
    speed: float
    drive_direction: str  # forward, backward
    
    # Vehicle role
    vehicle_role: str  # default, public_transport, emergency, road_work
    
    # Low frequency container (optional, sent every 500ms)
    vehicle_length: float = 4.5
    vehicle_width: float = 1.8
    
    # Path history
    path_history: List[tuple] = field(default_factory=list)


def create_cam_from_bsm(bsm: 'BSMCore') -> 'CooperativeAwarenessMessage':
    """
    Derive a CAM from an existing BSMCore.

    CAM (ETSI EN 302 637-2) and BSM (SAE J2735) carry equivalent kinematic data.
    This maps the common fields so the API can expose both formats.
    """
    direction = "reverse" if bsm.transmission_state == "reverse" else "forward"
    return CooperativeAwarenessMessage(
        station_id=bsm.vehicle_id,
        generation_time=bsm.timestamp,
        reference_position=(bsm.latitude, bsm.longitude, bsm.elevation),
        heading=bsm.heading,
        speed=bsm.speed,
        drive_direction=direction,
        vehicle_role="default",
        vehicle_length=bsm.vehicle_length,
        vehicle_width=bsm.vehicle_width,
    )
