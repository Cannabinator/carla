"""DENM (Decentralized Environmental Notification Message), ETSI EN 302 637-3."""

from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum
from typing import Optional


class DENMCauseCode(IntEnum):
    """
    DENM cause codes — ETSI EN 302 637-3 Table 2.
    Describes the type of environmental notification event.
    """
    RESERVED = 0
    TRAFFIC_CONDITION = 1
    ACCIDENT = 2
    ROAD_WORKS = 3
    ADVERSE_WEATHER_ADHESION = 6
    HAZARDOUS_SURFACE = 9
    OBSTACLE_ON_ROAD = 10
    ANIMAL_ON_ROAD = 11
    HUMAN_PRESENCE_ON_ROAD = 14
    WRONG_WAY_DRIVING = 17
    RESCUE_RECOVERY_IN_PROGRESS = 18
    SLOW_VEHICLE = 26
    VEHICLE_BREAKDOWN = 91
    POST_CRASH = 92
    HUMAN_PROBLEM = 93
    STATIONARY_VEHICLE = 94
    EMERGENCY_VEHICLE_APPROACHING = 95
    DANGEROUS_CURVE = 96
    COLLISION_RISK = 97
    SIGNAL_VIOLATION = 98
    DANGEROUS_END_OF_QUEUE = 99


@dataclass
class DENM:
    """
    Decentralized Environmental Notification Message (DENM)
    Based on ETSI EN 302 637-3 — ITS Vehicular Communications.

    Used to alert nearby vehicles of hazardous or relevant road events.
    """
    # Management container
    station_id: int          # Originating ITS station ID (maps to vehicle_id)
    action_id: int           # Sequence number at originating station
    detection_time: float    # Simulation time when event was detected (seconds)
    reference_time: float    # Time of last update (seconds)
    event_position: tuple    # (x, y, z) in local CARLA coordinates

    # Situation container
    cause_code: DENMCauseCode = DENMCauseCode.RESERVED
    subcause_code: int = 0   # Standard-defined per cause_code
    information_quality: int = 0  # 0 (unavailable) – 7 (fully safe/certain)

    # Location / relevance
    relevance_distance: float = 150.0    # metres; default = MEDIUM range
    relevance_traffic_direction: str = "allTrafficDirections"
    event_speed: Optional[float] = None  # m/s at event location
    event_heading: Optional[float] = None  # degrees at event location

    # Termination flag (None = active, "isCancellation" = revoked)
    termination: Optional[str] = None
