"""VRUAM (VRU Awareness Message), ETSI TS 103 300-3."""

from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum
from typing import Optional


class VRUProfile(IntEnum):
    """
    VRU device / user classification.

    Maps to ETSI TS 103 300-3 VRU Profile values.
    """
    PEDESTRIAN   = 0   # ITS-S on foot
    CYCLIST      = 1   # Bicycle, e-bike, cargo-bike
    MOTORCYCLIST = 2   # Powered two-wheeled vehicle (≤ 400 cc)
    ANIMAL       = 3   # Any animal on carriageway
    OTHER        = 255


@dataclass
class VRUAwarenessMessage:
    """
    VRU Awareness Message — ETSI TS 103 300-3 v2.1.1.

    Broadcast by Vulnerable Road User ITS stations (pedestrians, cyclists,
    motorcyclists).  The VRU basic service runs at 2–25 Hz depending on motion
    state; this simulation uses 2 Hz to match the BSM/CAM beacon cadence.

    position carries CARLA world (x, y, z) coordinates.
    """
    station_id: int
    timestamp: float
    vru_profile: VRUProfile
    position: tuple                       # (x, y, z) CARLA world coordinates
    speed: float = 0.0                    # m/s
    heading: float = 0.0                  # degrees (world frame)
    # Cluster membership — ETSI TS 103 300-3 §7.5
    cluster_id: Optional[int] = None
    is_cluster_leader: bool = False
    # Carriageway context
    is_on_road: bool = False              # True when VRU crosses carriageway
    # Motion state
    motion_state: str = "moving"          # "stationary" | "moving" | "stopped"
