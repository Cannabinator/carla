"""CPM (Collective Perception Message), ETSI TS 103 324."""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import IntEnum
from typing import Dict, List


class ObjectClassification(IntEnum):
    """Perceived object type as reported in a CPM."""
    UNKNOWN    = 0
    PEDESTRIAN = 1
    CYCLIST    = 2
    VEHICLE    = 3
    ANIMAL     = 4
    OBSTACLE   = 5


@dataclass
class PerceivedObject:
    """
    Single object entry in a Collective Perception Message.

    Corresponds to ETSI TS 103 324 PerceivedObject.  Position is relative to
    the sender in CARLA world-coordinate deltas (metres): dx = East, dy = North.
    heading and speed are absolute (world frame).
    """
    object_id: int
    object_type: ObjectClassification
    # Relative position from sender (CARLA world delta, metres)
    dx: float
    dy: float
    dz: float = 0.0
    # Absolute kinematics
    speed: float = 0.0                    # m/s
    heading: float = 0.0                  # degrees (world frame)
    # Bounding box (half-extents)
    length_m: float = 0.5
    width_m: float = 0.5
    # Detection confidence [0.0–1.0]
    confidence: float = 1.0
    # Measurement age (seconds since last sensor update)
    measurement_age_s: float = 0.0


@dataclass
class SensorInfo:
    """
    Sensor metadata included in a CPM sender's perception block.

    Corresponds to ETSI TS 103 324 SensorInformation.
    """
    sensor_id: int
    sensor_type: str                      # "lidar" | "radar" | "camera" | "fusion"
    detection_range_m: float = 100.0
    horizontal_fov_deg: float = 360.0


@dataclass
class CPMMessage:
    """
    Collective Perception Message — ETSI TS 103 324 v2.1.1.

    Transmitted by any equipped ITS station (vehicle or RSU) to share
    sensor-detected objects with nearby stations.  Complements BSM/CAM
    by distributing an environment model beyond the sender's own state.

    sender_position carries CARLA world (x, y, z) coordinates.
    """
    station_id: int
    timestamp: float                      # Simulation time (seconds)
    sender_position: tuple                # (x, y, z) CARLA world coords
    management_container: Dict = field(default_factory=dict)
    perceived_objects: List[PerceivedObject] = field(default_factory=list)
    sensor_info_list: List[SensorInfo] = field(default_factory=list)
