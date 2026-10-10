"""BSM (Basic Safety Message), SAE J2735 core data. Positions are CARLA world metres (see section S8)."""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import IntEnum
from typing import Dict, List, Optional


class VehicleType(IntEnum):
    """Vehicle classification"""
    PASSENGER_CAR = 0
    BUS = 1
    TRUCK = 2
    MOTORCYCLE = 3
    EMERGENCY = 4
    UNKNOWN = 99


class BrakingStatus(IntEnum):
    """Braking system status"""
    UNAVAILABLE = 0
    OFF = 1
    ON = 2
    ENGAGED = 3


@dataclass
class BSMCore:
    """
    Basic Safety Message (BSM) - Core Data
    Based on SAE J2735 standard for vehicle-to-vehicle communication.
    
    This is the essential data transmitted at high frequency (rates: see V2XNetwork)
    for cooperative awareness and collision avoidance.
    """
    # Temporal information
    timestamp: float  # Simulation time (seconds)
    msg_count: int  # Message counter (0-127, wraps around)
    
    # Vehicle identification
    vehicle_id: int  # Unique vehicle identifier
    vehicle_type: VehicleType = VehicleType.PASSENGER_CAR
    
    # Position: CARLA world coordinates in metres (NOT geodetic degrees)
    x_m: float = 0.0  # CARLA world x, metres
    y_m: float = 0.0  # CARLA world y, metres
    z_m: float = 0.0  # CARLA world z, metres
    
    # Position accuracy (meters)
    position_accuracy: float = 1.0
    
    # Motion state
    speed: float = 0.0  # m/s
    heading: float = 0.0  # degrees (0-359.9), vehicle yaw (direction it points, not necessarily moves)
    # Velocity vector in CARLA world axes (m/s). None only for hand-made BSMs; the extractor
    # always sets them. Threat assessment uses these, not speed along the heading (S14).
    vx_mps: Optional[float] = None
    vy_mps: Optional[float] = None
    steering_angle: float = 0.0  # degrees
    
    # Acceleration (m/s²)
    longitudinal_accel: float = 0.0
    lateral_accel: float = 0.0
    vertical_accel: float = 0.0
    yaw_rate: float = 0.0  # degrees/second
    
    # Vehicle size (meters)
    vehicle_length: float = 4.5
    vehicle_width: float = 1.8
    vehicle_height: float = 1.5
    
    # Brake status
    brake_status: BrakingStatus = BrakingStatus.UNAVAILABLE
    brake_pressure: float = 0.0  # 0-100%
    
    # Transmission state
    transmission_state: str = "neutral"  # park, reverse, neutral, forward
    
    # Control confidence
    throttle_confidence: float = 100.0  # 0-100%
    brake_confidence: float = 100.0  # 0-100%
    steering_confidence: float = 100.0  # 0-100%


@dataclass
class BSMPartII:
    """
    BSM Part II - Optional Extended Data
    Transmitted less frequently or on-demand for specific scenarios.
    """
    # Path history (breadcrumb trail)
    path_history: List[tuple] = field(default_factory=list)  # [(x, y, time), ...]
    path_prediction: List[tuple] = field(default_factory=list)  # Predicted path
    
    # Special vehicle status
    is_emergency_vehicle: bool = False
    siren_active: bool = False
    lights_active: bool = False
    
    # Weather/road info from sensors
    exterior_lights: str = "off"  # off, low_beam, high_beam, fog
    wiper_status: str = "off"  # off, intermittent, low, high
    
    # Vehicle-to-vehicle specific
    cooperative_status: str = "available"  # available, busy, unavailable


@dataclass
class EnhancedBSMMessage:
    """
    Enhanced BSM combining BSM with sensor data sharing
    For cooperative perception and sensor fusion scenarios.
    """
    # Core BSM data
    bsm: BSMCore
    
    # Part II (optional extended data)
    bsm_part2: Optional[BSMPartII] = None
    
    # Sensor data sharing (cooperative perception)
    detected_objects: List[Dict] = field(default_factory=list)  # Objects detected by sensors
    lidar_points_summary: Optional[Dict] = None  # Downsampled LiDAR data
    camera_detections: List[Dict] = field(default_factory=list)  # Vision-based detections
    
    # Communication metadata
    transmission_time: float = 0.0  # When message was sent
    reception_time: float = 0.0  # When message was received
    link_quality: float = 100.0  # Signal strength (0-100%)
    hop_count: int = 0  # For multi-hop scenarios
    
    # Message priority
    priority: int = 0  # 0=normal, 1=high, 2=emergency
