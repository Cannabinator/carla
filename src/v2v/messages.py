"""
V2V Message Standards - BSM (Basic Safety Message) Implementation
Based on SAE J2735 and ETSI ITS-G5 standards for V2V communication.
"""

from dataclasses import dataclass, field
from typing import List, Optional, Dict
from enum import IntEnum
import time
import math


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
    Based on SAE J2735 standard for V2V communication.
    
    This is the essential data transmitted at high frequency (2-10 Hz)
    for cooperative awareness and collision avoidance.
    """
    # Temporal information
    timestamp: float  # Simulation time (seconds)
    msg_count: int  # Message counter (0-127, wraps around)
    
    # Vehicle identification
    vehicle_id: int  # Unique vehicle identifier
    vehicle_type: VehicleType = VehicleType.PASSENGER_CAR
    
    # Position (WGS84 or local coordinate system)
    latitude: float = 0.0  # or X in local coords
    longitude: float = 0.0  # or Y in local coords
    elevation: float = 0.0  # or Z in local coords
    
    # Position accuracy (meters)
    position_accuracy: float = 1.0
    
    # Motion state
    speed: float = 0.0  # m/s
    heading: float = 0.0  # degrees (0-359.9)
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
    
    # V2V specific
    cooperative_status: str = "available"  # available, busy, unavailable


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


@dataclass
class V2VEnhancedMessage:
    """
    Enhanced V2V message combining BSM with sensor data sharing
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


def calculate_threat_level(ego_bsm: BSMCore, other_bsm: BSMCore) -> tuple:
    """
    Calculate collision threat level between two vehicles.
    
    Args:
        ego_bsm: Ego vehicle BSM
        other_bsm: Other vehicle BSM
    
    Returns:
        (threat_level, time_to_collision, distance)
        threat_level: 0=no threat, 1=low, 2=medium, 3=high, 4=imminent
    """
    # Calculate distance
    dx = other_bsm.latitude - ego_bsm.latitude
    dy = other_bsm.longitude - ego_bsm.longitude
    distance = math.sqrt(dx**2 + dy**2)
    
    # Calculate relative velocity
    ego_vx = ego_bsm.speed * math.cos(math.radians(ego_bsm.heading))
    ego_vy = ego_bsm.speed * math.sin(math.radians(ego_bsm.heading))
    
    other_vx = other_bsm.speed * math.cos(math.radians(other_bsm.heading))
    other_vy = other_bsm.speed * math.sin(math.radians(other_bsm.heading))
    
    rel_vx = other_vx - ego_vx
    rel_vy = other_vy - ego_vy
    rel_speed = math.sqrt(rel_vx**2 + rel_vy**2)
    
    # Time to collision (TTC)
    if rel_speed > 0.1:  # Avoid division by zero
        ttc = distance / rel_speed
    else:
        ttc = float('inf')
    
    # Determine threat level
    if distance > 100:
        threat = 0  # No threat
    elif ttc > 10:
        threat = 1  # Low threat
    elif ttc > 5:
        threat = 2  # Medium threat
    elif ttc > 2:
        threat = 3  # High threat
    else:
        threat = 4  # Imminent collision
    
    return (threat, ttc, distance)


def create_bsm_from_carla(vehicle, vehicle_id: int, msg_count: int, 
                          prev_velocity=None, delta_time=0.05) -> BSMCore:
    """
    Create BSM message from CARLA vehicle actor.
    
    Args:
        vehicle: CARLA vehicle actor
        vehicle_id: Unique vehicle identifier
        msg_count: Message counter
        prev_velocity: Previous velocity for accel calculation
        delta_time: Time since last update
    
    Returns:
        BSMCore instance
    """
    transform = vehicle.get_transform()
    velocity = vehicle.get_velocity()
    
    # Calculate speed
    speed_ms = math.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2)
    
    # Calculate accelerations
    if prev_velocity:
        long_accel = (speed_ms - prev_velocity) / delta_time if delta_time > 0 else 0.0
    else:
        long_accel = 0.0
    
    # Get vehicle control
    control = vehicle.get_control()
    
    # Determine brake status
    if control.brake > 0.5:
        brake_status = BrakingStatus.ENGAGED
    elif control.brake > 0.1:
        brake_status = BrakingStatus.ON
    else:
        brake_status = BrakingStatus.OFF
    
    # Determine transmission state
    if control.reverse:
        trans_state = "reverse"
    elif control.throttle > 0:
        trans_state = "forward"
    else:
        trans_state = "neutral"
    
    # Get bounding box for vehicle dimensions
    bbox = vehicle.bounding_box
    vehicle_length = bbox.extent.x * 2
    vehicle_width = bbox.extent.y * 2
    vehicle_height = bbox.extent.z * 2
    
    return BSMCore(
        timestamp=time.time(),
        msg_count=msg_count % 128,
        vehicle_id=vehicle_id,
        vehicle_type=VehicleType.PASSENGER_CAR,
        latitude=transform.location.x,
        longitude=transform.location.y,
        elevation=transform.location.z,
        position_accuracy=0.5,  # Assuming high accuracy in simulation
        speed=speed_ms,
        heading=transform.rotation.yaw % 360,
        steering_angle=control.steer * 70,  # Approximate max steering angle
        longitudinal_accel=long_accel,
        lateral_accel=0.0,  # Would need more complex calculation
        vertical_accel=0.0,
        yaw_rate=0.0,  # Would need angular velocity
        vehicle_length=vehicle_length,
        vehicle_width=vehicle_width,
        vehicle_height=vehicle_height,
        brake_status=brake_status,
        brake_pressure=control.brake * 100,
        transmission_state=trans_state,
        throttle_confidence=100.0,
        brake_confidence=100.0,
        steering_confidence=100.0
    )


# Message priority constants
PRIORITY_ROUTINE = 0
PRIORITY_HIGH = 1
PRIORITY_EMERGENCY = 2

# Transmission rates (Hz)
BSM_RATE_NORMAL = 2  # 2 Hz for normal driving
BSM_RATE_HIGH = 10  # 10 Hz for critical situations
CAM_RATE = 2  # 2 Hz for CAM messages

# Communication range (meters)
V2V_RANGE_SHORT = 50
V2V_RANGE_MEDIUM = 150
V2V_RANGE_LONG = 300

# Data sharing thresholds
SHARE_SENSOR_DATA_DISTANCE = 50  # Share sensor data within 50m
SHARE_LIDAR_DISTANCE = 30  # Share LiDAR data within 30m
EMERGENCY_BROADCAST_DISTANCE = 300  # Emergency messages go further
