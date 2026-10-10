"""CARLA actor -> BSM extraction. The only module in v2x that reads CARLA objects."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional

from .messages.bsm import BrakingStatus, BSMCore, VehicleType


# ── Vehicle-type detection from CARLA blueprint ID ────────────────────────────
# Maps substrings of vehicle.type_id to VehicleType.
# Checked in order; first match wins.  "passenger_car" catch-all is not listed
# here — any 4-wheeled blueprint that misses all patterns falls back to
# PASSENGER_CAR in _vehicle_type_from_actor().
_BLUEPRINT_TYPE_MAP: list = [
    # Emergency
    ("ambulance",           VehicleType.EMERGENCY),
    ("firetruck",           VehicleType.EMERGENCY),
    ("police",              VehicleType.EMERGENCY),
    # Buses / large passenger vans
    ("fusorosa",            VehicleType.BUS),      # Mitsubishi Fuso Rosa (full bus)
    ("carlacola",           VehicleType.BUS),      # CarlaMotors generic bus
    (".t2",                 VehicleType.BUS),      # VW Transporter T2 / T2_2021 (minibus)
    # Trucks / utility
    ("cybertruck",          VehicleType.TRUCK),    # Tesla Cybertruck
    ("sprinter",            VehicleType.TRUCK),    # Mercedes Sprinter
    # Motorcycles / bicycles / micro-vehicles
    ("harley",              VehicleType.MOTORCYCLE),
    ("kawasaki",            VehicleType.MOTORCYCLE),
    ("vespa",               VehicleType.MOTORCYCLE),
    ("yamaha",              VehicleType.MOTORCYCLE),
    ("ninja",               VehicleType.MOTORCYCLE),
    ("crossbike",           VehicleType.MOTORCYCLE),
    ("century",             VehicleType.MOTORCYCLE),  # diamondback.century
    ("omafiets",            VehicleType.MOTORCYCLE),  # gazelle.omafiets bicycle
    ("low_rider",           VehicleType.MOTORCYCLE),
]


def _vehicle_type_from_actor(vehicle) -> 'VehicleType':
    """
    Derive VehicleType from the CARLA actor's blueprint ID.

    Uses vehicle.type_id (e.g. 'vehicle.ford.ambulance') to classify the
    vehicle.  Falls back to actor.attributes['number_of_wheels'] (a cached
    blueprint dict — no RPC call) so that any two-wheeled blueprint not yet
    in _BLUEPRINT_TYPE_MAP is still classified as MOTORCYCLE rather than
    PASSENGER_CAR.

    Args:
        vehicle: CARLA vehicle actor with .type_id attribute.

    Returns:
        VehicleType enum value.
    """
    type_id = getattr(vehicle, 'type_id', '').lower()

    for fragment, vtype in _BLUEPRINT_TYPE_MAP:
        if fragment in type_id:
            return vtype

    # Wheel-count fallback: two-wheeled vehicles not caught by name lookup.
    # Uses actor.attributes (a cached blueprint dict — no RPC call) rather than
    # get_physics_control() which is a blocking server round-trip.
    try:
        num_wheels = int(getattr(vehicle, 'attributes', {}).get('number_of_wheels', '4'))
        if num_wheels <= 2:
            return VehicleType.MOTORCYCLE
    except (ValueError, TypeError):
        pass

    return VehicleType.PASSENGER_CAR


DEFAULT_MAX_STEER_DEG = 70.0  # used only if the physics control cannot be read


@dataclass(frozen=True)
class VehicleStatic:
    """Facts about a vehicle that never change while it lives (read once, at registration)."""
    vehicle_type: VehicleType
    length_m: float
    width_m: float
    height_m: float
    max_steer_angle_deg: float


def describe_vehicle(vehicle) -> VehicleStatic:
    """Read the unchanging properties of a vehicle. Does one blocking RPC (physics control)."""
    try:
        max_steer = vehicle.get_physics_control().wheels[0].max_steer_angle
    except Exception:
        max_steer = DEFAULT_MAX_STEER_DEG
    bbox = vehicle.bounding_box
    return VehicleStatic(
        vehicle_type=_vehicle_type_from_actor(vehicle),
        length_m=bbox.extent.x * 2,
        width_m=bbox.extent.y * 2,
        height_m=bbox.extent.z * 2,
        max_steer_angle_deg=max_steer,
    )


def create_bsm_from_carla(vehicle, vehicle_id: int, msg_count: int,
                          snapshot, prev_velocity=None, delta_time=0.05,
                          static: Optional[VehicleStatic] = None) -> Optional[BSMCore]:
    """
    Create a BSM from a CARLA vehicle, using only data of one WorldSnapshot.

    ``vehicle.get_velocity()`` returns data cached from the previous tick, which is wrong in
    synchronous simulation, so kinematics and the timestamp come from the snapshot only.

    Returns ``None`` if the vehicle is not in the snapshot (for example destroyed this tick).
    There is deliberately no fallback to RPC state or wall-clock time: mixing clocks in one
    data set is worse than skipping a station for one frame (defect S10). The caller counts
    the skip.

    Args:
        vehicle: CARLA vehicle actor
        vehicle_id: Unique vehicle identifier
        msg_count: Message counter
        snapshot: WorldSnapshot (required)
        prev_velocity: Previous speed for the acceleration estimate
        delta_time: Time since last update
        static: Cached VehicleStatic; computed on the spot (one RPC) if omitted

    Returns:
        BSMCore instance, or None if the vehicle is missing from the snapshot
    """
    if snapshot is None:
        return None
    actor_snapshot = snapshot.find(vehicle.id)
    if actor_snapshot is None:
        return None
    transform = actor_snapshot.get_transform()
    velocity = actor_snapshot.get_velocity()
    angular_velocity = actor_snapshot.get_angular_velocity()
    sim_timestamp = float(snapshot.timestamp.elapsed_seconds)
    if static is None:
        static = describe_vehicle(vehicle)

    # Calculate speed
    speed_ms = math.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2)
    
    # Calculate accelerations
    if prev_velocity is not None:
        long_accel = (speed_ms - prev_velocity) / delta_time if delta_time > 0 else 0.0
    else:
        long_accel = 0.0
    
    # Get vehicle control
    control = vehicle.get_control()
    
    # Vehicle-specific steering angle (max steer angle is cached, S11)
    steering_angle = control.steer * static.max_steer_angle_deg

    # Calculate lateral acceleration (simplified centripetal acceleration)
    if abs(control.steer) > 0.01 and speed_ms > 0.1:
        # Use wheelbase from vehicle dimensions
        wheelbase = static.length_m * 0.6  # Approximate 60% of length
        try:
            turn_radius = wheelbase / abs(math.tan(math.radians(steering_angle)))
            lateral_accel = (speed_ms ** 2) / turn_radius * math.copysign(1, control.steer)
        except (ZeroDivisionError, ValueError):
            lateral_accel = 0.0
    else:
        lateral_accel = 0.0
    
    # Yaw rate in deg/s. CARLA's get_angular_velocity() already returns deg/s, so no conversion
    # (the old math.degrees() inflated it by 57.3x, defect S2).
    # TODO(verify): confirm the unit against the 0.9.16 Python API reference / a live yaw test.
    yaw_rate = angular_velocity.z
    
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
    
    return BSMCore(
        timestamp=sim_timestamp,
        msg_count=msg_count % 128,
        vehicle_id=vehicle_id,
        vehicle_type=static.vehicle_type,
        x_m=transform.location.x,
        y_m=transform.location.y,
        z_m=transform.location.z,
        position_accuracy=0.5,  # Assuming high accuracy in simulation
        speed=speed_ms,
        heading=transform.rotation.yaw % 360,
        steering_angle=steering_angle,
        longitudinal_accel=long_accel,
        lateral_accel=lateral_accel,
        vertical_accel=0.0,  # Road vehicles typically don't need vertical accel
        yaw_rate=yaw_rate,
        vehicle_length=static.length_m,
        vehicle_width=static.width_m,
        vehicle_height=static.height_m,
        brake_status=brake_status,
        brake_pressure=control.brake * 100,
        transmission_state=trans_state,
        throttle_confidence=100.0,
        brake_confidence=100.0,
        steering_confidence=100.0
    )
