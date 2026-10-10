"""CARLA actor -> BSM extraction. The only module in v2x that reads CARLA objects."""

from __future__ import annotations

import math
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


def create_bsm_from_carla(vehicle, vehicle_id: int, msg_count: int,
                          snapshot, prev_velocity=None, delta_time=0.05) -> Optional[BSMCore]:
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

    # Calculate speed
    speed_ms = math.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2)
    
    # Calculate accelerations
    if prev_velocity is not None:
        long_accel = (speed_ms - prev_velocity) / delta_time if delta_time > 0 else 0.0
    else:
        long_accel = 0.0
    
    # Get vehicle control
    control = vehicle.get_control()
    
    # Calculate vehicle-specific steering angle
    try:
        physics_control = vehicle.get_physics_control()
        max_steer_angle = physics_control.wheels[0].max_steer_angle
        steering_angle = control.steer * max_steer_angle
    except Exception:
        # Fallback to generic approximation if physics control unavailable
        steering_angle = control.steer * 70.0
    
    # Calculate lateral acceleration (simplified centripetal acceleration)
    if abs(control.steer) > 0.01 and speed_ms > 0.1:
        # Use wheelbase from vehicle dimensions
        wheelbase = vehicle.bounding_box.extent.x * 2 * 0.6  # Approximate 60% of length
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
    
    # Get bounding box for vehicle dimensions
    bbox = vehicle.bounding_box
    vehicle_length = bbox.extent.x * 2
    vehicle_width = bbox.extent.y * 2
    vehicle_height = bbox.extent.z * 2
    
    return BSMCore(
        timestamp=sim_timestamp,
        msg_count=msg_count % 128,
        vehicle_id=vehicle_id,
        vehicle_type=_vehicle_type_from_actor(vehicle),
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
