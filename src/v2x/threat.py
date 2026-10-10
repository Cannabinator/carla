"""Time-to-collision based threat assessment from two BSMs."""

from __future__ import annotations

import math

from .messages.bsm import BSMCore


def _velocity_xy(bsm: BSMCore) -> tuple:
    """World-frame velocity. Uses the BSM's velocity vector; only a hand-made BSM without one
    falls back to speed along the heading (wrong when reversing or sliding)."""
    if bsm.vx_mps is not None and bsm.vy_mps is not None:
        return bsm.vx_mps, bsm.vy_mps
    h = math.radians(bsm.heading)
    return bsm.speed * math.cos(h), bsm.speed * math.sin(h)


def calculate_threat_level(ego_bsm: BSMCore, other_bsm: BSMCore) -> tuple:
    """
    Collision threat level between two vehicles from their BSMs.

    Time to collision uses the closing speed along the line of sight between the two reference
    points, computed from the velocity vectors (S14), so reversing or sliding vehicles are
    handled. Limitations: the distance is centre to centre (vehicle extents are ignored, so TTC
    is overestimated), and the level thresholds (TTC 10 / 5 / 2 s, range 100 m) are
    PROJECT-DEFINED, not taken from a standard.

    Args:
        ego_bsm: Ego vehicle BSM
        other_bsm: Other vehicle BSM

    Returns:
        (threat_level, time_to_collision, distance)
        threat_level: 0=no threat, 1=low, 2=medium, 3=high, 4=imminent
    """
    # Calculate distance
    dx = other_bsm.x_m - ego_bsm.x_m
    dy = other_bsm.y_m - ego_bsm.y_m
    distance = math.sqrt(dx**2 + dy**2)

    # Relative velocity from the velocity vectors
    ego_vx, ego_vy = _velocity_xy(ego_bsm)
    other_vx, other_vy = _velocity_xy(other_bsm)

    rel_vx = other_vx - ego_vx
    rel_vy = other_vy - ego_vy

    if distance > 1e-6:
        # Use radial closing speed along line of sight, not relative speed magnitude.
        ux = dx / distance
        uy = dy / distance
        closing_speed = -(rel_vx * ux + rel_vy * uy)
    else:
        closing_speed = float('inf')
    
    # Time to collision (TTC)
    if closing_speed > 0.1:  # Positive means vehicles are converging
        ttc = distance / closing_speed
    else:
        ttc = float('inf')
    
    # Determine threat level
    if distance > 100 or closing_speed <= 0.1:
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
