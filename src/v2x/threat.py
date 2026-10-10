"""Time-to-collision based threat assessment from two BSMs."""

from __future__ import annotations

import math

from .messages.bsm import BSMCore


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
    dx = other_bsm.x_m - ego_bsm.x_m
    dy = other_bsm.y_m - ego_bsm.y_m
    distance = math.sqrt(dx**2 + dy**2)
    
    # Calculate relative velocity
    ego_vx = ego_bsm.speed * math.cos(math.radians(ego_bsm.heading))
    ego_vy = ego_bsm.speed * math.sin(math.radians(ego_bsm.heading))
    
    other_vx = other_bsm.speed * math.cos(math.radians(other_bsm.heading))
    other_vy = other_bsm.speed * math.sin(math.radians(other_bsm.heading))
    
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
