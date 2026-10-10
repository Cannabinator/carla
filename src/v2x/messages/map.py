"""MAP message (intersection geometry), SAE J2735 / ETSI TS 103 301."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import List


@dataclass
class MAPLane:
    """
    Single lane descriptor inside a MAP intersection.

    Corresponds to SAE J2735 GenericLane.  Minimal set of fields sufficient
    for lane-level routing and SPAT association.
    """
    lane_id: int
    direction: str                        # "ingress" | "egress"
    shared_with: List[int] = field(default_factory=list)  # Connected lane IDs
    speed_limit_ms: float = 13.89         # 50 km/h default


@dataclass
class MAPMessage:
    """
    MAP Data Message — SAE J2735 §7.4 / ETSI TS 103 301 v2.1.1.

    Describes intersection geometry so receiving vehicles can associate SPAT
    phase information with specific approach / departure lanes.  The message
    is transmitted by an RSU and cached by vehicles; it changes only when road
    geometry changes (revision counter bump).

    ref_point carries CARLA world (x, y, z) coordinates — not geodetic.
    """
    intersection_id: int
    revision: int                         # Increments on geometry change
    ref_point: tuple                      # (x, y, z) CARLA world stop-bar centre
    lane_width_m: float = 3.5             # Nominal lane width (m)
    speed_limit_ms: float = 13.89         # 50 km/h default
    lanes: List[MAPLane] = field(default_factory=list)
    timestamp: float = 0.0                # RSU generation time (simulation seconds)
