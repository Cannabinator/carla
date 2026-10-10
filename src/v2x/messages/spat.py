"""SPAT message (signal phase and timing), SAE J2735 / ETSI TS 103 301."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Optional


@dataclass
class SPATMovementState:
    """
    Signal state for one signal group (approach direction).

    Corresponds to SAE J2735 MovementState / ETSI TS 103 301 MovementEvent.

    movement_event values follow SAE J2735 MovementPhaseState strings:
      "unavailable", "dark", "stop-And-Remain", "stop-Then-Proceed",
      "permissiveMovementAllowed", "protectedMovementAllowed",
      "permissiveClearance", "protectedClearance",
      "caution-Conflicting-Traffic"
    """
    signal_group_id: int
    movement_event: str                   # SAE J2735 MovementPhaseState string
    min_end_time_s: float                 # Earliest end of current phase (sim seconds)
    max_end_time_s: float                 # Latest end of current phase (sim seconds)
    likely_time_s: Optional[float] = None # Most-probable end time


@dataclass
class SPATMessage:
    """
    Signal Phase and Timing Message — SAE J2735 §7.18 / ETSI TS 103 301 v2.1.1.

    Broadcast by RSUs at 2 Hz so that approaching vehicles can plan their
    trajectory through an intersection (speed advisory, stop-line timing).
    """
    intersection_id: int
    intersection_name: str
    timestamp: float                      # Simulation time at generation (seconds)
    status: str = "running"               # "running" | "shutdown" | "standby"
    states: List[SPATMovementState] = field(default_factory=list)
