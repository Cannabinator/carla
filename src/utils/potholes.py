"""
Pothole Management for CARLA V2V Research Platform

Visibility:
- CARLA client / spectator: colored bounding boxes drawn via world.debug
  (yellow=minor, orange=moderate, red=severe)
- Web LiDAR viewer: flat disc overlays rendered in the Three.js scene
  via the /api/potholes REST endpoint

Integration::

    mgr = PotholeManager()
    mgr.initialize(world)            # call once after CARLA world is available

    # In main loop (every ~10 frames):
    mgr.draw_debug()

    # REST API / serialization:
    mgr.to_dict_list()               # → List[dict]

    # Spatial query near ego:
    mgr.get_nearby(ego_x, ego_y, radius=50.0)  # → List[PotholeInfo]
"""
from __future__ import annotations

import random
import logging
from dataclasses import dataclass
from typing import List, Dict, Any, Optional

logger = logging.getLogger(__name__)

# ── RGBA colors per severity for CARLA debug drawing ─────────────────────────
_SEVERITY_COLORS_CARLA: Dict[int, tuple] = {
    1: (255, 255,   0, 255),   # Yellow  — minor
    2: (255, 140,   0, 255),   # Orange  — moderate
    3: (255,  30,   0, 255),   # Red     — severe
}

# ── Half-extents [x, y, z] in metres for debug bounding boxes ────────────────
_SEVERITY_EXTENTS: Dict[int, tuple] = {
    1: (0.25, 0.25, 0.06),
    2: (0.35, 0.35, 0.08),
    3: (0.50, 0.50, 0.10),
}

# ── Human-readable severity names ────────────────────────────────────────────
_SEVERITY_NAMES: Dict[int, str] = {1: "minor", 2: "mod", 3: "severe"}


@dataclass
class PotholeInfo:
    """Immutable record for a single pothole."""
    id: int
    x: float
    y: float
    z: float
    radius: float         # visual / query radius in metres
    severity: int         # 1=minor, 2=moderate, 3=severe
    road_id: int = 0
    lane_id: int = 0
    s: float = 0.0        # distance along lane (waypoint.s)

    def to_dict(self) -> Dict[str, Any]:
        """Serialize to plain dict for JSON/REST output."""
        return {
            "id":       self.id,
            "x":        round(self.x, 3),
            "y":        round(self.y, 3),
            "z":        round(self.z, 3),
            "radius":   round(self.radius, 3),
            "severity": self.severity,
            "road_id":  self.road_id,
            "lane_id":  self.lane_id,
            "s":        round(self.s, 3),
        }


class PotholeManager:
    """
    Manages static potholes for CARLA V2V research — BSM communication only.

    This manager provides data structures and utilities for pothole detection
    and V2V communication via BSM messages. It does NOT generate potholes —
    potholes should be pre-placed in your custom CARLA map.

    The manager supports:
      - Visualization via CARLA debug drawing
      - Spatial queries for nearby potholes (for BSM inclusion)
      - REST API serialization for web viewer overlays
      - Pothole proximity event logging

    Usage::

        mgr = PotholeManager()
        # Manually add potholes from your custom map
        mgr.add_pothole(x=100.0, y=50.0, z=0.1, severity=2, radius=0.35)
        
        # main simulation loop
        if frame % 10 == 0:
            mgr.draw_debug(world)

        # expose via FastAPI
        app.get("/api/potholes")(lambda: mgr.to_dict_list())
    """

    def __init__(self) -> None:
        self.potholes: List[PotholeInfo] = []
        self._world = None
        self._initialized: bool = False

    # ── Manual Pothole Registration ───────────────────────────────────────────

    def add_pothole(
        self,
        x: float,
        y: float,
        z: float,
        severity: int = 1,
        radius: Optional[float] = None,
        road_id: int = 0,
        lane_id: int = 0,
        s: float = 0.0,
    ) -> PotholeInfo:
        """
        Register a pothole at the specified location.

        Use this to add potholes that are pre-placed in your custom CARLA map.
        The pothole will be visualized and included in BSM proximity reports.

        Args:
            x, y, z: CARLA world coordinates (metres)
            severity: 1=minor, 2=moderate, 3=severe (default: 1)
            radius: Visual/query radius in metres (auto-set by severity if None)
            road_id, lane_id, s: Map reference (optional, for documentation)

        Returns:
            The created PotholeInfo object
        """
        if radius is None:
            radius = float(_SEVERITY_EXTENTS[severity][0])
        
        pothole = PotholeInfo(
            id=len(self.potholes),
            x=float(x),
            y=float(y),
            z=float(z),
            radius=float(radius),
            severity=severity,
            road_id=road_id,
            lane_id=lane_id,
            s=float(s),
        )
        self.potholes.append(pothole)
        return pothole

    def initialize(self, world) -> None:
        """
        Set the CARLA world reference for debug visualization.

        Call this once after the CARLA world is available if you want
        potholes to be visualized in the CARLA client/spectator.

        Args:
            world: ``carla.World`` instance
        """
        self._world = world
        self._initialized = True
        
        logger.info(
            "PotholeManager: %d potholes registered for BSM communication",
            len(self.potholes),
        )

    # ── CARLA debug visualization ─────────────────────────────────────────────

    def draw_debug(self, life_time: float = 0.6) -> None:
        """
        Draw colored bounding boxes and text labels for all potholes in the
        CARLA client / spectator window.

        Call every 10 simulation frames (0.5 s at 20 FPS) so boxes stay
        visible continuously without flicker.  ``life_time`` is set slightly
        longer than the draw interval.

        Args:
            life_time: Seconds each debug element remains visible.
        """
        if not self._initialized or self._world is None:
            return

        import carla  # lazy import

        debug = self._world.debug

        for ph in self.potholes:
            r, g, b, a = _SEVERITY_COLORS_CARLA[ph.severity]
            color = carla.Color(r, g, b, a)
            ext   = _SEVERITY_EXTENTS[ph.severity]

            center = carla.Location(x=ph.x, y=ph.y, z=ph.z)
            box    = carla.BoundingBox(
                center, carla.Vector3D(ext[0], ext[1], ext[2])
            )

            # Flat box aligned with road plane
            debug.draw_box(
                box,
                carla.Rotation(0, 0, 0),
                thickness=0.04,
                color=color,
                life_time=life_time,
            )

            # Text label slightly above the box
            label_loc = carla.Location(x=ph.x, y=ph.y, z=ph.z + 0.5)
            debug.draw_string(
                label_loc,
                f"PH{ph.id} ({_SEVERITY_NAMES[ph.severity]})",
                draw_shadow=True,
                color=color,
                life_time=life_time,
            )

    # ── Spatial queries ───────────────────────────────────────────────────────

    def get_nearby(self, x: float, y: float, radius: float) -> List[PotholeInfo]:
        """
        Return potholes within *radius* metres of the XY position (x, y).

        Uses 2-D Euclidean distance in the CARLA road plane (ignores Z).

        Args:
            x:      CARLA world X coordinate (metres)
            y:      CARLA world Y coordinate (metres)
            radius: Search radius in metres
        """
        r2 = radius * radius
        return [
            ph for ph in self.potholes
            if (ph.x - x) ** 2 + (ph.y - y) ** 2 <= r2
        ]

    # ── Serialization ─────────────────────────────────────────────────────────

    def to_dict_list(self) -> List[Dict[str, Any]]:
        """Serialize all potholes to a list of dicts for REST API / JSON."""
        return [ph.to_dict() for ph in self.potholes]

    # ── Dunder helpers ────────────────────────────────────────────────────────

    def __len__(self) -> int:
        return len(self.potholes)

    def __repr__(self) -> str:
        return (
            f"PotholeManager(n={len(self.potholes)}, "
            f"initialized={self._initialized})"
        )
