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

# ── Fixed severity distribution for 15 potholes ──────────────────────────────
# 5 minor | 5 moderate | 5 severe — balanced for research data collection
_SEVERITY_PROFILE: List[int] = [1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3]

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
    Manages 15 static, deterministic road potholes for CARLA V2V research.

    A dedicated seed (``POTHOLE_SEED = 12345``) is used so pothole positions
    are always identical regardless of the main simulation seed.  The manager
    does **not** spawn physical actors — potholes are purely logical objects
    visualised via CARLA debug boxes and a web-viewer REST overlay.

    Example::

        mgr = PotholeManager()
        mgr.initialize(world)

        # main simulation loop
        if frame % 10 == 0:
            mgr.draw_debug()

        # expose via FastAPI
        app.get("/api/potholes")(lambda: mgr.to_dict_list())
    """

    NUM_POTHOLES: int = 15
    POTHOLE_SEED: int = 12345  # independent of main simulation seed

    def __init__(self) -> None:
        self.potholes: List[PotholeInfo] = []
        self._world = None
        self._initialized: bool = False

    # ── Initialization ────────────────────────────────────────────────────────

    def initialize(self, world) -> None:
        """
        Compute 15 static pothole positions from map waypoints using a fixed seed.

        Selects non-junction, drivable-lane waypoints deterministically.
        Call once after the CARLA world is available.

        Args:
            world: ``carla.World`` instance
        """
        import carla  # lazy import — carla may not be installed in all envs

        self._world = world
        map_ = world.get_map()

        rng = random.Random(self.POTHOLE_SEED)
        all_waypoints = map_.generate_waypoints(4.0)  # one waypoint every 4 m

        # Prefer straight, drivable, non-junction sections
        road_wps = [
            wp for wp in all_waypoints
            if not wp.is_junction
            and wp.lane_type == carla.LaneType.Driving
        ]

        # Fallback: any non-junction waypoint if filter yields too few
        if len(road_wps) < self.NUM_POTHOLES:
            logger.warning(
                "LaneType.Driving filter returned %d waypoints (need %d) — "
                "falling back to all non-junction waypoints.",
                len(road_wps), self.NUM_POTHOLES
            )
            road_wps = [wp for wp in all_waypoints if not wp.is_junction]

        if len(road_wps) < self.NUM_POTHOLES:
            logger.warning(
                "Only %d valid waypoints found; will generate %d potholes.",
                len(road_wps), len(road_wps)
            )

        rng.shuffle(road_wps)
        selected = road_wps[:self.NUM_POTHOLES]
        severities = _SEVERITY_PROFILE[:len(selected)]

        for i, (wp, sev) in enumerate(zip(selected, severities)):
            loc = wp.transform.location
            ext = _SEVERITY_EXTENTS[sev]
            self.potholes.append(PotholeInfo(
                id=i,
                x=float(loc.x),
                y=float(loc.y),
                z=float(loc.z) + 0.05,   # 5 cm above road surface
                radius=float(ext[0]),
                severity=sev,
                road_id=wp.road_id,
                lane_id=wp.lane_id,
                s=float(wp.s),
            ))

        n_minor    = sum(1 for p in self.potholes if p.severity == 1)
        n_moderate = sum(1 for p in self.potholes if p.severity == 2)
        n_severe   = sum(1 for p in self.potholes if p.severity == 3)
        self._initialized = True

        logger.info(
            "PotholeManager: %d potholes initialized (seed=%d) — "
            "%d minor / %d moderate / %d severe",
            len(self.potholes), self.POTHOLE_SEED,
            n_minor, n_moderate, n_severe,
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
            f"initialized={self._initialized}, seed={self.POTHOLE_SEED})"
        )
