#!/usr/bin/env python3
"""Empirical check: can the project's semantic LiDAR sensor detect the
baked pothole geometry on Town10HD_Potholes_Deeper?

Ground truth comes from ``world.get_environment_objects()`` (the actual
placed mesh actors), not from the build-time JSON, so this reflects the
real cooked map. For each pothole the sensor is teleported to several
standoff distances along the lane heading and one full LiDAR revolution
is captured; a hit counts if any returned point's ``object_idx`` matches
the pothole's environment-object id (CARLA's native static-object tag)
OR falls geometrically inside the pothole's bounding footprint with a
z-drop below the local road surface.

Requires a running CARLA server with Town10HD_Potholes_Deeper loaded
(see CarlaServer/start_deeper_map.sh) and the project's production LiDAR
config: channels=32, points_per_second=500000, range=100, rotation_frequency=20,
upper_fov=15, lower_fov=-30 (see src/server/persistent_world.py enable_client_lidar).

Usage: python3 scripts/lidar_pothole_detection_check.py [--host H] [--port P]
"""
from __future__ import annotations

import argparse
import csv
import math
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import List, Optional

import carla
import numpy as np

LIDAR_CONFIG = {
    "channels": "32",
    "range": "100.0",
    "points_per_second": "500000",
    "rotation_frequency": "20.0",
    "upper_fov": "15.0",
    "lower_fov": "-30.0",
    "horizontal_fov": "360.0",
}
SENSOR_HEIGHT_M = 2.4  # matches vehicle-roof mount in collector.py
TEST_DISTANCES_M = [5.0, 10.0, 15.0, 20.0, 25.0, 30.0]
GEOMETRIC_MARGIN_M = 0.3   # extra radius around bbox footprint
Z_DROP_THRESHOLD_M = 0.03  # min depth below road surface to count as geometric hit
STRUCT_DTYPE = np.dtype([
    ("x", np.float32), ("y", np.float32), ("z", np.float32),
    ("cos_inc_angle", np.float32), ("object_idx", np.uint32), ("object_tag", np.uint32),
])


@dataclass
class PotholeGT:
    id: int
    name: str
    x: float
    y: float
    z: float
    extent_x: float
    extent_y: float


@dataclass
class DistanceResult:
    distance_m: float
    total_points: int
    tag_hits: int
    geo_hits: int
    min_range_m: Optional[float] = None


@dataclass
class PotholeResult:
    gt: PotholeGT
    per_distance: List[DistanceResult] = field(default_factory=list)

    @property
    def detected_any(self) -> bool:
        return any(d.tag_hits or d.geo_hits for d in self.per_distance)


class FrameCapture:
    def __init__(self):
        self.frame_number: Optional[int] = None
        self.transform: Optional[carla.Transform] = None
        self.points: Optional[np.ndarray] = None

    def __call__(self, data: "carla.SemanticLidarMeasurement") -> None:
        self.frame_number = data.frame
        self.transform = data.transform
        self.points = np.frombuffer(data.raw_data, dtype=STRUCT_DTYPE).copy()


def world_points(local: np.ndarray, transform: carla.Transform) -> np.ndarray:
    """Transform local LiDAR points into world XYZ (same math as collector.py)."""
    loc, rot = transform.location, transform.rotation
    yaw, pitch, roll = map(math.radians, (rot.yaw, rot.pitch, rot.roll))
    cy, sy = math.cos(yaw), math.sin(yaw)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cr, sr = math.cos(roll), math.sin(roll)
    rot_m = np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])
    xyz = np.column_stack([local["x"], local["y"], local["z"]])
    out = (rot_m @ xyz.T).T
    out[:, 0] += loc.x
    out[:, 1] += loc.y
    out[:, 2] += loc.z
    return out


def find_potholes(world: carla.World) -> List[PotholeGT]:
    objs = world.get_environment_objects(carla.CityObjectLabel.Any)
    found = [o for o in objs if "pothole" in o.name.lower()]
    out = []
    for o in found:
        bb = o.bounding_box
        out.append(PotholeGT(
            id=o.id, name=o.name,
            x=o.transform.location.x, y=o.transform.location.y, z=o.transform.location.z,
            extent_x=max(bb.extent.x, 0.2), extent_y=max(bb.extent.y, 0.2),
        ))
    return out


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=2000)
    args = ap.parse_args()

    client = carla.Client(args.host, args.port)
    client.set_timeout(30.0)
    world = client.get_world()
    map_name = world.get_map().name
    print(f"Connected. Active map: {map_name}")
    if "Potholes" not in map_name:
        print("WARNING: active map does not look like a pothole map.")

    potholes = find_potholes(world)
    print(f"Ground-truth pothole environment objects found: {len(potholes)}")
    if not potholes:
        print("No 'pothole' environment objects found — aborting.")
        return 1

    original_settings = world.get_settings()
    settings = world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = 0.05
    world.apply_settings(settings)

    bp_lib = world.get_blueprint_library()
    lidar_bp = bp_lib.find("sensor.lidar.ray_cast_semantic")
    for k, v in LIDAR_CONFIG.items():
        lidar_bp.set_attribute(k, v)

    spawn_tf = carla.Transform(carla.Location(x=potholes[0].x, y=potholes[0].y,
                                               z=potholes[0].z + SENSOR_HEIGHT_M))
    sensor = world.spawn_actor(lidar_bp, spawn_tf)
    capture = FrameCapture()
    sensor.listen(capture)

    results: List[PotholeResult] = []
    town_map = world.get_map()

    try:
        for gt in potholes:
            wp = town_map.get_waypoint(carla.Location(x=gt.x, y=gt.y, z=gt.z),
                                        project_to_road=True)
            yaw_rad = math.radians(wp.transform.rotation.yaw) if wp else 0.0
            fwd = (math.cos(yaw_rad), math.sin(yaw_rad))
            road_z = wp.transform.location.z if wp else gt.z

            pr = PotholeResult(gt=gt)
            for dist in TEST_DISTANCES_M:
                sx = gt.x - fwd[0] * dist
                sy = gt.y - fwd[1] * dist
                tf = carla.Transform(
                    carla.Location(x=sx, y=sy, z=road_z + SENSOR_HEIGHT_M),
                    carla.Rotation(yaw=wp.transform.rotation.yaw if wp else 0.0),
                )
                sensor.set_transform(tf)
                world.tick()
                world.tick()  # settle: ensure a fresh full revolution after teleport

                if capture.points is None:
                    pr.per_distance.append(DistanceResult(dist, 0, 0, 0))
                    continue

                pts_world = world_points(capture.points, capture.transform)
                dx = pts_world[:, 0] - gt.x
                dy = pts_world[:, 1] - gt.y
                horiz2 = dx * dx + dy * dy
                radius = max(gt.extent_x, gt.extent_y) + GEOMETRIC_MARGIN_M
                geo_mask = horiz2 <= radius * radius
                z_drop = (road_z - pts_world[:, 2]) >= Z_DROP_THRESHOLD_M
                geo_hits = int(np.count_nonzero(geo_mask & z_drop))
                tag_hits = int(np.count_nonzero(capture.points["object_idx"] == gt.id))

                min_range = None
                any_mask = geo_mask | (capture.points["object_idx"] == gt.id)
                if np.any(any_mask):
                    rng = np.sqrt(dx[any_mask] ** 2 + dy[any_mask] ** 2)
                    min_range = float(np.min(rng))

                pr.per_distance.append(DistanceResult(
                    dist, int(len(capture.points)), tag_hits, geo_hits, min_range,
                ))
            results.append(pr)
            pts_str = ",".join(str(d.total_points) for d in pr.per_distance)
            print(f"  pothole id={gt.id:>6} '{gt.name}' -> "
                  f"{'DETECTED' if pr.detected_any else 'missed'} "
                  f"@ distances {[d.distance_m for d in pr.per_distance if d.tag_hits or d.geo_hits]} "
                  f"(points/frame: {pts_str})")
    finally:
        sensor.destroy()
        world.apply_settings(original_settings)

    # ── Summary ──────────────────────────────────────────────────────────
    n_detected = sum(1 for r in results if r.detected_any)
    print(f"\n=== SUMMARY: {n_detected}/{len(results)} potholes detected at >=1 distance ===")
    by_dist_hits = {d: 0 for d in TEST_DISTANCES_M}
    for r in results:
        for dr in r.per_distance:
            if dr.tag_hits or dr.geo_hits:
                by_dist_hits[dr.distance_m] += 1
    for d in TEST_DISTANCES_M:
        print(f"  {d:5.1f} m ahead: {by_dist_hits[d]}/{len(results)} potholes hit")

    out_csv = Path("/tmp/lidar_pothole_detection_report.csv")
    out_csv.parent.mkdir(parents=True, exist_ok=True)
    with out_csv.open("w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["pothole_id", "name", "x", "y", "distance_m", "total_points",
                    "tag_hits", "geo_hits", "min_range_m"])
        for r in results:
            for dr in r.per_distance:
                w.writerow([r.gt.id, r.gt.name, r.gt.x, r.gt.y, dr.distance_m,
                            dr.total_points, dr.tag_hits, dr.geo_hits, dr.min_range_m])
    print(f"Full per-distance report written to {out_csv}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
