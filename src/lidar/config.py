"""LiDAR sensor configuration and the schema the UI builds its forms from (REFACTOR_HANDOFF 5.5).

CARLA fixes a sensor's attributes at spawn time, so "reconfigure" means destroy and respawn (see
sensor.py). Two sensor types exist and have DIFFERENT attributes: ``ray_cast_semantic`` returns a semantic
tag and hit-actor id per point and has no noise/dropoff/intensity; ``ray_cast`` returns intensity and
supports noise/dropoff but no tags. A config that sets an attribute its sensor type does not have is
rejected here, not silently ignored by CARLA.

TODO(verify) against the CARLA 0.9.16 sensor reference: attribute names, ranges and defaults below, and the
semantic tag table (ids follow the Cityscapes-aligned scheme introduced in 0.9.14).
"""

from __future__ import annotations

from typing import Any, Dict, List, Literal, Optional

from pydantic import BaseModel, ConfigDict, Field, model_validator

SENSOR_SEMANTIC = "ray_cast_semantic"
SENSOR_RAYCAST = "ray_cast"
SensorType = Literal["ray_cast_semantic", "ray_cast"]

_RAYCAST_ONLY = ("noise_stddev", "dropoff_general_rate", "dropoff_intensity_limit",
                 "dropoff_zero_intensity", "atmosphere_attenuation_rate")


class Mount(BaseModel):
    """Sensor pose relative to the vehicle it is attached to (CARLA axes: x forward, y right, z up)."""
    model_config = ConfigDict(extra="forbid", frozen=True)
    x_m: float = Field(default=0.0, ge=-10.0, le=10.0)
    y_m: float = Field(default=0.0, ge=-10.0, le=10.0)
    z_m: float = Field(default=2.4, ge=-1.0, le=10.0)      # roof
    pitch_deg: float = Field(default=0.0, ge=-90.0, le=90.0)
    yaw_deg: float = Field(default=0.0, ge=-180.0, le=180.0)
    roll_deg: float = Field(default=0.0, ge=-90.0, le=90.0)


class LidarConfig(BaseModel):
    model_config = ConfigDict(extra="forbid", frozen=True)

    sensor_type: SensorType = SENSOR_SEMANTIC
    channels: int = Field(default=32, ge=1, le=256)
    range_m: float = Field(default=100.0, gt=0.0, le=300.0)
    points_per_second: int = Field(default=500_000, ge=1_000, le=10_000_000)
    rotation_frequency_hz: float = Field(default=20.0, gt=0.0, le=100.0)
    upper_fov_deg: float = Field(default=15.0, ge=-90.0, le=90.0)
    lower_fov_deg: float = Field(default=-30.0, ge=-90.0, le=90.0)
    horizontal_fov_deg: float = Field(default=360.0, gt=0.0, le=360.0)
    mount: Mount = Field(default_factory=Mount)

    # ray_cast (non-semantic) only; None = leave CARLA's default
    noise_stddev: Optional[float] = Field(default=None, ge=0.0, le=10.0)
    dropoff_general_rate: Optional[float] = Field(default=None, ge=0.0, le=1.0)
    dropoff_intensity_limit: Optional[float] = Field(default=None, ge=0.0, le=1.0)
    dropoff_zero_intensity: Optional[float] = Field(default=None, ge=0.0, le=1.0)
    atmosphere_attenuation_rate: Optional[float] = Field(default=None, ge=0.0, le=1.0)

    @model_validator(mode="after")
    def _rules(self) -> "LidarConfig":
        if self.lower_fov_deg >= self.upper_fov_deg:
            raise ValueError("lower_fov_deg must be below upper_fov_deg")
        if self.sensor_type == SENSOR_SEMANTIC:
            bad = [n for n in _RAYCAST_ONLY if getattr(self, n) is not None]
            if bad:
                raise ValueError(f"{', '.join(bad)} exist only on sensor type 'ray_cast', not 'ray_cast_semantic'")
        return self

    # ── derived values ──────────────────────────────────────────────────────────
    def points_per_frame(self, fixed_delta_seconds: float) -> float:
        """Points CARLA produces per simulation step (points_per_second spread over time)."""
        return self.points_per_second * fixed_delta_seconds

    def warnings(self, fixed_delta_seconds: float) -> List[str]:
        """Non-fatal problems for the run manifest. With rotation_frequency != 1/dt one measurement is not one
        full sweep (L7): each frame shows a part of a revolution, or a revolution is spread over several frames."""
        out = []
        if abs(self.rotation_frequency_hz - 1.0 / fixed_delta_seconds) > 1e-6:
            out.append(f"rotation_frequency_hz={self.rotation_frequency_hz} but 1/dt={1.0 / fixed_delta_seconds:g}: "
                       f"one measurement is not one full sweep")
        return out

    def carla_attributes(self) -> Dict[str, str]:
        """Attributes to pass to ``blueprint.set_attribute`` (all values as strings, as CARLA expects)."""
        attrs = {
            "channels": str(self.channels),
            "range": str(self.range_m),
            "points_per_second": str(self.points_per_second),
            "rotation_frequency": str(self.rotation_frequency_hz),
            "upper_fov": str(self.upper_fov_deg),
            "lower_fov": str(self.lower_fov_deg),
            "horizontal_fov": str(self.horizontal_fov_deg),
        }
        for name in _RAYCAST_ONLY:
            v = getattr(self, name)
            if v is not None:
                attrs[name] = str(v)
        return attrs

    @property
    def blueprint_id(self) -> str:
        return f"sensor.lidar.{self.sensor_type}"

    @property
    def point_fields(self) -> tuple:
        """Which per-point fields this sensor type delivers beyond x, y, z."""
        return ("tag", "object_idx", "cos_inc") if self.sensor_type == SENSOR_SEMANTIC else ("intensity",)


# ── semantic tags (CARLA 0.9.14+ ids, Cityscapes-style colours) - TODO(verify) against 0.9.16 ──────────────
SEMANTIC_TAGS: List[Dict[str, Any]] = [
    {"id": i, "name": n, "color": list(c)} for i, (n, c) in enumerate([
        ("Unlabeled", (0, 0, 0)), ("Roads", (128, 64, 128)), ("SideWalks", (244, 35, 232)),
        ("Building", (70, 70, 70)), ("Wall", (102, 102, 156)), ("Fence", (190, 153, 153)),
        ("Pole", (153, 153, 153)), ("TrafficLight", (250, 170, 30)), ("TrafficSign", (220, 220, 0)),
        ("Vegetation", (107, 142, 35)), ("Terrain", (152, 251, 152)), ("Sky", (70, 130, 180)),
        ("Pedestrian", (220, 20, 60)), ("Rider", (255, 0, 0)), ("Car", (0, 0, 142)),
        ("Truck", (0, 0, 70)), ("Bus", (0, 60, 100)), ("Train", (0, 80, 100)),
        ("Motorcycle", (0, 0, 230)), ("Bicycle", (119, 11, 32)), ("Static", (110, 190, 160)),
        ("Dynamic", (170, 120, 50)), ("Other", (55, 90, 80)), ("Water", (45, 60, 150)),
        ("RoadLines", (157, 234, 50)), ("Ground", (81, 0, 81)), ("Bridge", (150, 100, 100)),
        ("RailTrack", (230, 150, 140)), ("GuardRail", (180, 165, 180)),
    ])
]
MAX_SEMANTIC_TAG = len(SEMANTIC_TAGS) - 1


def _attr(name, label, kind, unit, lo, hi, default, types):
    return {"name": name, "label": label, "type": kind, "unit": unit, "min": lo, "max": hi,
            "default": default, "sensor_types": list(types)}


def lidar_schema(limits: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
    """JSON-serialisable description of what can be configured, per sensor type, plus the tag table.
    Only attributes valid for a sensor type are listed under it."""
    both = (SENSOR_SEMANTIC, SENSOR_RAYCAST)
    d = LidarConfig()
    m = Mount()
    common = [
        _attr("channels", "Channels", "int", "", 1, 256, d.channels, both),
        _attr("range_m", "Range", "float", "m", 0.0, 300.0, d.range_m, both),
        _attr("points_per_second", "Points per second", "int", "pt/s", 1_000, 10_000_000, d.points_per_second, both),
        _attr("rotation_frequency_hz", "Rotation frequency", "float", "Hz", 0.0, 100.0, d.rotation_frequency_hz, both),
        _attr("upper_fov_deg", "Upper FOV", "float", "deg", -90, 90, d.upper_fov_deg, both),
        _attr("lower_fov_deg", "Lower FOV", "float", "deg", -90, 90, d.lower_fov_deg, both),
        _attr("horizontal_fov_deg", "Horizontal FOV", "float", "deg", 0.0, 360.0, d.horizontal_fov_deg, both),
    ]
    mount = [
        _attr("mount.x_m", "Mount x", "float", "m", -10, 10, m.x_m, both),
        _attr("mount.y_m", "Mount y", "float", "m", -10, 10, m.y_m, both),
        _attr("mount.z_m", "Mount z", "float", "m", -1, 10, m.z_m, both),
        _attr("mount.pitch_deg", "Mount pitch", "float", "deg", -90, 90, m.pitch_deg, both),
        _attr("mount.yaw_deg", "Mount yaw", "float", "deg", -180, 180, m.yaw_deg, both),
        _attr("mount.roll_deg", "Mount roll", "float", "deg", -90, 90, m.roll_deg, both),
    ]
    ray = (SENSOR_RAYCAST,)
    raycast_only = [
        _attr("noise_stddev", "Noise std", "float", "m", 0.0, 10.0, None, ray),
        _attr("dropoff_general_rate", "General dropoff rate", "float", "", 0.0, 1.0, None, ray),
        _attr("dropoff_intensity_limit", "Dropoff intensity limit", "float", "", 0.0, 1.0, None, ray),
        _attr("dropoff_zero_intensity", "Dropoff at zero intensity", "float", "", 0.0, 1.0, None, ray),
        _attr("atmosphere_attenuation_rate", "Atmosphere attenuation", "float", "1/m", 0.0, 1.0, None, ray),
    ]
    def for_type(t):
        return [a for a in common + mount + raycast_only if t in a["sensor_types"]]
    return {
        "protocol_version": 3,
        "sensor_types": {
            SENSOR_SEMANTIC: {"label": "Semantic LiDAR", "attributes": for_type(SENSOR_SEMANTIC),
                              "point_fields": ["tag", "object_idx", "cos_inc"]},
            SENSOR_RAYCAST: {"label": "LiDAR (intensity)", "attributes": for_type(SENSOR_RAYCAST),
                             "point_fields": ["intensity"]},
        },
        "semantic_tags": SEMANTIC_TAGS,
        "limits": limits or {},
    }
