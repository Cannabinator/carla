import sys
from pathlib import Path

import pytest
from pydantic import ValidationError

sys.path.insert(0, str(Path(__file__).parent.parent.parent))

from src.lidar.config import MAX_SEMANTIC_TAG, SEMANTIC_TAGS, LidarConfig, Mount, lidar_schema


def test_defaults_match_the_previous_hard_coded_per_session_sensor():
    c = LidarConfig()
    assert (c.sensor_type, c.channels, c.points_per_second, c.range_m) == ("ray_cast_semantic", 32, 500_000, 100.0)
    assert (c.rotation_frequency_hz, c.upper_fov_deg, c.lower_fov_deg, c.horizontal_fov_deg) == (20.0, 15.0, -30.0, 360.0)
    assert (c.mount.x_m, c.mount.z_m) == (0.0, 2.4)


def test_carla_attributes_are_strings_and_type_specific():
    sem = LidarConfig().carla_attributes()
    assert sem["channels"] == "32" and sem["points_per_second"] == "500000" and "noise_stddev" not in sem
    assert all(isinstance(v, str) for v in sem.values())
    ray = LidarConfig(sensor_type="ray_cast", noise_stddev=0.02, dropoff_general_rate=0.45).carla_attributes()
    assert ray["noise_stddev"] == "0.02" and ray["dropoff_general_rate"] == "0.45" and "dropoff_zero_intensity" not in ray
    assert LidarConfig(sensor_type="ray_cast").blueprint_id == "sensor.lidar.ray_cast"
    assert LidarConfig().blueprint_id == "sensor.lidar.ray_cast_semantic"


def test_raycast_only_attributes_are_rejected_on_the_semantic_sensor():
    with pytest.raises(ValidationError, match="ray_cast"):
        LidarConfig(noise_stddev=0.1)
    LidarConfig(sensor_type="ray_cast", noise_stddev=0.1)   # fine


@pytest.mark.parametrize("kw", [
    {"lower_fov_deg": 20.0}, {"upper_fov_deg": -40.0}, {"channels": 0}, {"channels": 1000},
    {"range_m": 0.0}, {"range_m": 1000.0}, {"points_per_second": 10}, {"rotation_frequency_hz": 0.0},
    {"horizontal_fov_deg": 0.0}, {"horizontal_fov_deg": 400.0}, {"sensor_type": "camera"}, {"unknown": 1},
    {"mount": {"z_m": 50.0}}, {"mount": {"pitch_deg": 120.0}},
])
def test_invalid_configs_are_rejected(kw):
    with pytest.raises(ValidationError):
        LidarConfig(**kw)


def test_points_per_frame_and_sweep_warning():
    c = LidarConfig(points_per_second=500_000)
    assert c.points_per_frame(0.05) == 25_000.0
    assert c.warnings(0.05) == []
    w = LidarConfig(rotation_frequency_hz=10.0).warnings(0.05)
    assert len(w) == 1 and "full sweep" in w[0]


def test_config_is_immutable_and_json_roundtrips():
    c = LidarConfig(channels=64, mount={"z_m": 1.9})
    with pytest.raises(ValidationError):
        c.channels = 8
    assert LidarConfig.model_validate_json(c.model_dump_json()) == c


def test_point_fields_per_type():
    assert LidarConfig().point_fields == ("tag", "object_idx", "cos_inc")
    assert LidarConfig(sensor_type="ray_cast").point_fields == ("intensity",)


def test_schema_lists_only_valid_attributes_per_type():
    s = lidar_schema({"max_sensors": 4})
    sem = {a["name"] for a in s["sensor_types"]["ray_cast_semantic"]["attributes"]}
    ray = {a["name"] for a in s["sensor_types"]["ray_cast"]["attributes"]}
    assert "noise_stddev" not in sem and "noise_stddev" in ray
    assert {"channels", "range_m", "mount.z_m", "horizontal_fov_deg"} <= sem <= ray
    assert s["protocol_version"] == 3 and s["limits"] == {"max_sensors": 4}
    # every schema attribute is a real config field (guards against schema drift)
    fields = set(LidarConfig.model_fields) | {f"mount.{n}" for n in Mount.model_fields}
    assert (sem | ray) <= fields - {"mount", "sensor_type"}


def test_schema_defaults_equal_config_defaults():
    d = LidarConfig()
    for a in lidar_schema()["sensor_types"]["ray_cast_semantic"]["attributes"]:
        cur = d
        for part in a["name"].split("."):
            cur = getattr(cur, part)
        assert cur == a["default"], a["name"]


def test_semantic_tag_table_is_consistent():
    ids = [t["id"] for t in SEMANTIC_TAGS]
    assert ids == list(range(MAX_SEMANTIC_TAG + 1)) and MAX_SEMANTIC_TAG == 28
    assert len({t["name"] for t in SEMANTIC_TAGS}) == len(SEMANTIC_TAGS)
    assert all(len(t["color"]) == 3 and all(0 <= c <= 255 for c in t["color"]) for t in SEMANTIC_TAGS)
    assert SEMANTIC_TAGS[1]["name"] == "Roads" and SEMANTIC_TAGS[3]["name"] == "Building"   # not the pre-0.9.14 ids
