"""
Enums for type-safe constants throughout the codebase.
Phase 4: Code Quality - Replace magic values with enums.
"""

from enum import Enum, IntEnum


class SemanticTag(IntEnum):
    """CARLA Semantic LiDAR tags."""
    UNLABELED = 0
    BUILDING = 1
    FENCE = 2
    OTHER = 3
    PEDESTRIAN = 4
    POLE = 5
    ROAD_LINE = 6
    ROAD = 7
    SIDEWALK = 8
    VEGETATION = 9
    VEHICLE = 10
    WALL = 11
    TRAFFIC_SIGN = 12
    SKY = 13
    GROUND = 14
    BRIDGE = 15
    RAIL_TRACK = 16
    GUARD_RAIL = 17
    TRAFFIC_LIGHT = 18
    STATIC = 19
    DYNAMIC = 20
    WATER = 21
    TERRAIN = 22


class LiDARQuality(Enum):
    """LiDAR quality presets."""
    ULTRA = "ultra"      # 128 channels, 2M points/s, 150m range
    HIGH = "high"        # 64 channels, 1M points/s, 100m range
    MEDIUM = "medium"    # 32 channels, 500k points/s, 75m range
    FAST = "fast"        # 16 channels, 250k points/s, 50m range
    LOWEST = "lowest"    # 8 channels, 100k points/s, 30m range


class VehicleColor(Enum):
    """Common vehicle colors."""
    RED = "255,0,0"
    GREEN = "0,255,0"
    BLUE = "0,0,255"
    WHITE = "255,255,255"
    BLACK = "0,0,0"
    YELLOW = "255,255,0"
    CYAN = "0,255,255"
    MAGENTA = "255,0,255"
    ORANGE = "255,165,0"
    PURPLE = "128,0,128"


class LogLevel(Enum):
    """Logging levels."""
    DEBUG = "DEBUG"
    INFO = "INFO"
    WARNING = "WARNING"
    ERROR = "ERROR"
    CRITICAL = "CRITICAL"


class ScenarioMode(Enum):
    """Scenario execution modes."""
    NORMAL = "normal"
    DATA_COLLECTION = "data_collection"
    PERFORMANCE_TEST = "performance_test"
    DEBUG = "debug"
    REPLAY = "replay"


class TrafficManagerPort(IntEnum):
    """Standard Traffic Manager ports."""
    DEFAULT = 8000
    SECONDARY = 8001
    TERTIARY = 8002
    QUATERNARY = 8003


class SimulationFPS(IntEnum):
    """Standard simulation frame rates."""
    FPS_10 = 10
    FPS_20 = 20
    FPS_30 = 30
    FPS_60 = 60
    FPS_120 = 120
