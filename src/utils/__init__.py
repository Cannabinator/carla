"""Utility modules for data collection and processing"""

from .data_collector import DataCollector
from .carla_utils import (
    calculate_speed,
    calculate_distance_2d,
    calculate_distance_3d,
    setup_synchronous_mode,
    restore_world_settings,
    setup_traffic_manager,
    get_fresh_velocity,
    spawn_vehicle,
    destroy_actors
)
from .session import CARLASession, VehicleState
from .actor_manager import ActorManager
from .observers import (
    ScenarioObserver,
    ConsoleObserver,
    CARLADebugObserver,
    CSVDataLogger,
    CompactLogObserver
)
from .builder import (
    ScenarioConfig,
    ScenarioBuilder,
    quick_scenario,
    v2v_lidar_scenario,
    performance_test_scenario
)
from .enums import (
    SemanticTag,
    LiDARQuality,
    VehicleColor,
    LogLevel,
    ScenarioMode,
    TrafficManagerPort,
    SimulationFPS
)
from .binary_protocol import BinaryProtocol, compare_bandwidth
from .octree import OctreeDownsampler
from .lazy import LazyProperty, LazyVehicleStats, memoize, lazy_init, Timer

__all__ = [
    'DataCollector',
    'calculate_speed',
    'calculate_distance_2d',
    'calculate_distance_3d',
    'setup_synchronous_mode',
    'restore_world_settings',
    'setup_traffic_manager',
    'get_fresh_velocity',
    'spawn_vehicle',
    'destroy_actors',
    'CARLASession',
    'VehicleState',
    'ActorManager',
    'ScenarioObserver',
    'ConsoleObserver',
    'CARLADebugObserver',
    'CSVDataLogger',
    'CompactLogObserver',
    'ScenarioConfig',
    'ScenarioBuilder',
    'quick_scenario',
    'v2v_lidar_scenario',
    'performance_test_scenario',
    # Phase 3 & 4
    'SemanticTag',
    'LiDARQuality',
    'VehicleColor',
    'LogLevel',
    'ScenarioMode',
    'TrafficManagerPort',
    'SimulationFPS',
    'BinaryProtocol',
    'compare_bandwidth',
    'OctreeDownsampler',
    'LazyProperty',
    'LazyVehicleStats',
    'memoize',
    'lazy_init',
    'Timer',
]
