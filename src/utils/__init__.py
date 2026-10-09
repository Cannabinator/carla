"""Utility modules for data collection and processing.

Submodules are imported lazily (PEP 562) so that importing a light module such
as ``src.utils.binary_protocol`` does not pull in ``carla``.
"""

import importlib

_LAZY_EXPORTS = {
    'DataCollector': 'data_collector',
    'calculate_speed': 'carla_utils',
    'calculate_distance_2d': 'carla_utils',
    'calculate_distance_3d': 'carla_utils',
    'setup_synchronous_mode': 'carla_utils',
    'restore_world_settings': 'carla_utils',
    'setup_traffic_manager': 'carla_utils',
    'get_fresh_velocity': 'carla_utils',
    'spawn_vehicle': 'carla_utils',
    'destroy_actors': 'carla_utils',
    'CARLASession': 'session',
    'VehicleState': 'session',
    'ActorManager': 'actor_manager',
    'ScenarioObserver': 'observers',
    'ConsoleObserver': 'observers',
    'CARLADebugObserver': 'observers',
    'CSVDataLogger': 'observers',
    'CompactLogObserver': 'observers',
    'SpectatorFollowObserver': 'observers',
    'V2VMessageLogger': 'observers',
    'RunDirectory': 'observers',
    'PotholeEventLogger': 'observers',
    'ScenarioConfig': 'builder',
    'ScenarioBuilder': 'builder',
    'quick_scenario': 'builder',
    'v2v_lidar_scenario': 'builder',
    'performance_test_scenario': 'builder',
    'SemanticTag': 'enums',
    'LiDARQuality': 'enums',
    'VehicleColor': 'enums',
    'LogLevel': 'enums',
    'ScenarioMode': 'enums',
    'TrafficManagerPort': 'enums',
    'SimulationFPS': 'enums',
    'BinaryProtocol': 'binary_protocol',
    'compare_bandwidth': 'binary_protocol',
    'OctreeDownsampler': 'octree',
    'LazyProperty': 'lazy',
    'LazyVehicleStats': 'lazy',
    'memoize': 'lazy',
    'lazy_init': 'lazy',
    'Timer': 'lazy',
    'PotholeManager': 'potholes',
    'PotholeInfo': 'potholes',
}

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
    'SpectatorFollowObserver',
    'V2VMessageLogger',
    'RunDirectory',
    'PotholeEventLogger',
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
    'PotholeManager',
    'PotholeInfo',
]


def __getattr__(name):
    module = _LAZY_EXPORTS.get(name)
    if module is None:
        raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
    value = getattr(importlib.import_module(f".{module}", __name__), name)
    globals()[name] = value
    return value
