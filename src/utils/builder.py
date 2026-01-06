"""
Builder pattern for clean scenario configuration.
Provides fluent API for setting up complex scenarios.
"""

from typing import Optional
from dataclasses import dataclass, field


@dataclass
class ScenarioConfig:
    """Complete scenario configuration."""
    # CARLA Connection
    host: str = '192.168.1.110'
    port: int = 2000
    timeout: float = 30.0
    
    # Simulation
    duration: int = 60
    fps: int = 20
    fixed_delta_seconds: float = 0.05
    random_seed: int = 42
    
    # Vehicles
    num_vehicles: int = 15
    ego_blueprint: str = 'vehicle.tesla.model3'
    ego_color: str = '255,0,0'
    
    # V2V
    v2v_enabled: bool = True
    v2v_range: float = 50.0
    v2v_update_interval_frames: int = 4
    
    # LiDAR
    lidar_enabled: bool = False
    lidar_quality: str = 'high'  # 'high' or 'fast'
    lidar_web_port: int = 8000
    
    # Traffic Manager
    tm_port: int = 8001
    tm_seed: int = 42
    use_hybrid_physics: bool = False
    hybrid_physics_radius: float = 70.0
    global_speed_difference: float = -30.0  # -30% slower
    ego_speed_difference: float = 0.0
    safety_distance: float = 3.0
    
    # Visualization
    console_output: bool = True
    console_interval_seconds: float = 2.0
    carla_debug_viz: bool = True
    debug_viz_interval_frames: int = 5
    
    # Logging
    csv_logging: bool = False
    csv_output_path: Optional[str] = None
    compact_logging: bool = True
    
    # Display
    warmup_frames: int = 100
    stats_display_interval_seconds: float = 2.0


class ScenarioBuilder:
    """Fluent builder for scenario configuration."""
    
    def __init__(self):
        """Initialize with default config."""
        self._config = ScenarioConfig()
    
    # CARLA Connection
    def with_carla_server(self, host: str, port: int = 2000, timeout: float = 30.0):
        """Set CARLA server connection."""
        self._config.host = host
        self._config.port = port
        self._config.timeout = timeout
        return self
    
    def with_duration(self, seconds: int):
        """Set scenario duration in seconds."""
        self._config.duration = seconds
        return self
    
    def with_fps(self, fps: int):
        """Set simulation frame rate."""
        self._config.fps = fps
        self._config.fixed_delta_seconds = 1.0 / fps
        return self
    
    def with_seed(self, seed: int):
        """Set random seed for reproducibility."""
        self._config.random_seed = seed
        self._config.tm_seed = seed
        return self
    
    # Vehicles
    def with_vehicles(self, count: int):
        """Set number of vehicles to spawn."""
        self._config.num_vehicles = count
        return self
    
    def with_ego_vehicle(self, blueprint: str = 'vehicle.tesla.model3', color: str = '255,0,0'):
        """Configure ego vehicle."""
        self._config.ego_blueprint = blueprint
        self._config.ego_color = color
        return self
    
    # V2V
    def with_v2v(self, enabled: bool = True, range_m: float = 50.0, update_interval_frames: int = 4):
        """Configure V2V communication."""
        self._config.v2v_enabled = enabled
        self._config.v2v_range = range_m
        self._config.v2v_update_interval_frames = update_interval_frames
        return self
    
    def without_v2v(self):
        """Disable V2V communication."""
        self._config.v2v_enabled = False
        return self
    
    # LiDAR
    def with_lidar(self, quality: str = 'high', web_port: int = 8000):
        """Enable LiDAR streaming."""
        self._config.lidar_enabled = True
        self._config.lidar_quality = quality
        self._config.lidar_web_port = web_port
        return self
    
    def without_lidar(self):
        """Disable LiDAR streaming."""
        self._config.lidar_enabled = False
        return self
    
    # Traffic Manager
    def with_traffic_manager(self, port: int = 8001, global_speed_diff: float = -30.0):
        """Configure traffic manager."""
        self._config.tm_port = port
        self._config.global_speed_difference = global_speed_diff
        return self
    
    def with_hybrid_physics(self, enabled: bool = True, radius: float = 70.0):
        """Configure hybrid physics mode."""
        self._config.use_hybrid_physics = enabled
        self._config.hybrid_physics_radius = radius
        return self
    
    def with_safety_distance(self, distance: float):
        """Set vehicle safety distance."""
        self._config.safety_distance = distance
        return self
    
    # Visualization
    def with_console_output(self, enabled: bool = True, interval_seconds: float = 2.0):
        """Configure console output."""
        self._config.console_output = enabled
        self._config.console_interval_seconds = interval_seconds
        return self
    
    def with_carla_debug(self, enabled: bool = True, interval_frames: int = 5):
        """Configure CARLA debug visualization."""
        self._config.carla_debug_viz = enabled
        self._config.debug_viz_interval_frames = interval_frames
        return self
    
    # Logging
    def with_csv_logging(self, enabled: bool = True, output_path: Optional[str] = None):
        """Enable CSV data logging."""
        self._config.csv_logging = enabled
        self._config.csv_output_path = output_path
        return self
    
    def with_compact_logging(self, enabled: bool = True):
        """Enable compact logger output."""
        self._config.compact_logging = enabled
        return self
    
    # Build
    def build(self) -> ScenarioConfig:
        """Build and return the configuration."""
        return self._config
    
    @classmethod
    def from_args(cls, args):
        """Create builder from argparse arguments."""
        builder = cls()
        
        # CARLA
        if hasattr(args, 'host'):
            builder.with_carla_server(args.host, args.port if hasattr(args, 'port') else 2000)
        
        # Duration
        if hasattr(args, 'duration'):
            builder.with_duration(args.duration)
        
        # Vehicles
        if hasattr(args, 'vehicles'):
            builder.with_vehicles(args.vehicles)
        
        # V2V
        if hasattr(args, 'v2v_range'):
            builder.with_v2v(range_m=args.v2v_range)
        
        # LiDAR
        if hasattr(args, 'enable_lidar') and args.enable_lidar:
            quality = args.lidar_quality if hasattr(args, 'lidar_quality') else 'high'
            port = args.web_port if hasattr(args, 'web_port') else 8000
            builder.with_lidar(quality, port)
        
        return builder


# Convenience factory functions
def quick_scenario(host: str = '192.168.1.110', duration: int = 60, num_vehicles: int = 10) -> ScenarioConfig:
    """Quick scenario with minimal config."""
    return (ScenarioBuilder()
        .with_carla_server(host)
        .with_duration(duration)
        .with_vehicles(num_vehicles)
        .with_v2v()
        .build())


def v2v_lidar_scenario(host: str = '192.168.1.110', duration: int = 120, 
                       num_vehicles: int = 15, lidar_quality: str = 'high') -> ScenarioConfig:
    """Full V2V + LiDAR scenario."""
    return (ScenarioBuilder()
        .with_carla_server(host)
        .with_duration(duration)
        .with_vehicles(num_vehicles)
        .with_v2v(range_m=50.0)
        .with_lidar(quality=lidar_quality)
        .with_csv_logging(enabled=True)
        .build())


def performance_test_scenario(host: str = '192.168.1.110', num_vehicles: int = 50) -> ScenarioConfig:
    """High vehicle count for performance testing."""
    return (ScenarioBuilder()
        .with_carla_server(host)
        .with_duration(300)
        .with_vehicles(num_vehicles)
        .with_v2v(range_m=100.0)
        .without_lidar()
        .with_console_output(enabled=False)
        .with_compact_logging(enabled=True)
        .build())
