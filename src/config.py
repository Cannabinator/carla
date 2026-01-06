"""
Configuration constants for CARLA V2V scenarios.
Centralized configuration to avoid magic numbers and improve maintainability.
"""

from dataclasses import dataclass
from typing import Optional


@dataclass
class SimulationConfig:
    """Core simulation configuration."""
    # Synchronous mode settings
    fixed_delta_seconds: float = 0.05  # 20 FPS
    timeout: float = 30.0  # CARLA client timeout
    
    # Seeds for reproducibility
    random_seed: int = 42
    
    # Traffic Manager settings
    tm_port: int = 8000
    tm_seed: int = 42
    
    # Physics settings
    use_hybrid_physics: bool = False  # False to avoid zero velocity issue
    hybrid_physics_radius: float = 70.0
    
    # Traffic behavior
    global_speed_difference: float = 30.0  # % slower than speed limit
    safety_distance: float = 2.5  # meters
    ego_speed_difference: float = 20.0  # % slower for ego vehicle
    
    # Warmup
    warmup_frames: int = 50
    
    # Update intervals
    v2v_update_interval_frames: int = 4  # Update every 4 frames (0.2s at 20 FPS)
    debug_log_interval_frames: int = 10  # Log debug every 10 frames (0.5s)
    stats_display_interval_seconds: float = 2.0  # Display stats every 2 seconds


@dataclass
class VisualizationConfig:
    """Visualization configuration."""
    # V2V range circle
    range_circle_segments: int = 16
    range_circle_color: tuple = (0, 200, 0, 40)  # RGBA
    range_circle_thickness: float = 0.02
    range_circle_z_offset: float = 0.2
    
    # V2V connection lines
    connection_line_color: tuple = (100, 200, 255, 80)  # RGBA
    connection_line_thickness: float = 0.02
    connection_line_z_offset: float = 1.0
    
    # Display settings
    max_neighbors_displayed: int = 5


@dataclass
class V2VConfig:
    """V2V network configuration."""
    max_range: float = 50.0  # meters
    update_interval: float = 0.1  # seconds
    ego_vehicle_id: int = 0


@dataclass
class VehicleConfig:
    """Vehicle configuration."""
    ego_blueprint: str = 'vehicle.tesla.model3'
    ego_color: str = '255,0,0'  # Red
    min_wheels: int = 4  # Filter for 4-wheeled vehicles only


# Default configurations
DEFAULT_SIM_CONFIG = SimulationConfig()
DEFAULT_VIZ_CONFIG = VisualizationConfig()
DEFAULT_V2V_CONFIG = V2VConfig()
DEFAULT_VEHICLE_CONFIG = VehicleConfig()
