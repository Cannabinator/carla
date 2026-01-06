"""
Observer pattern for scenario visualization and logging.
Separates scenario logic from visualization concerns.
"""

from abc import ABC, abstractmethod
from typing import Dict, Any, Optional
import carla
from datetime import datetime
from pathlib import Path
import csv

from .session import VehicleState
from ..v2v import V2VNetwork
from ..config import DEFAULT_VIZ_CONFIG


class ScenarioObserver(ABC):
    """Abstract base class for scenario observers."""
    
    @abstractmethod
    def on_frame(self, frame: int, state: VehicleState, v2v_data: Dict[str, Any]):
        """Called every frame with current state."""
        pass
    
    @abstractmethod
    def on_complete(self, total_frames: int, elapsed_time: float):
        """Called when scenario completes."""
        pass


class ConsoleObserver(ScenarioObserver):
    """Print vehicle stats to console at regular intervals."""
    
    def __init__(self, interval_seconds: float = 2.0, fps: int = 20):
        """
        Args:
            interval_seconds: How often to print stats
            fps: Simulation frame rate for conversion
        """
        self.interval_frames = int(interval_seconds * fps)
        self.last_print_frame = 0
    
    def on_frame(self, frame: int, state: VehicleState, v2v_data: Dict[str, Any]):
        """Print stats every interval."""
        if frame - self.last_print_frame >= self.interval_frames:
            self._print_stats(state, v2v_data)
            self.last_print_frame = frame
    
    def _print_stats(self, state: VehicleState, v2v_data: Dict[str, Any]):
        """Format and print vehicle statistics."""
        neighbors = v2v_data.get('neighbors', [])
        total_vehicles = v2v_data.get('total_vehicles', 0)
        lidar_points = v2v_data.get('lidar_points', 0)
        
        print(f"\n{'='*85}")
        print(f"🚗 LEADING VEHICLE - Frame {state.frame:4d} | Speed: {state.speed_kmh:.1f} km/h")
        print(f"{'='*85}")
        print(f"📍 Position:      X={state.position[0]:9.2f}m  Y={state.position[1]:9.2f}m  Z={state.position[2]:8.2f}m")
        print(f"🏃 Velocity:      Vx={state.velocity[0]:8.3f}  Vy={state.velocity[1]:8.3f}  Vz={state.velocity[2]:8.3f} m/s")
        print(f"⚡ Speed:         {state.speed_kmh:7.2f} km/h ({state.speed_ms:6.3f} m/s)")
        print(f"🧭 Orientation:   Yaw={state.orientation[0]:7.2f}°  Pitch={state.orientation[1]:6.2f}°  Roll={state.orientation[2]:6.2f}°")
        
        if state.control:
            print(f"🎮 Control:       Throttle={state.control.throttle:.3f}  Brake={state.control.brake:.3f}  Steer={state.control.steer:.3f}")
        
        print(f"🔄 Angular Vel:   ωx={state.angular_velocity[0]:7.2f}  ωy={state.angular_velocity[1]:7.2f}  ωz={state.angular_velocity[2]:7.2f} °/s")
        print(f"📡 V2V Comms:     {len(neighbors)}/{total_vehicles-1} vehicles in range")
        
        if lidar_points > 0:
            print(f"🎯 LiDAR Points:  {lidar_points:,} points/frame")
        
        if neighbors:
            print(f"\n   🔗 Connected Vehicles:")
            max_display = DEFAULT_VIZ_CONFIG.max_neighbors_displayed
            for i, neighbor in enumerate(neighbors[:max_display], 1):
                from ..utils import calculate_distance_3d
                dist = calculate_distance_3d(neighbor.location, state.position)
                neighbor_speed_kmh = neighbor.speed * 3.6
                rel_speed = neighbor_speed_kmh - state.speed_kmh
                print(f"      {i}. ID {neighbor.vehicle_id:3d}: {neighbor_speed_kmh:6.2f} km/h | "
                      f"Dist: {dist:6.2f}m | Δv: {rel_speed:+6.2f} km/h")
            if len(neighbors) > max_display:
                print(f"      ... and {len(neighbors) - max_display} more")
        
        print(f"{'='*85}\n")
    
    def on_complete(self, total_frames: int, elapsed_time: float):
        """Print completion summary."""
        simulated_time = total_frames / 20  # 20 FPS
        print(f"\n✓ Scenario completed ({total_frames} frames, {simulated_time:.1f}s simulated, {elapsed_time:.1f}s real)")


class CARLADebugObserver(ScenarioObserver):
    """Draw debug visualizations in CARLA world."""
    
    def __init__(self, world: carla.World, v2v_network: V2VNetwork, 
                 ego_id: int = 0, update_interval_frames: int = 5):
        """
        Args:
            world: CARLA world instance
            v2v_network: V2V network to visualize
            ego_id: Ego vehicle ID in V2V network
            update_interval_frames: How often to redraw (avoid overhead)
        """
        self.world = world
        self.v2v = v2v_network
        self.ego_id = ego_id
        self.update_interval = update_interval_frames
        self.config = DEFAULT_VIZ_CONFIG
    
    def on_frame(self, frame: int, state: VehicleState, v2v_data: Dict[str, Any]):
        """Draw V2V connections and range circle."""
        if frame % self.update_interval != 0:
            return
        
        self._draw_v2v_visualization(state)
    
    def _draw_v2v_visualization(self, state: VehicleState):
        """Draw V2V range and connections."""
        import numpy as np
        
        ego_state = self.v2v.get_state(self.ego_id)
        if not ego_state:
            return
        
        ego_loc = carla.Location(*ego_state.location)
        debug = self.world.debug
        frame_duration = 0.25  # Slightly longer than update interval
        
        # Draw range circle
        num_segments = self.config.range_circle_segments
        range_m = self.v2v.max_range
        
        for i in range(num_segments):
            angle1 = (i / num_segments) * 2 * np.pi
            angle2 = ((i + 1) / num_segments) * 2 * np.pi
            
            x1 = ego_loc.x + range_m * np.cos(angle1)
            y1 = ego_loc.y + range_m * np.sin(angle1)
            x2 = ego_loc.x + range_m * np.cos(angle2)
            y2 = ego_loc.y + range_m * np.sin(angle2)
            
            p1 = carla.Location(x=x1, y=y1, z=ego_loc.z + self.config.range_circle_z_offset)
            p2 = carla.Location(x=x2, y=y2, z=ego_loc.z + self.config.range_circle_z_offset)
            
            debug.draw_line(p1, p2, thickness=self.config.range_circle_thickness,
                           color=carla.Color(*self.config.range_circle_color), 
                           life_time=frame_duration)
        
        # Draw connection lines
        neighbors = self.v2v.get_neighbors(self.ego_id)
        for neighbor in neighbors:
            neighbor_loc = carla.Location(*neighbor.location)
            debug.draw_line(
                ego_loc + carla.Location(z=self.config.connection_line_z_offset),
                neighbor_loc + carla.Location(z=self.config.connection_line_z_offset),
                thickness=self.config.connection_line_thickness,
                color=carla.Color(*self.config.connection_line_color),
                life_time=frame_duration
            )
    
    def on_complete(self, total_frames: int, elapsed_time: float):
        """Cleanup - nothing needed for debug drawing."""
        pass


class CSVDataLogger(ScenarioObserver):
    """Log vehicle and V2V data to CSV file."""
    
    def __init__(self, output_path: Optional[Path] = None):
        """
        Args:
            output_path: CSV file path. If None, auto-generated in logs/
        """
        if output_path is None:
            log_dir = Path(__file__).parent.parent.parent / 'logs'
            log_dir.mkdir(exist_ok=True)
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            output_path = log_dir / f'v2v_data_{timestamp}.csv'
        
        self.output_path = output_path
        self.csv_file = None
        self.writer = None
        self.total_rows = 0
    
    def on_frame(self, frame: int, state: VehicleState, v2v_data: Dict[str, Any]):
        """Log frame data to CSV."""
        # Lazy file opening
        if self.csv_file is None:
            self._open_csv()
        
        neighbors = v2v_data.get('neighbors', [])
        
        # Write row
        self.writer.writerow({
            'frame': state.frame,
            'timestamp': datetime.now().isoformat(),
            'pos_x': state.position[0],
            'pos_y': state.position[1],
            'pos_z': state.position[2],
            'vel_x': state.velocity[0],
            'vel_y': state.velocity[1],
            'vel_z': state.velocity[2],
            'speed_kmh': state.speed_kmh,
            'speed_ms': state.speed_ms,
            'yaw': state.orientation[0],
            'pitch': state.orientation[1],
            'roll': state.orientation[2],
            'throttle': state.control.throttle if state.control else 0,
            'brake': state.control.brake if state.control else 0,
            'steer': state.control.steer if state.control else 0,
            'v2v_neighbors': len(neighbors),
            'lidar_points': v2v_data.get('lidar_points', 0)
        })
        self.total_rows += 1
    
    def _open_csv(self):
        """Open CSV file and write header."""
        self.csv_file = open(self.output_path, 'w', newline='')
        fieldnames = [
            'frame', 'timestamp', 
            'pos_x', 'pos_y', 'pos_z',
            'vel_x', 'vel_y', 'vel_z',
            'speed_kmh', 'speed_ms',
            'yaw', 'pitch', 'roll',
            'throttle', 'brake', 'steer',
            'v2v_neighbors', 'lidar_points'
        ]
        self.writer = csv.DictWriter(self.csv_file, fieldnames=fieldnames)
        self.writer.writeheader()
    
    def on_complete(self, total_frames: int, elapsed_time: float):
        """Close CSV file."""
        if self.csv_file:
            self.csv_file.close()
            print(f"📊 Logged {self.total_rows} frames to {self.output_path}")


class CompactLogObserver(ScenarioObserver):
    """Compact single-line logging for debugging."""
    
    def __init__(self, logger):
        """
        Args:
            logger: Python logger instance
        """
        self.logger = logger
    
    def on_frame(self, frame: int, state: VehicleState, v2v_data: Dict[str, Any]):
        """Log compact frame info."""
        neighbors = v2v_data.get('neighbors', [])
        self.logger.info(f"F{frame:04d} | {state} | V2V:{len(neighbors)}")
    
    def on_complete(self, total_frames: int, elapsed_time: float):
        """Log completion."""
        self.logger.info(f"Scenario completed: {total_frames} frames in {elapsed_time:.1f}s")
