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
from ..v2v import V2VNetworkEnhanced
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
                # Support both BSMCore (enhanced) and V2VState (old)
                if hasattr(neighbor, 'latitude'):
                    neighbor_loc = (neighbor.latitude, neighbor.longitude, neighbor.elevation)
                else:
                    neighbor_loc = neighbor.location
                dist = calculate_distance_3d(neighbor_loc, state.position)
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
    
    def __init__(self, world: carla.World, v2v_network: V2VNetworkEnhanced, 
                 ego_id: int = 0, update_interval_frames: int = 5):
        """
        Args:
            world: CARLA world instance
            v2v_network: V2VNetworkEnhanced instance to visualize
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
        
        # Get ego BSM from V2VNetworkEnhanced
        ego_bsm = self.v2v.get_bsm(self.ego_id)
        if not ego_bsm:
            return
        ego_loc = carla.Location(ego_bsm.latitude, ego_bsm.longitude, ego_bsm.elevation)
        
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
            # BSMCore from V2VNetworkEnhanced
            neighbor_loc = carla.Location(neighbor.latitude, neighbor.longitude, neighbor.elevation)
                
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
    """Log vehicle and V2V data to CSV file with detailed BSM information."""
    
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
        """Log frame data to CSV with detailed V2V information."""
        # Lazy file opening
        if self.csv_file is None:
            self._open_csv()
        
        neighbors = v2v_data.get('neighbors', [])
        threats = v2v_data.get('threats', [])
        bsm = v2v_data.get('bsm', None)
        
        # Prepare neighbor data (IDs and distances)
        neighbor_ids = ','.join([str(n.vehicle_id) for n in neighbors[:5]]) if neighbors else ''
        neighbor_distances = ','.join([f"{n.distance:.1f}" for n in neighbors[:5] if hasattr(n, 'distance')]) or ''
        
        # Threat data
        threat_count = len([t for t in threats if t.get('level', 0) >= 2]) if threats else 0
        min_ttc = min([t.get('ttc', 999) for t in threats], default=999) if threats else 999
        
        # BSM data
        bsm_heading = bsm.heading if bsm and hasattr(bsm, 'heading') else 0
        bsm_accel = bsm.longitudinal_accel if bsm and hasattr(bsm, 'longitudinal_accel') else 0
        
        # Write row
        self.writer.writerow({  # type: ignore
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
            'neighbor_ids': neighbor_ids,
            'neighbor_distances': neighbor_distances,
            'threats': threat_count,
            'min_ttc': min_ttc,
            'bsm_heading': bsm_heading,
            'bsm_accel': bsm_accel,
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
            'v2v_neighbors', 'neighbor_ids', 'neighbor_distances',
            'threats', 'min_ttc',
            'bsm_heading', 'bsm_accel',
            'lidar_points'
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


class SpectatorFollowObserver(ScenarioObserver):
    """
    Bird's-eye view spectator camera following the ego vehicle.
    
    Provides a smooth overhead view in the CARLA server GUI that follows
    the leading vehicle throughout the simulation.
    
    Uses a critically-damped spring model for smooth, flicker-free camera
    movement. For pure top-down view (-90° pitch), yaw is locked to
    eliminate rotational flickering.
    
    Based on CARLA's spectator API pattern from PythonAPI examples.
    """
    
    def __init__(self, world: carla.World, vehicle: carla.Actor,
                 height: float = 50.0, pitch: float = -90.0,
                 update_interval_frames: int = 1,
                 smoothing_factor: float = 0.1):
        """
        Initialize spectator follower.
        
        Args:
            world: CARLA world instance
            vehicle: Vehicle to follow (ego vehicle)
            height: Camera height above vehicle (meters)
            pitch: Camera pitch angle (degrees, -90 = straight down)
            update_interval_frames: How often to update camera position
            smoothing_factor: Exponential smoothing factor (0-1, lower = smoother)
                             Recommended: 0.08-0.15 for smooth bird's-eye
        """
        self.world = world
        self.vehicle = vehicle
        self.height = height
        self.pitch = pitch
        self.update_interval = update_interval_frames
        self.spectator = world.get_spectator()
        self.smoothing = smoothing_factor
        
        # Is this a pure top-down view? If so, lock yaw to prevent rotation flicker
        self._is_topdown = (pitch <= -85.0)
        
        # Camera state (initialized on first frame)
        self._cam_x: Optional[float] = None
        self._cam_y: Optional[float] = None
        self._cam_z: Optional[float] = None
        self._cam_yaw: Optional[float] = None
        
        # Velocity tracking for critically-damped spring
        self._vel_x: float = 0.0
        self._vel_y: float = 0.0
        self._vel_z: float = 0.0
        self._vel_yaw: float = 0.0
        
        # Fixed timestep from CARLA config (default 0.05s = 20 FPS)
        self._dt: float = 0.05
        
        # Spring damping parameters - tuned for smooth, responsive following
        # omega = natural frequency, controls how fast camera catches up
        self._omega: float = 4.0  # ~4 Hz natural frequency → smooth follow
    
    def _spring_smooth(self, target: float, current: float, velocity: float) -> tuple:
        """
        Critically-damped spring smoothing for flicker-free motion.
        
        Returns (new_position, new_velocity).
        
        A critically-damped spring reaches the target as fast as possible
        without overshooting or oscillating, eliminating all flickering.
        """
        omega = self._omega
        dt = self._dt
        
        # Critically damped: damping ratio = 1.0, so zeta*omega = omega
        exp_term = 2.718281828 ** (-omega * dt)
        
        diff = current - target
        new_pos = target + (diff + (velocity + omega * diff) * dt) * exp_term
        new_vel = (velocity - omega * omega * diff * dt) * exp_term
        
        return new_pos, new_vel
    
    def _spring_smooth_angle(self, target: float, current: float, velocity: float) -> tuple:
        """Critically-damped spring smoothing for angles with wraparound."""
        # Normalize target relative to current to handle -180/180 wraparound
        diff = target - current
        if diff > 180:
            diff -= 360
        elif diff < -180:
            diff += 360
        
        adjusted_target = current + diff
        new_pos, new_vel = self._spring_smooth(adjusted_target, current, velocity)
        
        # Normalize result to -180..180
        while new_pos > 180:
            new_pos -= 360
        while new_pos < -180:
            new_pos += 360
        
        return new_pos, new_vel
    
    def on_frame(self, frame: int, state: VehicleState, v2v_data: Dict[str, Any]):
        """Update spectator camera to follow vehicle from above with smooth spring dynamics."""
        if frame % self.update_interval != 0:
            return
        
        # Get vehicle transform
        vehicle_transform = self.vehicle.get_transform()
        vehicle_location = vehicle_transform.location
        vehicle_rotation = vehicle_transform.rotation
        
        target_x = vehicle_location.x
        target_y = vehicle_location.y
        target_z = vehicle_location.z + self.height
        
        # Initialize camera on first frame (snap to position, no smoothing)
        if self._cam_x is None:
            self._cam_x = target_x
            self._cam_y = target_y
            self._cam_z = target_z
            self._cam_yaw = 180.0 if self._is_topdown else vehicle_rotation.yaw
            self._vel_x = 0.0
            self._vel_y = 0.0
            self._vel_z = 0.0
            self._vel_yaw = 0.0
        else:
            # Apply critically-damped spring to X, Y, Z
            self._cam_x, self._vel_x = self._spring_smooth(
                target_x, self._cam_x, self._vel_x)
            self._cam_y, self._vel_y = self._spring_smooth(
                target_y, self._cam_y, self._vel_y)
            self._cam_z, self._vel_z = self._spring_smooth(
                target_z, self._cam_z, self._vel_z)
            
            # For pure top-down (-90°), lock yaw at 0 to prevent rotation flicker
            # For angled views, smoothly follow vehicle yaw
            if not self._is_topdown:
                self._cam_yaw, self._vel_yaw = self._spring_smooth_angle(
                    vehicle_rotation.yaw, self._cam_yaw, self._vel_yaw)
        
        # Build spectator transform
        spectator_transform = carla.Transform(
            carla.Location(x=self._cam_x, y=self._cam_y, z=self._cam_z),
            carla.Rotation(pitch=self.pitch, yaw=self._cam_yaw, roll=0)
        )
        self.spectator.set_transform(spectator_transform)
    
    def on_complete(self, total_frames: int, elapsed_time: float):
        """No cleanup needed for spectator."""
        pass


class V2VMessageLogger(ScenarioObserver):
    """
    Detailed V2V message logger for research analysis.
    
    Logs all V2V BSM messages exchanged between vehicles, including:
    - Sender and receiver vehicle IDs
    - BSM core data (position, speed, heading, acceleration)
    - Transmission metadata (distance, signal timing)
    - Threat assessments between vehicle pairs
    
    Creates a dedicated CSV file for V2V message analysis.
    """
    
    def __init__(self, v2v_network, output_path: Optional[Path] = None,
                 log_interval_frames: int = 10):
        """
        Initialize V2V message logger.
        
        Args:
            v2v_network: V2VNetworkEnhanced instance to log from
            output_path: CSV file path. If None, auto-generated in logs/
            log_interval_frames: How often to log V2V messages (default every 10 frames = 2Hz at 20 FPS)
        """
        from ..v2v import V2VNetworkEnhanced
        
        self.v2v: V2VNetworkEnhanced = v2v_network
        self.log_interval = log_interval_frames
        
        if output_path is None:
            log_dir = Path(__file__).parent.parent.parent / 'logs'
            log_dir.mkdir(exist_ok=True)
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            output_path = log_dir / f'v2v_messages_{timestamp}.csv'
        
        self.output_path = output_path
        self.csv_file = None
        self.writer = None
        self.total_messages = 0
        self._file_opened = False
    
    def _open_csv(self):
        """Open CSV file and write header."""
        self.csv_file = open(self.output_path, 'w', newline='')
        fieldnames = [
            # Timing
            'frame', 'sim_timestamp', 'wall_timestamp',
            # Sender info
            'sender_id', 'sender_x', 'sender_y', 'sender_z',
            'sender_speed_ms', 'sender_heading', 'sender_accel',
            'sender_brake_status', 'sender_msg_count',
            # Receiver info (neighbor who receives this BSM)
            'receiver_id',
            # Transmission info
            'distance_m', 'relative_speed_ms',
            # Threat assessment
            'threat_level', 'time_to_collision',
            # Network stats
            'total_neighbors', 'cooperative_shares'
        ]
        self.writer = csv.DictWriter(self.csv_file, fieldnames=fieldnames)
        self.writer.writeheader()
        self._file_opened = True
    
    def on_frame(self, frame: int, state: VehicleState, v2v_data: Dict[str, Any]):
        """Log V2V message exchanges at specified interval."""
        if frame % self.log_interval != 0:
            return
        
        # Lazy file opening
        if not self._file_opened:
            self._open_csv()
        
        sim_timestamp = state.frame * 0.05  # 20 FPS = 0.05s per frame
        wall_timestamp = datetime.now().isoformat()
        
        # Get all BSM messages
        all_bsm = self.v2v.get_all_bsm()
        network_stats = self.v2v.get_network_stats()
        
        # Log each vehicle's BSM broadcast to its neighbors
        for sender_id, sender_bsm in all_bsm.items():
            neighbors = self.v2v.get_neighbors(sender_id)
            threats = self.v2v.get_threats(sender_id)
            
            # Log a row for each neighbor that receives this BSM
            for neighbor_bsm in neighbors:
                receiver_id = neighbor_bsm.vehicle_id
                
                # Calculate relative info
                distance = self.v2v.get_distance(sender_id, receiver_id) or 0
                relative_speed = sender_bsm.speed - neighbor_bsm.speed
                
                # Find threat info for this pair
                threat_info = next(
                    (t for t in threats if t.get('other_vehicle_id') == receiver_id),
                    {'level': 0, 'ttc': 999}
                )
                
                # Write row
                self.writer.writerow({  # type: ignore
                    'frame': frame,
                    'sim_timestamp': f"{sim_timestamp:.2f}",
                    'wall_timestamp': wall_timestamp,
                    'sender_id': sender_id,
                    'sender_x': f"{sender_bsm.latitude:.2f}",
                    'sender_y': f"{sender_bsm.longitude:.2f}",
                    'sender_z': f"{sender_bsm.elevation:.2f}",
                    'sender_speed_ms': f"{sender_bsm.speed:.2f}",
                    'sender_heading': f"{sender_bsm.heading:.1f}",
                    'sender_accel': f"{sender_bsm.longitudinal_accel:.2f}",
                    'sender_brake_status': sender_bsm.brake_status.name,
                    'sender_msg_count': sender_bsm.msg_count,
                    'receiver_id': receiver_id,
                    'distance_m': f"{distance:.2f}",
                    'relative_speed_ms': f"{relative_speed:.2f}",
                    'threat_level': threat_info.get('level', 0),
                    'time_to_collision': f"{threat_info.get('ttc', 999):.2f}",
                    'total_neighbors': len(neighbors),
                    'cooperative_shares': network_stats.get('cooperative_shares', 0)
                })
                self.total_messages += 1
    
    def on_complete(self, total_frames: int, elapsed_time: float):
        """Close CSV file and print summary."""
        if self.csv_file:
            self.csv_file.close()
            print(f"📡 Logged {self.total_messages} V2V messages to {self.output_path}")
