#!/usr/bin/env python3
"""
V2V Communication Scenario - Refactored Version
Clean, maintainable implementation using modern Python patterns.
"""

import carla
import random
import time
import numpy as np
import argparse
import logging
from datetime import datetime
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).parent.parent.parent))

from src.v2v import V2VNetwork
from src.config import DEFAULT_SIM_CONFIG, DEFAULT_VIZ_CONFIG, DEFAULT_V2V_CONFIG, DEFAULT_VEHICLE_CONFIG
from src.utils import (
    CARLASession, VehicleState, ActorManager,
    calculate_distance_3d, setup_traffic_manager
)
from src.visualization.lidar import create_ego_lidar_stream

# Setup logging
log_dir = Path(__file__).parent.parent.parent / 'logs'
log_dir.mkdir(exist_ok=True)
log_file = log_dir / f"v2v_clean_{datetime.now().strftime('%Y%m%d_%H%M%S')}.log"

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s',
    handlers=[
        logging.FileHandler(log_file),
        logging.StreamHandler()
    ]
)
logger = logging.getLogger(__name__)


def visualize_v2v_connections(
    world: carla.World, 
    network: V2VNetwork, 
    ego_id: int, 
    frame_duration: float = 0.2, 
    config=DEFAULT_VIZ_CONFIG
):
    """Draw V2V range and connections."""
    ego_state = network.get_state(ego_id)
    if not ego_state:
        return
    
    ego_loc = carla.Location(*ego_state.location)
    debug = world.debug
    
    # Draw range circle
    num_segments = config.range_circle_segments
    range_m = network.max_range
    
    for i in range(num_segments):
        angle1 = (i / num_segments) * 2 * np.pi
        angle2 = ((i + 1) / num_segments) * 2 * np.pi
        
        x1 = ego_loc.x + range_m * np.cos(angle1)
        y1 = ego_loc.y + range_m * np.sin(angle1)
        x2 = ego_loc.x + range_m * np.cos(angle2)
        y2 = ego_loc.y + range_m * np.sin(angle2)
        
        p1 = carla.Location(x=x1, y=y1, z=ego_loc.z + config.range_circle_z_offset)
        p2 = carla.Location(x=x2, y=y2, z=ego_loc.z + config.range_circle_z_offset)
        
        debug.draw_line(p1, p2, thickness=config.range_circle_thickness,
                       color=carla.Color(*config.range_circle_color), 
                       life_time=frame_duration)
    
    # Draw connection lines
    neighbors = network.get_neighbors(ego_id)
    for neighbor in neighbors:
        neighbor_loc = carla.Location(*neighbor.location)
        debug.draw_line(
            ego_loc + carla.Location(z=config.connection_line_z_offset),
            neighbor_loc + carla.Location(z=config.connection_line_z_offset),
            thickness=config.connection_line_thickness,
            color=carla.Color(*config.connection_line_color),
            life_time=frame_duration
        )


def print_vehicle_stats(state: VehicleState, v2v_neighbors: list, total_vehicles: int, lidar_points: int = 0):
    """Print formatted vehicle statistics."""
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
    print(f"📡 V2V Comms:     {len(v2v_neighbors)}/{total_vehicles-1} vehicles in range")
    
    if lidar_points > 0:
        print(f"🎯 LiDAR Points:  {lidar_points:,} points/frame")
    
    if v2v_neighbors:
        viz_config = DEFAULT_VIZ_CONFIG
        print(f"\n   🔗 Connected Vehicles:")
        for i, neighbor in enumerate(v2v_neighbors[:viz_config.max_neighbors_displayed], 1):
            dist = calculate_distance_3d(neighbor.location, state.position)
            neighbor_speed_kmh = neighbor.speed * 3.6
            rel_speed = neighbor_speed_kmh - state.speed_kmh
            print(f"      {i}. ID {neighbor.vehicle_id:3d}: {neighbor_speed_kmh:6.2f} km/h | "
                  f"Dist: {dist:6.2f}m | Δv: {rel_speed:+6.2f} km/h")
        if len(v2v_neighbors) > viz_config.max_neighbors_displayed:
            print(f"      ... and {len(v2v_neighbors) - viz_config.max_neighbors_displayed} more")
    
    print(f"{'='*85}\n")


def run_v2v_scenario_clean(
    host: str = '192.168.1.110',
    port: int = 2000,
    duration: int = 60,
    v2v_range: float = 50.0,
    num_vehicles: int = 15,
    enable_lidar: bool = True,
    web_port: int = 8000,
    lidar_quality: str = 'high'
):
    """
    Run V2V scenario with clean, maintainable code.
    
    Args:
        host: CARLA server IP
        port: CARLA server port
        duration: Scenario duration in seconds
        v2v_range: V2V communication range in meters
        num_vehicles: Number of vehicles to spawn
        enable_lidar: Enable ego LiDAR streaming
        web_port: Web server port for LiDAR viewer
        lidar_quality: 'high' or 'fast'
    """
    lidar_api = None
    
    try:
        # Use context manager for automatic cleanup
        with CARLASession(host, port, DEFAULT_SIM_CONFIG) as session:
            
            print(f"🔄 Connecting to {host}:{port}")
            print(f"✓ Connected to {session.world.get_map().name}\n")
            
            # Set deterministic seeds
            random.seed(session.config.random_seed)
            np.random.seed(session.config.random_seed)
            
            # Initialize systems
            v2v = V2VNetwork(max_range=v2v_range)
            actor_mgr = ActorManager(session.world, session.bp_lib)
            
            # Limit vehicles to available spawn points
            num_vehicles = min(num_vehicles, len(session.spawn_points))
            random.shuffle(session.spawn_points)
            
            # Spawn ego vehicle
            vehicle_config = DEFAULT_VEHICLE_CONFIG
            ego = actor_mgr.spawn_ego(
                blueprint_id=vehicle_config.ego_blueprint,
                spawn_point=session.spawn_points[0],
                color=vehicle_config.ego_color
            )
            session.add_actor(ego)  # Register for cleanup
            v2v.register(0, ego)
            print(f"👑 Ego vehicle spawned (RED) at {session.spawn_points[0].location}")
            
            # Initialize ego LiDAR if enabled
            if enable_lidar:
                print(f"\n📡 Initializing ego LiDAR streaming...")
                lidar_api = create_ego_lidar_stream(
                    world=session.world,
                    ego_vehicle=ego,
                    web_port=web_port,
                    high_quality=(lidar_quality == 'high')
                )
                logger.info(f"Ego LiDAR streaming initialized on port {web_port} ({lidar_quality} quality)")
                print(f"✓ LiDAR viewer ready\n")
            
            # Setup Traffic Manager
            tm_config = session.config
            tm = setup_traffic_manager(
                session.client, 
                tm_config.tm_port, 
                tm_config.tm_seed,
                tm_config.use_hybrid_physics, 
                tm_config.hybrid_physics_radius
            )
            tm.global_percentage_speed_difference(tm_config.global_speed_difference)
            tm.set_global_distance_to_leading_vehicle(tm_config.safety_distance)
            
            # Enable autopilot on ego
            ego.set_autopilot(True, tm_config.tm_port)
            tm.update_vehicle_lights(ego, True)
            tm.ignore_lights_percentage(ego, 0)
            tm.auto_lane_change(ego, True)
            tm.vehicle_percentage_speed_difference(ego, tm_config.ego_speed_difference)
            
            # Spawn traffic vehicles
            print(f"🚗 Spawning {num_vehicles-1} traffic vehicles...")
            traffic = actor_mgr.spawn_traffic(
                num_vehicles - 1,
                session.spawn_points[1:]
            )
            
            # Register traffic with session and V2V
            for i, vehicle in enumerate(traffic):
                session.add_actor(vehicle)
                v2v.register(i + 1, vehicle)
                vehicle.set_autopilot(True, tm_config.tm_port)
                tm.update_vehicle_lights(vehicle, True)
            
            print(f"✓ Spawned {len(traffic)} traffic vehicles")
            print(f"✓ Total V2V nodes: {actor_mgr.count()}\n")
            
            # Warmup
            print("⏱️  Warming up simulation...")
            for _ in range(tm_config.warmup_frames):
                session.world.tick()
            logger.info("Warmup complete")
            
            # Main simulation loop
            print(f"🚀 Running V2V scenario for {duration}s...\n")
            start_time = time.time()
            frame = 0
            last_stats_time = 0
            
            v2v_config = DEFAULT_V2V_CONFIG
            
            while time.time() - start_time < duration:
                frame += 1
                session.world.tick()
                snapshot = session.world.get_snapshot()
                
                # Update V2V network
                if frame % tm_config.v2v_update_interval_frames == 0:
                    v2v.update(force=True, snapshot=snapshot)
                    visualize_v2v_connections(session.world, v2v, v2v_config.ego_vehicle_id, 0.25)
                
                # Get ego state from snapshot
                ego_snapshot = snapshot.find(ego.id)
                if not ego_snapshot:
                    continue
                
                # Create vehicle state object
                state = VehicleState.from_snapshot(
                    frame=frame,
                    actor_snapshot=ego_snapshot,
                    control=ego.get_control()
                )
                
                # Print stats every 2 seconds
                current_time = time.time()
                if current_time - last_stats_time >= tm_config.stats_display_interval_seconds:
                    neighbors = v2v.get_neighbors(v2v_config.ego_vehicle_id)
                    lidar_points = lidar_api.get_point_count() if lidar_api else 0
                    
                    print_vehicle_stats(
                        state=state,
                        v2v_neighbors=neighbors,
                        total_vehicles=actor_mgr.count(),
                        lidar_points=lidar_points
                    )
                    
                    # Compact logging
                    logger.info(f"F{frame:04d} | {state} | V2V:{len(neighbors)}")
                    last_stats_time = current_time
            
            print(f"\n✓ Scenario completed ({frame} frames, {frame/20:.1f}s simulated)")
            logger.info(f"Scenario completed: {frame} frames")
    
    except KeyboardInterrupt:
        print("\n⚠️  Interrupted by user")
        logger.warning("Scenario interrupted by user")
    except Exception as e:
        print(f"\n❌ Error: {e}")
        logger.error(f"Error: {e}", exc_info=True)
        import traceback
        traceback.print_exc()
    finally:
        # Stop LiDAR before context manager cleanup
        if lidar_api:
            lidar_api.stop()
            print("✓ LiDAR streaming stopped")
        
        # Context manager handles CARLA cleanup automatically
        print(f"\n📝 Log file saved: {log_file}")


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='V2V Scenario (Refactored Clean Version)')
    parser.add_argument('--host', default='192.168.1.110', help='CARLA server IP')
    parser.add_argument('--port', type=int, default=2000, help='CARLA server port')
    parser.add_argument('--duration', type=int, default=60, help='Scenario duration (seconds)')
    parser.add_argument('--v2v-range', type=float, default=50.0, dest='v2v_range', help='V2V range (meters)')
    parser.add_argument('--vehicles', type=int, default=15, help='Number of vehicles')
    parser.add_argument('--enable-lidar', action='store_true', default=True, help='Enable ego LiDAR')
    parser.add_argument('--no-lidar', action='store_false', dest='enable_lidar', help='Disable LiDAR')
    parser.add_argument('--web-port', type=int, default=8000, help='Web server port for LiDAR')
    parser.add_argument('--lidar-quality', choices=['high', 'fast'], default='high', help='LiDAR quality')
    
    args = parser.parse_args()
    
    run_v2v_scenario_clean(
        host=args.host,
        port=args.port,
        duration=args.duration,
        v2v_range=args.v2v_range,
        num_vehicles=args.vehicles,
        enable_lidar=args.enable_lidar,
        web_port=args.web_port,
        lidar_quality=args.lidar_quality
    )
