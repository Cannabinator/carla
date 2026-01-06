#!/usr/bin/env python3
"""
V2V Communication Scenario for CARLA 0.9.16
Scientifically accurate simulation with proper physics and vehicle control.
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
import math

sys.path.insert(0, str(Path(__file__).parent.parent.parent))
from src.v2v import V2VNetwork
from src.config import DEFAULT_SIM_CONFIG, DEFAULT_VIZ_CONFIG, DEFAULT_V2V_CONFIG, DEFAULT_VEHICLE_CONFIG
from src.utils.carla_utils import (
    calculate_speed, calculate_distance_3d, setup_synchronous_mode,
    restore_world_settings, setup_traffic_manager, get_fresh_velocity,
    spawn_vehicle, destroy_actors
)
from src.visualization.lidar import create_ego_lidar_stream

# Setup logging
log_dir = Path(__file__).parent.parent.parent / 'logs'
log_dir.mkdir(exist_ok=True)
log_file = log_dir / f"v2v_scenario_{datetime.now().strftime('%Y%m%d_%H%M%S')}.log"

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s',
    handlers=[
        logging.FileHandler(log_file),
        logging.StreamHandler()
    ]
)
logger = logging.getLogger(__name__)


def visualize_v2v_connections(world, network, ego_id, frame_duration=0.2, config=DEFAULT_VIZ_CONFIG):
    """Draw V2V range and connections - minimal visualization for scientific clarity.
    
    Args:
        world: CARLA world instance
        network: V2VNetwork instance
        ego_id: ID of the ego vehicle
        frame_duration: How long to display the visualization
        config: VisualizationConfig instance
    """
    ego_state = network.get_state(ego_id)
    if not ego_state:
        return
    
    ego_loc = carla.Location(*ego_state.location)
    debug = world.debug
    
    # Draw minimal range indicator circle
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
    
    # Draw thin lines to neighbors
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


def run_v2v_scenario(host='192.168.1.110', port=2000, duration=60, v2v_range=50.0, num_vehicles=15, 
                     enable_lidar=True, web_port=8000, lidar_quality='high'):
    """Run V2V communication scenario with proper physics and control.
    
    Args:
        host: CARLA server IP
        port: CARLA server port
        duration: Scenario duration in seconds
        v2v_range: V2V communication range in meters
        num_vehicles: Number of vehicles to spawn
        enable_lidar: Enable ego vehicle LiDAR streaming to web viewer
        web_port: Web server port for LiDAR visualization
        lidar_quality: 'high' or 'fast' - LiDAR quality/performance tradeoff
    """
    client = None
    world = None
    actors = []
    original_settings = None
    lidar_api = None
    
    try:
        # Connect to CARLA server
        config = DEFAULT_SIM_CONFIG
        print(f"🔄 Connecting to {host}:{port}")
        logger.info(f"Connecting to CARLA server at {host}:{port}")
        client = carla.Client(host, port)
        client.set_timeout(config.timeout)
        world = client.get_world()
        print(f"✓ Connected to {world.get_map().name}\n")
        logger.info(f"Connected to map: {world.get_map().name}")
        
        # Configure synchronous mode for deterministic physics
        original_settings = setup_synchronous_mode(world, config.fixed_delta_seconds)
        logger.info(f"Synchronous mode enabled: delta={config.fixed_delta_seconds}s (20 FPS)")
        
        # Set deterministic seeds
        random.seed(config.random_seed)
        np.random.seed(config.random_seed)
        
        # Initialize V2V network
        v2v = V2VNetwork(max_range=v2v_range)
        
        # Get blueprint library and spawn points
        bp_lib = world.get_blueprint_library()
        spawn_points = world.get_map().get_spawn_points()
        
        if len(spawn_points) < num_vehicles:
            print(f"⚠️  Only {len(spawn_points)} spawn points available")
            num_vehicles = len(spawn_points)
        
        # Shuffle spawn points deterministically
        random.shuffle(spawn_points)
        
        # Spawn ego vehicle (leader) with specific attributes
        vehicle_config = DEFAULT_VEHICLE_CONFIG
        v2v_config = DEFAULT_V2V_CONFIG
        
        ego_bp = bp_lib.filter(vehicle_config.ego_blueprint)[0]
        ego_bp.set_attribute('color', vehicle_config.ego_color)
        ego = world.spawn_actor(ego_bp, spawn_points[0])
        actors.append(ego)
        v2v.register(v2v_config.ego_vehicle_id, ego)
        logger.info(f"Ego vehicle spawned - ID: {ego.id}, Type: {ego_bp.id}, Location: {spawn_points[0].location}")
        print(f"👑 Ego vehicle spawned (RED) at {spawn_points[0].location}")
        
        # Initialize ego-only LiDAR streaming if enabled
        if enable_lidar:
            print(f"\n📡 Initializing ego LiDAR streaming...")
            lidar_api = create_ego_lidar_stream(
                world=world,
                ego_vehicle=ego,
                web_port=web_port,
                high_quality=(lidar_quality == 'high')
            )
            logger.info(f"Ego LiDAR streaming initialized on port {web_port} ({lidar_quality} quality)")
            print(f"✓ LiDAR viewer ready\n")
        
        # Setup Traffic Manager with proper configuration
        tm = setup_traffic_manager(client, config.tm_port, config.tm_seed, 
                                   config.use_hybrid_physics, config.hybrid_physics_radius)
        logger.info(f"Traffic Manager configured: sync=True, seed={config.tm_seed}, hybrid_physics={config.use_hybrid_physics}")
        
        # Configure traffic manager behavior for realistic speeds
        tm.global_percentage_speed_difference(config.global_speed_difference)
        tm.set_global_distance_to_leading_vehicle(config.safety_distance)
        logger.info(f"TM behavior: speed_diff={config.global_speed_difference}%, safety_distance={config.safety_distance}m")
        
        # Enable autopilot on ego vehicle for reproducible behavior
        ego.set_autopilot(True, config.tm_port)
        tm.update_vehicle_lights(ego, True)
        tm.ignore_lights_percentage(ego, 0)
        tm.auto_lane_change(ego, True)
        tm.vehicle_percentage_speed_difference(ego, config.ego_speed_difference)
        logger.info(f"Ego autopilot enabled on TM port {config.tm_port} - speed_diff={config.ego_speed_difference}%, lane_change=True, respect_lights=True")
        
        # Spawn traffic vehicles with variety
        vehicle_bps = [x for x in bp_lib.filter('vehicle.*') 
                      if int(x.get_attribute('number_of_wheels')) == vehicle_config.min_wheels]
        
        print(f"🚗 Spawning {num_vehicles-1} traffic vehicles...")
        spawned_count = 0
        for i in range(1, num_vehicles):
            if i >= len(spawn_points):
                break
            try:
                veh_bp = random.choice(vehicle_bps)
                # Randomize vehicle color
                if veh_bp.has_attribute('color'):
                    color = random.choice(veh_bp.get_attribute('color').recommended_values)
                    veh_bp.set_attribute('color', color)
                
                veh = world.spawn_actor(veh_bp, spawn_points[i])
                veh.set_autopilot(True, config.tm_port)
                tm.update_vehicle_lights(veh, True)
                actors.append(veh)
                v2v.register(i, veh)
                spawned_count += 1
            except RuntimeError as e:
                # Spawn point collision, skip
                continue
        
        print(f"✓ Spawned {spawned_count} traffic vehicles")
        print(f"✓ Total V2V nodes: {len(actors)}\n")
        logger.info(f"Total vehicles spawned: {len(actors)} (1 ego + {spawned_count} traffic)")
        
        # Warm-up: tick world to let Traffic Manager initialize routes
        print("⏱️  Warming up simulation (initializing routes)...")
        logger.info("Starting warmup phase...")
        for i in range(config.warmup_frames):
            world.tick()
            if i % config.debug_log_interval_frames == 0:
                vel = ego.get_velocity()
                _, speed_kmh = calculate_speed(vel)
                logger.debug(f"Warmup frame {i}: ego speed={speed_kmh:.2f} km/h")
        logger.info("Warmup complete")
        
        # Simulation loop
        print(f"🚀 Running V2V scenario for {duration}s...")
        start_time = time.time()
        frame = 0
        last_stats_time = 0
        
        while time.time() - start_time < duration:
            # Tick world to advance simulation - CARLA 0.9.16 returns frame ID, not snapshot
            frame += 1
            world.tick()  # Advance simulation by one step
            
            # CRITICAL: Get snapshot IMMEDIATELY after tick for freshest data
            # This must be done right after tick() to minimize staleness
            snapshot = world.get_snapshot()
            
            # Update V2V network every 0.2s (4 frames at 20 FPS)
            # CRITICAL: Pass snapshot from tick to ensure fresh velocity data
            if frame % config.v2v_update_interval_frames == 0:
                v2v.update(force=True, snapshot=snapshot)
                visualize_v2v_connections(world, v2v, v2v_config.ego_vehicle_id, frame_duration=0.25)
            
            # CRITICAL: Use snapshot from IMMEDIATELY after tick - it contains FRESH velocity data
            # Using actor.get_velocity() returns CACHED data from previous tick
            # Using snapshot from immediately after tick() guarantees fresh data from THIS tick
            ego_snapshot = snapshot.find(ego.id)
            if not ego_snapshot:
                logger.error(f"Frame {frame}: Ego vehicle not found in snapshot!")
                continue
            
            # Extract data from snapshot (guaranteed fresh from this tick)
            trans = ego_snapshot.get_transform()
            vel_snapshot = ego_snapshot.get_velocity()  # This is from the snapshot - fresh data!
            angular_vel_snapshot = ego_snapshot.get_angular_velocity()
            
            # Also get control from actor (this is okay to get from actor)
            control = ego.get_control()
            
            # Log detailed debug state every 0.5s
            if frame % config.debug_log_interval_frames == 0:
                _, speed_kmh = calculate_speed(vel_snapshot)
                acc_snapshot = ego_snapshot.get_acceleration()
                
                logger.debug(f"Frame {frame}: speed={speed_kmh:.2f}km/h, "
                           f"throttle={control.throttle:.3f}, brake={control.brake:.3f}, steer={control.steer:.3f}, "
                           f"acc=({acc_snapshot.x:.2f},{acc_snapshot.y:.2f},{acc_snapshot.z:.2f}), "
                           f"loc=({trans.location.x:.1f},{trans.location.y:.1f})")
            
            # Print detailed stats every 2s (using snapshot data from above)
            current_time = time.time()
            if current_time - last_stats_time >= config.stats_display_interval_seconds:
                neighbors = v2v.get_neighbors(v2v_config.ego_vehicle_id)
                
                # Use velocity and transform from snapshot collected above (guaranteed fresh)
                # Velocity components in world frame (m/s) - FROM SNAPSHOT
                # Use velocity from snapshot (FRESH data)
                vel_x = vel_snapshot.x
                vel_y = vel_snapshot.y
                vel_z = vel_snapshot.z
                
                # Calculate speed magnitude
                speed_ms, speed_kmh = calculate_speed(vel_snapshot)
                
                # Position in world frame (meters)
                pos_x = trans.location.x
                pos_y = trans.location.y
                pos_z = trans.location.z
                
                # Orientation (degrees)
                yaw = trans.rotation.yaw
                pitch = trans.rotation.pitch
                roll = trans.rotation.roll
                
                # Angular velocity (rad/s to deg/s)
                ang_vel_x = np.degrees(angular_vel_snapshot.x)
                ang_vel_y = np.degrees(angular_vel_snapshot.y)
                ang_vel_z = np.degrees(angular_vel_snapshot.z)
                
                # V2V communication count
                num_neighbors = len(neighbors)
                
                # Get control inputs
                control = ego.get_control()
                
                print(f"\n{'='*85}")
                print(f"🚗 LEADING VEHICLE - Frame {frame:4d} | Sim: {frame*0.05:.1f}s | Real: {(current_time-start_time):.1f}s")
                print(f"{'='*85}")
                print(f"📍 Position:      X={pos_x:9.2f}m  Y={pos_y:9.2f}m  Z={pos_z:8.2f}m")
                print(f"🏃 Velocity:      Vx={vel_x:8.3f}  Vy={vel_y:8.3f}  Vz={vel_z:8.3f} m/s")
                print(f"⚡ Speed:         {speed_kmh:7.2f} km/h ({speed_ms:6.3f} m/s)")
                print(f"🧭 Orientation:   Yaw={yaw:7.2f}°  Pitch={pitch:6.2f}°  Roll={roll:6.2f}°")
                print(f"🎮 Control:       Throttle={control.throttle:.3f}  Brake={control.brake:.3f}  Steer={control.steer:.3f}")
                print(f"🔄 Angular Vel:   ωx={ang_vel_x:7.2f}  ωy={ang_vel_y:7.2f}  ωz={ang_vel_z:7.2f} °/s")
                print(f"📡 V2V Comms:     {num_neighbors}/{len(actors)-1} vehicles in range")
                if lidar_api:
                    point_count = lidar_api.get_point_count()
                    print(f"🎯 LiDAR Points:  {point_count:,} points/frame")
                
                if num_neighbors > 0:
                    viz_config = DEFAULT_VIZ_CONFIG
                    print(f"\n   🔗 Connected Vehicles:")
                    for i, neighbor in enumerate(neighbors[:viz_config.max_neighbors_displayed], 1):
                        dist = calculate_distance_3d(neighbor.location, (pos_x, pos_y, pos_z))
                        # Convert neighbor speed from m/s to km/h
                        neighbor_speed_kmh = neighbor.speed * 3.6
                        rel_speed = neighbor_speed_kmh - speed_kmh
                        print(f"      {i}. ID {neighbor.vehicle_id:3d}: {neighbor_speed_kmh:6.2f} km/h | "
                              f"Dist: {dist:6.2f}m | Δv: {rel_speed:+6.2f} km/h")
                    if num_neighbors > viz_config.max_neighbors_displayed:
                        print(f"      ... and {num_neighbors - viz_config.max_neighbors_displayed} more")
                
                print(f"{'='*85}\n")
                
                # Compact logging
                logger.info(f"F{frame:04d} | Pos:({pos_x:.1f},{pos_y:.1f},{pos_z:.1f}) | "
                           f"Vel:({vel_x:.2f},{vel_y:.2f},{vel_z:.2f}) | "
                           f"Spd:{speed_kmh:.1f}km/h | Yaw:{yaw:.1f}° | "
                           f"Ctrl:T{control.throttle:.2f}/B{control.brake:.2f}/S{control.steer:.2f} | "
                           f"V2V:{num_neighbors}")
                
                last_stats_time = current_time
        
        print(f"\n✓ Scenario completed ({frame} frames, {frame/20:.1f}s simulated)")
        logger.info(f"Scenario completed: {frame} frames, {frame/20:.1f}s simulated")
    
    except KeyboardInterrupt:
        print("\n⚠️  Interrupted by user")
        logger.warning("Scenario interrupted by user")
    except Exception as e:
        print(f"\n❌ Error occurred: {e}")
        logger.error(f"Error occurred: {e}", exc_info=True)
        import traceback
        traceback.print_exc()
    finally:
        # Cleanup - CRITICAL: Restore settings and destroy actors
        print("\n🧹 Cleaning up...")
        logger.info("Starting cleanup...")
        
        # Stop LiDAR streaming first
        if lidar_api:
            lidar_api.stop()
            print("✓ LiDAR streaming stopped")
            logger.info("LiDAR streaming stopped")
        
        # Restore original settings
        if world and original_settings:
            restore_world_settings(world, original_settings)
            print("✓ Restored world settings")
            logger.info("World settings restored")
        
        # Destroy all spawned actors
        if client:
            destroy_actors(client, actors)
            print(f"✓ Destroying {len(actors)} actors...")
            logger.info(f"Destroying {len(actors)} actors")
        
        print("✓ Cleanup complete\n")
        print(f"📝 Log file saved: {log_file}")
        logger.info(f"Cleanup complete. Log saved to: {log_file}")


def log_vehicle_state(ego: carla.Actor, ego_id: int, v2v: V2VNetwork, frame: int, logger):
    """Log detailed vehicle state including V2V neighbors."""
    transform = ego.get_transform()
    velocity_vec = ego.get_velocity()
    
    # Calculate speed properly (m/s to km/h)
    speed_ms = math.sqrt(velocity_vec.x**2 + velocity_vec.y**2 + velocity_vec.z**2)
    speed_kmh = speed_ms * 3.6
    
    # Get control - handle autopilot mode
    try:
        control = ego.get_control()
        throttle = control.throttle
        brake = control.brake
        steer = control.steer
    except:
        # If control not available (rare case), use zeros
        throttle = brake = steer = 0.0
    
    neighbors = v2v.get_neighbors(ego_id)
    
    logger.info(
        f"F{frame:04d} | "
        f"Pos:({transform.location.x:.1f},{transform.location.y:.1f},{transform.location.z:.1f}) | "
        f"Vel:({velocity_vec.x:.2f},{velocity_vec.y:.2f},{velocity_vec.z:.2f}) | "
        f"Spd:{speed_kmh:.1f}km/h | "
        f"Yaw:{transform.rotation.yaw:.1f}° | "
        f"Ctrl:T{throttle:.2f}/B{brake:.2f}/S{steer:.2f} | "
        f"V2V:{len(neighbors)}"
    )


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='V2V Communication Scenario')
    parser.add_argument('--host', default='192.168.1.110', help='CARLA server IP')
    parser.add_argument('--port', type=int, default=2000, help='CARLA server port')
    parser.add_argument('--duration', type=int, default=60, help='Scenario duration (seconds)')
    parser.add_argument('--v2v-range', type=float, default=50.0, dest='v2v_range', help='V2V range (meters)')
    parser.add_argument('--vehicles', type=int, default=15, help='Number of vehicles')
    parser.add_argument('--enable-lidar', action='store_true', default=True, help='Enable ego LiDAR streaming (default: True)')
    parser.add_argument('--no-lidar', action='store_false', dest='enable_lidar', help='Disable LiDAR streaming')
    parser.add_argument('--web-port', type=int, default=8000, help='Web server port for LiDAR viewer')
    parser.add_argument('--lidar-quality', choices=['high', 'fast'], default='high', help='LiDAR quality (high=dense, fast=lighter)')
    
    args = parser.parse_args()
    
    run_v2v_scenario(
        host=args.host,
        port=args.port,
        duration=args.duration,
        v2v_range=args.v2v_range,
        num_vehicles=args.vehicles,
        enable_lidar=args.enable_lidar,
        web_port=args.web_port,
        lidar_quality=args.lidar_quality
    )
