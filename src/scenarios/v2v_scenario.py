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


def visualize_v2v_connections(world, network, ego_id, frame_duration=0.2):
    """Draw V2V range and connections - minimal visualization for scientific clarity."""
    ego_state = network.get_state(ego_id)
    if not ego_state:
        return
    
    ego_loc = carla.Location(*ego_state.location)
    debug = world.debug
    
    # Draw minimal range indicator circle (16 segments, subtle color)
    num_segments = 16
    range_m = network.max_range
    
    for i in range(num_segments):
        angle1 = (i / num_segments) * 2 * np.pi
        angle2 = ((i + 1) / num_segments) * 2 * np.pi
        
        x1 = ego_loc.x + range_m * np.cos(angle1)
        y1 = ego_loc.y + range_m * np.sin(angle1)
        x2 = ego_loc.x + range_m * np.cos(angle2)
        y2 = ego_loc.y + range_m * np.sin(angle2)
        
        p1 = carla.Location(x=x1, y=y1, z=ego_loc.z + 0.2)
        p2 = carla.Location(x=x2, y=y2, z=ego_loc.z + 0.2)
        
        # Subtle green circle with low alpha
        debug.draw_line(p1, p2, thickness=0.02, color=carla.Color(0, 200, 0, 40), 
                       life_time=frame_duration)
    
    # Draw thin lines to neighbors
    neighbors = network.get_neighbors(ego_id)
    for neighbor in neighbors:
        neighbor_loc = carla.Location(*neighbor.location)
        # Subtle connection line
        debug.draw_line(
            ego_loc + carla.Location(z=1.0),
            neighbor_loc + carla.Location(z=1.0),
            thickness=0.02,
            color=carla.Color(100, 200, 255, 80),
            life_time=frame_duration
        )


def run_v2v_scenario(host='192.168.1.110', port=2000, duration=60, v2v_range=50.0, num_vehicles=15):
    """Run V2V communication scenario with proper physics and control."""
    client = None
    world = None
    actors = []
    original_settings = None
    
    try:
        # Connect to CARLA server
        print(f"🔄 Connecting to {host}:{port}")
        logger.info(f"Connecting to CARLA server at {host}:{port}")
        client = carla.Client(host, port)
        client.set_timeout(30.0)
        world = client.get_world()
        print(f"✓ Connected to {world.get_map().name}\n")
        logger.info(f"Connected to map: {world.get_map().name}")
        logger.info(f"Connected to map: {world.get_map().name}")
        
        # Save original settings
        original_settings = world.get_settings()
        
        # Configure synchronous mode for deterministic physics
        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 0.05  # 20 FPS for stable physics
        settings.no_rendering_mode = False  # Keep rendering for visualization
        world.apply_settings(settings)
        logger.info(f"Synchronous mode enabled: delta={settings.fixed_delta_seconds}s (20 FPS)")
        logger.info(f"Synchronous mode enabled: delta={settings.fixed_delta_seconds}s (20 FPS)")
        
        # Set deterministic seeds
        random.seed(42)
        np.random.seed(42)
        
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
        ego_bp = bp_lib.filter('vehicle.tesla.model3')[0]
        ego_bp.set_attribute('color', '255,0,0')   # Red color
        ego = world.spawn_actor(ego_bp, spawn_points[0])
        actors.append(ego)
        v2v.register(0, ego)
        logger.info(f"Ego vehicle spawned - ID: {ego.id}, Type: {ego_bp.id}, Location: {spawn_points[0].location}")
        print(f"👑 Ego vehicle spawned (RED) at {spawn_points[0].location}")
        logger.info(f"Ego vehicle spawned - ID: {ego.id}, Type: {ego_bp.id}, Location: {spawn_points[0].location}")
        
        # Setup Traffic Manager with proper configuration
        tm = client.get_trafficmanager(8000)
        tm.set_synchronous_mode(True)
        tm.set_random_device_seed(42)
        # DISABLE hybrid physics - it causes vehicles to teleport instead of drive!
        # tm.set_hybrid_physics_mode(True)  # DISABLED - causes zero velocity
        # tm.set_hybrid_physics_radius(70.0)  # DISABLED
        logger.info("Traffic Manager configured: sync=True, seed=42, hybrid_physics=False")
        
        # Configure traffic manager behavior for realistic speeds
        # Default speed is 70% of speed limit, we reduce to 30% for smoother city driving
        tm.global_percentage_speed_difference(30.0)  # 70% of speed limit
        tm.set_global_distance_to_leading_vehicle(2.5)  # 2.5m safety distance
        logger.info("TM behavior: speed_diff=30%, safety_distance=2.5m")
        
        # Enable autopilot on ego vehicle for reproducible behavior
        ego.set_autopilot(True, 8000)
        tm.update_vehicle_lights(ego, True)  # Auto-update lights
        tm.ignore_lights_percentage(ego, 0)  # Ego respects traffic lights
        tm.auto_lane_change(ego, True)  # Enable lane changes for natural movement
        tm.vehicle_percentage_speed_difference(ego, 20.0)  # Ego slightly slower for safety
        logger.info(f"Ego autopilot enabled on TM port 8000 - speed_diff=20%, lane_change=True, respect_lights=True")
        
        # Spawn traffic vehicles with variety
        vehicle_bps = [x for x in bp_lib.filter('vehicle.*') 
                      if int(x.get_attribute('number_of_wheels')) == 4]
        
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
                veh.set_autopilot(True, 8000)
                tm.update_vehicle_lights(veh, True)  # Auto-update lights
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
        for i in range(50):  # Longer warmup for TM route planning
            world.tick()
            if i % 10 == 0:
                vel = ego.get_velocity()
                speed = 3.6 * np.sqrt(vel.x**2 + vel.y**2 + vel.z**2)
                logger.debug(f"Warmup frame {i}: ego speed={speed:.2f} km/h")
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
            if frame % 4 == 0:
                v2v.update(force=True, snapshot=snapshot)  # Pass fresh snapshot!
                visualize_v2v_connections(world, v2v, 0, frame_duration=0.25)
            
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
            if frame % 10 == 0:
                speed_kmh = 3.6 * np.sqrt(vel_snapshot.x**2 + vel_snapshot.y**2 + vel_snapshot.z**2)
                acc_snapshot = ego_snapshot.get_acceleration()
                
                logger.debug(f"Frame {frame}: speed={speed_kmh:.2f}km/h, "
                           f"throttle={control.throttle:.3f}, brake={control.brake:.3f}, steer={control.steer:.3f}, "
                           f"acc=({acc_snapshot.x:.2f},{acc_snapshot.y:.2f},{acc_snapshot.z:.2f}), "
                           f"loc=({trans.location.x:.1f},{trans.location.y:.1f})")
            
            # Print detailed stats every 2s (using snapshot data from above)
            current_time = time.time()
            if current_time - last_stats_time >= 2:
                neighbors = v2v.get_neighbors(0)
                
                # Use velocity and transform from snapshot collected above (guaranteed fresh)
                # Velocity components in world frame (m/s) - FROM SNAPSHOT
                # Use velocity from snapshot (FRESH data)
                vel_x = vel_snapshot.x
                vel_y = vel_snapshot.y
                vel_z = vel_snapshot.z
                
                # Calculate speed magnitude
                speed_ms = np.linalg.norm([vel_x, vel_y, vel_z])
                speed_kmh = 3.6 * speed_ms
                
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
                
                if num_neighbors > 0:
                    print(f"\n   🔗 Connected Vehicles:")
                    for i, neighbor in enumerate(neighbors[:5], 1):
                        dist = np.linalg.norm([neighbor.location[0] - pos_x,
                                              neighbor.location[1] - pos_y,
                                              neighbor.location[2] - pos_z])
                        # Convert neighbor speed from m/s to km/h
                        neighbor_speed_kmh = neighbor.speed * 3.6
                        rel_speed = neighbor_speed_kmh - speed_kmh
                        print(f"      {i}. ID {neighbor.vehicle_id:3d}: {neighbor_speed_kmh:6.2f} km/h | "
                              f"Dist: {dist:6.2f}m | Δv: {rel_speed:+6.2f} km/h")
                    if num_neighbors > 5:
                        print(f"      ... and {num_neighbors - 5} more")
                
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
        
        # Restore original settings
        if world and original_settings:
            world.apply_settings(original_settings)
            print("✓ Restored world settings")
            logger.info("World settings restored")
        
        # Destroy all spawned actors
        if client and actors:
            print(f"✓ Destroying {len(actors)} actors...")
            logger.info(f"Destroying {len(actors)} actors")
            client.apply_batch([carla.command.DestroyActor(x) for x in actors])
        
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
    
    args = parser.parse_args()
    
    run_v2v_scenario(
        host=args.host,
        port=args.port,
        duration=args.duration,
        v2v_range=args.v2v_range,
        num_vehicles=args.vehicles
    )
