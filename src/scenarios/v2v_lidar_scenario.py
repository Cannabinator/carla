#!/usr/bin/env python3
"""
V2V LiDAR Visualization Scenario
Combines V2V communication with real-time LiDAR streaming to web browser.
"""

import carla
import random
import time
import numpy as np
import argparse
import logging
import asyncio
import threading
from datetime import datetime
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).parent.parent.parent))
from src.v2v import V2VNetwork
from src.config import DEFAULT_SIM_CONFIG, DEFAULT_V2V_CONFIG
from src.utils.carla_utils import (
    setup_synchronous_mode, restore_world_settings,
    setup_traffic_manager, destroy_actors
)
from src.visualization.lidar import (
    LiDARDataCollector, app, manager, set_collector
)

# Setup logging
log_dir = Path(__file__).parent.parent.parent / 'logs'
log_dir.mkdir(exist_ok=True)
log_file = log_dir / f"v2v_lidar_{datetime.now().strftime('%Y%m%d_%H%M%S')}.log"

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s',
    handlers=[
        logging.FileHandler(log_file),
        logging.StreamHandler()
    ]
)
logger = logging.getLogger(__name__)


def run_fastapi_server(host: str, port: int):
    """Run FastAPI server in background thread."""
    import uvicorn
    uvicorn.run(app, host=host, port=port, log_level="info")


def spawn_vehicle_with_retry(world, bp_lib, spawn_points, vehicle_id, max_attempts=10):
    """Try to spawn vehicle with multiple spawn points.
    
    Args:
        world: CARLA world
        bp_lib: Blueprint library
        spawn_points: List of available spawn points
        vehicle_id: Vehicle identifier
        max_attempts: Maximum spawn attempts
        
    Returns:
        Spawned vehicle actor or None
    """
    vehicle_bps = [x for x in bp_lib.filter('vehicle.*') 
                   if int(x.get_attribute('number_of_wheels')) == 4]
    
    # Try different spawn points
    attempted_indices = set()
    attempts = 0
    
    while attempts < max_attempts and len(attempted_indices) < len(spawn_points):
        attempts += 1
        
        # Select random spawn point not yet tried
        available_indices = set(range(len(spawn_points))) - attempted_indices
        if not available_indices:
            break
            
        spawn_idx = random.choice(list(available_indices))
        attempted_indices.add(spawn_idx)
        spawn_point = spawn_points[spawn_idx]
        
        try:
            # Create vehicle blueprint
            veh_bp = random.choice(vehicle_bps)
            if vehicle_id == 0:
                # Ego vehicle - red
                if veh_bp.has_attribute('color'):
                    veh_bp.set_attribute('color', '255,0,0')
            else:
                # Random color for others
                if veh_bp.has_attribute('color'):
                    color = random.choice(veh_bp.get_attribute('color').recommended_values)
                    veh_bp.set_attribute('color', color)
            
            # Attempt spawn
            vehicle = world.try_spawn_actor(veh_bp, spawn_point)
            if vehicle:
                logger.info(f"Vehicle {vehicle_id} spawned at spawn point {spawn_idx}")
                return vehicle
                
        except Exception as e:
            logger.debug(f"Spawn attempt {attempts} failed: {e}")
            continue
    
    logger.warning(f"Failed to spawn vehicle {vehicle_id} after {attempts} attempts")
    return None


def run_v2v_lidar_scenario(
    carla_host='192.168.1.110',
    carla_port=2000,
    web_host='0.0.0.0',
    web_port=8000,
    duration=300,
    num_vehicles=3,
    keep_server_alive=False
):
    """Run V2V scenario with LiDAR visualization.
    
    Args:
        carla_host: CARLA server IP
        carla_port: CARLA server port
        web_host: Web server host
        web_port: Web server port
        duration: Scenario duration in seconds
        num_vehicles: Number of vehicles to attempt spawning
        keep_server_alive: Keep web server running after scenario ends
    """
    client = None
    world = None
    actors = []
    original_settings = None
    lidar_collector = None
    
    # Get network IP for Windows access
    import socket
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        network_ip = s.getsockname()[0]
        s.close()
    except:
        network_ip = "192.168.1.113"  # fallback
    
    try:
        # CARLA scenario logic (can fail without stopping web server)
        
        # Connect to CARLA
        config = DEFAULT_SIM_CONFIG
        print(f"🔄 Connecting to CARLA at {carla_host}:{carla_port}")
        logger.info(f"Connecting to CARLA server at {carla_host}:{carla_port}")
        
        client = carla.Client(carla_host, carla_port)
        client.set_timeout(config.timeout)
        world = client.get_world()
        
        print(f"✓ Connected to {world.get_map().name}\n")
        logger.info(f"Connected to map: {world.get_map().name}")
        
        # Configure synchronous mode
        original_settings = setup_synchronous_mode(world, config.fixed_delta_seconds)
        logger.info(f"Synchronous mode enabled: delta={config.fixed_delta_seconds}s (20 FPS)")
        
        # Set deterministic seeds
        random.seed(config.random_seed)
        np.random.seed(config.random_seed)
        
        # Initialize V2V network and LiDAR collector
        v2v = V2VNetwork(max_range=50.0)
        lidar_collector = LiDARDataCollector(world, downsample_factor=1)  # No downsampling for dense point cloud
        
        # Register collector with server (MUST be before server starts streaming)
        set_collector(lidar_collector)
        logger.info("LiDAR collector registered with web server")
        
        # NOW start FastAPI server in background thread
        logger.info(f"Starting web server on {web_host}:{web_port}")
        server_thread = threading.Thread(
            target=run_fastapi_server,
            args=(web_host, web_port),
            daemon=True
        )
        server_thread.start()
        
        print(f"\n{'='*80}")
        print(f"🌐 Web Viewer URLs:")
        print(f"   Local (Ubuntu):    http://localhost:{web_port}")
        print(f"   Network (Windows): http://{network_ip}:{web_port}")
        print(f"{'='*80}\n")
        time.sleep(2)  # Give server time to start and trigger startup event
        
        # Get blueprint library and spawn points
        bp_lib = world.get_blueprint_library()
        spawn_points = world.get_map().get_spawn_points()
        
        if len(spawn_points) == 0:
            raise RuntimeError("No spawn points available on this map")
        
        num_vehicles = min(num_vehicles, len(spawn_points))
        
        random.shuffle(spawn_points)
        
        # Spawn vehicles with retry logic
        print(f"🚗 Spawning up to {num_vehicles} vehicles with LiDAR...")
        
        for i in range(num_vehicles):
            vehicle = spawn_vehicle_with_retry(world, bp_lib, spawn_points, i, max_attempts=10)
            
            if vehicle:
                actors.append(vehicle)
                
                # Register with V2V network
                v2v.register(i, vehicle)
                
                # Register with LiDAR collector (attach sensor)
                lidar_collector.register_vehicle(i, vehicle)
        
        if len(actors) == 0:
            raise RuntimeError("Failed to spawn any vehicles. Please check the map and spawn points.")
        
        print(f"✓ Spawned {len(actors)} vehicles with semantic LiDAR")
        print(f"✓ Total V2V nodes: {len(actors)}\n")
        logger.info(f"Total vehicles spawned: {len(actors)}")
        
        # Setup Traffic Manager (only if we have vehicles)
        tm = setup_traffic_manager(client, config.tm_port, config.tm_seed, 
                                   config.use_hybrid_physics, config.hybrid_physics_radius)
        tm.global_percentage_speed_difference(config.global_speed_difference)
        tm.set_global_distance_to_leading_vehicle(config.safety_distance)
        logger.info(f"Traffic Manager configured")
        
        # Enable autopilot for all vehicles
        for i, vehicle in enumerate(actors):
            vehicle.set_autopilot(True, config.tm_port)
            tm.update_vehicle_lights(vehicle, True)
            if i == 0:
                tm.vehicle_percentage_speed_difference(vehicle, config.ego_speed_difference)
        
        logger.info("All vehicles in autopilot mode")
        
        # Warmup
        print("⏱️  Warming up simulation (initializing routes and LiDAR)...")
        logger.info("Starting warmup phase...")
        for i in range(config.warmup_frames):
            world.tick()
        logger.info("Warmup complete")
        
        # Streaming is automatically handled by FastAPI startup event
        print("✓ Streaming active in FastAPI server\n")
        
        # Simulation loop
        print(f"🚀 Running V2V + LiDAR scenario for {duration}s...")
        print(f"📡 Open browser: http://localhost:{web_port}\n")
        
        start_time = time.time()
        frame = 0
        last_stats_time = 0
        
        while time.time() - start_time < duration:
            frame += 1
            world.tick()
            
            # Update V2V network
            if frame % config.v2v_update_interval_frames == 0:
                snapshot = world.get_snapshot()
                v2v.update(force=True, snapshot=snapshot)
            
            # Print stats every 5 seconds
            current_time = time.time()
            if current_time - last_stats_time >= 5.0:
                elapsed = current_time - start_time
                num_connected = len(manager.active_connections)
                
                print(f"[{elapsed:6.1f}s] Frame {frame:5d} | "
                      f"Vehicles: {len(actors)} | "
                      f"V2V Active | "
                      f"Web Clients: {num_connected}")
                
                logger.info(f"Frame {frame}: {len(actors)} vehicles, {num_connected} web clients")
                last_stats_time = current_time
        
        print(f"\n✓ Scenario completed ({frame} frames, {frame/20:.1f}s simulated)")
        logger.info(f"Scenario completed: {frame} frames")
        
    except KeyboardInterrupt:
        print("\n⚠️  Interrupted by user")
        logger.warning("Scenario interrupted by user")
    except Exception as e:
        print(f"\n❌ Scenario error: {e}")
        print(f"\n🌐 Web server still running at http://localhost:{web_port}")
        print("   You can view the static frontend or wait for scenario restart")
        logger.error(f"Scenario error: {e}", exc_info=True)
        import traceback
        traceback.print_exc()
    finally:
        # Cleanup
        print("\n🧹 Cleaning up...")
        logger.info("Starting cleanup...")
        
        # Cleanup LiDAR sensors
        if lidar_collector:
            lidar_collector.cleanup()
        
        # Restore world settings
        restore_world_settings(world, original_settings)
        logger.info("World settings restored")
        
        # Destroy actors
        destroy_actors(client, actors)
        logger.info(f"Destroyed {len(actors)} actors")
        
        print("✓ Cleanup complete\n")
        print(f"📝 Log file saved: {log_file}")
        print(f"\n🌐 Web server remains active at http://localhost:{web_port}")
        
        if keep_server_alive:
            print("\n⏸️  Keeping web server alive. Press Ctrl+C to exit...")
            logger.info("Web server keep-alive mode enabled")
            try:
                while True:
                    time.sleep(1)
            except KeyboardInterrupt:
                print("\n👋 Shutting down web server...")
        else:
            print("   (Will shut down when script exits)")
        
        logger.info(f"Cleanup complete. Log saved to: {log_file}")


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='V2V LiDAR Visualization Scenario')
    parser.add_argument('--carla-host', default='192.168.1.110', 
                       help='CARLA server IP')
    parser.add_argument('--carla-port', type=int, default=2000, 
                       help='CARLA server port')
    parser.add_argument('--web-host', default='0.0.0.0', 
                       help='Web server host (0.0.0.0 for all interfaces)')
    parser.add_argument('--web-port', type=int, default=8000, 
                       help='Web server port')
    parser.add_argument('--duration', type=int, default=300, 
                       help='Scenario duration (seconds)')
    parser.add_argument('--vehicles', type=int, default=3, 
                       help='Number of vehicles to attempt spawning')
    parser.add_argument('--keep-alive', action='store_true',
                       help='Keep web server running after scenario ends')
    
    args = parser.parse_args()
    
    run_v2v_lidar_scenario(
        carla_host=args.carla_host,
        carla_port=args.carla_port,
        web_host=args.web_host,
        web_port=args.web_port,
        duration=args.duration,
        num_vehicles=args.vehicles,
        keep_server_alive=args.keep_alive
    )
