#!/usr/bin/env python3
"""
V2V Communication Scenario with API Server
Demonstrates BSM-based V2V communication with 2 Hz update rate and REST API.
"""

import carla
import random
import numpy as np
import time
import argparse
import sys
import threading
import uvicorn

# Add project root to path
sys.path.insert(0, '/home/workstation/carla')

from src.v2v import V2VNetworkEnhanced, create_v2v_api, create_bsm_from_carla


def print_one_line(text: str):
    """Print one line, clearing previous line"""
    sys.stdout.write('\r' + ' ' * 120 + '\r')  # Clear line
    sys.stdout.write(text)
    sys.stdout.flush()


def run_api_server(v2v_network, port=8001):
    """Run V2V API server in separate thread"""
    api = create_v2v_api(v2v_network, port)
    config = uvicorn.Config(api.app, host="0.0.0.0", port=port, log_level="warning")
    server = uvicorn.Server(config)
    server.run()


def main():
    parser = argparse.ArgumentParser(description='V2V Communication Scenario with API')
    parser.add_argument('--host', type=str, default='192.168.1.110',
                       help='CARLA server host (default: 192.168.1.110)')
    parser.add_argument('--port', type=int, default=2000,
                       help='CARLA server port (default: 2000)')
    parser.add_argument('--api-port', type=int, default=8001,
                       help='V2V API port (default: 8001)')
    parser.add_argument('--seed', type=int, default=42,
                       help='Random seed for reproducibility')
    parser.add_argument('--duration', type=int, default=60,
                       help='Scenario duration in seconds (default: 60)')
    parser.add_argument('--num-vehicles', type=int, default=20,
                       help='Number of traffic vehicles (default: 20)')
    parser.add_argument('--v2v-range', type=float, default=150.0,
                       help='V2V communication range in meters (default: 150)')
    parser.add_argument('--enable-coop', action='store_true',
                       help='Enable cooperative perception (bidirectional sharing)')
    
    args = parser.parse_args()
    
    # Set random seed
    random.seed(args.seed)
    np.random.seed(args.seed)
    
    client = None
    world = None
    actors = []
    
    try:
        # Connect to CARLA
        print(f"Connecting to CARLA server at {args.host}:{args.port}...")
        client = carla.Client(args.host, args.port)
        client.set_timeout(30.0)
        world = client.get_world()
        print("✓ Connected to CARLA")
        
        # Setup synchronous mode with 2 Hz tick rate (0.5s per tick)
        original_settings = world.get_settings()
        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 0.5  # 2 Hz simulation tick
        world.apply_settings(settings)
        print("✓ Synchronous mode enabled (2 Hz tick rate)")
        
        # Get blueprint library
        blueprint_library = world.get_blueprint_library()
        vehicle_bp = blueprint_library.filter('vehicle.*')[0]
        
        # Get spawn points
        spawn_points = world.get_map().get_spawn_points()
        random.shuffle(spawn_points)
        
        # Spawn ego vehicle
        print("Spawning ego vehicle...")
        ego_vehicle = world.spawn_actor(vehicle_bp, spawn_points[0])
        actors.append(ego_vehicle)
        print(f"✓ Ego vehicle spawned (ID: {ego_vehicle.id})")
        
        # Setup traffic manager
        tm = client.get_trafficmanager(8000)
        tm.set_synchronous_mode(True)
        tm.set_random_device_seed(args.seed)
        
        # Spawn traffic vehicles
        print(f"Spawning {args.num_vehicles} traffic vehicles...")
        for i in range(args.num_vehicles):
            if i + 1 >= len(spawn_points):
                break
            
            traffic_vehicle = world.spawn_actor(vehicle_bp, spawn_points[i + 1])
            traffic_vehicle.set_autopilot(True, 8000)
            actors.append(traffic_vehicle)
        
        print(f"✓ Spawned {len(actors)-1} traffic vehicles")
        
        # Initialize V2V network with 2 Hz update rate
        print(f"Initializing V2V network (range: {args.v2v_range}m, rate: 2 Hz)...")
        v2v = V2VNetworkEnhanced(
            max_range=args.v2v_range,
            update_rate_hz=2.0,
            world=world
        )
        
        # Register all vehicles
        v2v.register(ego_vehicle.id, ego_vehicle)
        for actor in actors[1:]:
            v2v.register(actor.id, actor)
        
        # Enable cooperative perception if requested
        if args.enable_coop:
            v2v.enable_bidirectional_sharing(ego_vehicle.id)
            print("✓ Cooperative perception enabled")
        
        print(f"✓ Registered {len(actors)} vehicles in V2V network")
        
        # Start API server in separate thread
        print(f"Starting V2V API server on port {args.api_port}...")
        api_thread = threading.Thread(
            target=run_api_server,
            args=(v2v, args.api_port),
            daemon=True
        )
        api_thread.start()
        print(f"✓ V2V API server started at http://0.0.0.0:{args.api_port}")
        print(f"  API endpoints: http://localhost:{args.api_port}/docs")
        
        # Setup spectator to follow ego vehicle
        spectator = world.get_spectator()
        
        # Run scenario
        print(f"\nRunning scenario for {args.duration} seconds...")
        print("=" * 120)
        
        start_time = time.time()
        tick_count = 0
        
        while time.time() - start_time < args.duration:
            # Tick world (2 Hz - advances 0.5s per tick)
            world.tick()
            tick_count += 1
            
            # Update V2V network (enforces 2 Hz rate internally)
            v2v.update()
            
            # Get one-line status
            status_line = v2v.get_one_line_status(ego_vehicle.id)
            
            # Add tick and elapsed time info
            elapsed = time.time() - start_time
            full_status = f"[Tick: {tick_count:04d} | {elapsed:.1f}s] {status_line}"
            
            # Print one line (overwrites previous)
            print_one_line(full_status)
            
            # Update spectator to follow ego vehicle
            if tick_count % 2 == 0:  # Update camera every 2 ticks (1 second)
                ego_transform = ego_vehicle.get_transform()
                spectator.set_transform(carla.Transform(
                    ego_transform.location + carla.Location(x=-10, z=6),
                    carla.Rotation(pitch=-20, yaw=ego_transform.rotation.yaw)
                ))
        
        print("\n" + "=" * 120)
        print("\nScenario completed!")
        
        # Print final statistics
        stats = v2v.get_network_stats()
        print("\nV2V Network Statistics:")
        print(f"  Total vehicles: {len(actors)}")
        print(f"  Total BSM messages sent: {stats['total_messages_sent']}")
        print(f"  Average neighbors per vehicle: {stats['average_neighbors']:.1f}")
        print(f"  Max neighbors observed: {stats['max_neighbors']}")
        print(f"  Cooperative shares: {stats['cooperative_shares']}")
        print(f"  Update rate: {v2v.update_rate_hz} Hz")
        print(f"  Communication range: {args.v2v_range} m")
        
        # Print BSM example
        ego_bsm = v2v.get_bsm(ego_vehicle.id)
        if ego_bsm:
            print(f"\nExample BSM (Ego Vehicle):")
            print(f"  Speed: {ego_bsm.speed:.1f} m/s ({ego_bsm.speed*3.6:.1f} km/h)")
            print(f"  Heading: {ego_bsm.heading:.1f}°")
            print(f"  Position: ({ego_bsm.latitude:.2f}, {ego_bsm.longitude:.2f}, {ego_bsm.elevation:.2f})")
            print(f"  Acceleration: Long={ego_bsm.longitudinal_accel:.2f} m/s², Lat={ego_bsm.lateral_accel:.2f} m/s²")
            print(f"  Brake status: {ego_bsm.brake_status}")
            print(f"  Message count: {ego_bsm.msg_count}")
        
        print(f"\nV2V API still available at: http://localhost:{args.api_port}/docs")
        print("Press Ctrl+C to exit and cleanup...")
        
        # Keep API server running until user exits
        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            print("\n\nShutting down...")
    
    except KeyboardInterrupt:
        print("\n\nInterrupted by user")
    
    except Exception as e:
        print(f"\nError: {e}")
        import traceback
        traceback.print_exc()
    
    finally:
        # Cleanup
        print("\nCleaning up...")
        
        if world:
            # Restore original settings
            world.apply_settings(original_settings)
        
        if client and actors:
            print(f"Destroying {len(actors)} actors...")
            client.apply_batch([carla.command.DestroyActor(x) for x in actors])
        
        print("✓ Cleanup complete")


if __name__ == '__main__':
    main()
