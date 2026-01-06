#!/usr/bin/env python3
"""
V2V Scenario - Advanced Version with Observer and Builder Patterns
Demonstrates Phase 2 refactoring: clean separation of concerns.
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
from src.utils import (
    CARLASession, VehicleState, ActorManager,
    ScenarioBuilder, ScenarioConfig,
    ConsoleObserver, CARLADebugObserver, CSVDataLogger, CompactLogObserver
)
from src.visualization.lidar import create_ego_lidar_stream

# Setup logging
log_dir = Path(__file__).parent.parent.parent / 'logs'
log_dir.mkdir(exist_ok=True)
log_file = log_dir / f"v2v_advanced_{datetime.now().strftime('%Y%m%d_%H%M%S')}.log"

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s',
    handlers=[
        logging.FileHandler(log_file),
        logging.StreamHandler()
    ]
)
logger = logging.getLogger(__name__)


def run_v2v_scenario_advanced(config: ScenarioConfig):
    """
    Run V2V scenario with advanced architecture.
    Uses Observer pattern for visualization and Builder for configuration.
    
    Args:
        config: Scenario configuration from ScenarioBuilder
    """
    lidar_api = None
    observers = []
    
    try:
        # Use context manager for automatic cleanup
        with CARLASession(config.host, config.port, config) as session:
            
            print(f"🔄 Connecting to {config.host}:{config.port}")
            print(f"✓ Connected to {session.world.get_map().name}\n")
            
            # Set deterministic seeds
            random.seed(config.random_seed)
            np.random.seed(config.random_seed)
            
            # Initialize systems
            v2v = V2VNetwork(max_range=config.v2v_range) if config.v2v_enabled else None
            actor_mgr = ActorManager(session.world, session.bp_lib)
            
            # Limit vehicles to available spawn points
            num_vehicles = min(config.num_vehicles, len(session.spawn_points))
            random.shuffle(session.spawn_points)
            
            # Spawn ego vehicle
            ego = actor_mgr.spawn_ego(
                blueprint_id=config.ego_blueprint,
                spawn_point=session.spawn_points[0],
                color=config.ego_color
            )
            session.add_actor(ego)
            if v2v:
                v2v.register(0, ego)
            print(f"👑 Ego vehicle spawned (RED) at {session.spawn_points[0].location}")
            
            # Initialize ego LiDAR if enabled
            if config.lidar_enabled:
                print(f"\n📡 Initializing ego LiDAR streaming...")
                lidar_api = create_ego_lidar_stream(
                    world=session.world,
                    ego_vehicle=ego,
                    web_port=config.lidar_web_port,
                    high_quality=(config.lidar_quality == 'high')
                )
                logger.info(f"Ego LiDAR streaming initialized on port {config.lidar_web_port} ({config.lidar_quality} quality)")
                print(f"✓ LiDAR viewer ready\n")
            
            # Setup Traffic Manager
            from src.utils import setup_traffic_manager
            tm = setup_traffic_manager(
                session.client,
                config.tm_port,
                config.tm_seed,
                config.use_hybrid_physics,
                config.hybrid_physics_radius
            )
            tm.global_percentage_speed_difference(config.global_speed_difference)
            tm.set_global_distance_to_leading_vehicle(config.safety_distance)
            
            # Enable autopilot on ego
            ego.set_autopilot(True, config.tm_port)
            tm.update_vehicle_lights(ego, True)
            tm.ignore_lights_percentage(ego, 0)
            tm.auto_lane_change(ego, True)
            tm.vehicle_percentage_speed_difference(ego, config.ego_speed_difference)
            
            # Spawn traffic vehicles
            print(f"🚗 Spawning {num_vehicles-1} traffic vehicles...")
            traffic = actor_mgr.spawn_traffic(
                num_vehicles - 1,
                session.spawn_points[1:]
            )
            
            # Register traffic with session and V2V
            for i, vehicle in enumerate(traffic):
                session.add_actor(vehicle)
                if v2v:
                    v2v.register(i + 1, vehicle)
                vehicle.set_autopilot(True, config.tm_port)
                tm.update_vehicle_lights(vehicle, True)
            
            print(f"✓ Spawned {len(traffic)} traffic vehicles")
            print(f"✓ Total vehicles: {actor_mgr.count()}\n")
            
            # ===== OBSERVER PATTERN: Setup visualization observers =====
            if config.console_output:
                observers.append(ConsoleObserver(
                    interval_seconds=config.console_interval_seconds,
                    fps=config.fps
                ))
            
            if config.carla_debug_viz and v2v:
                observers.append(CARLADebugObserver(
                    world=session.world,
                    v2v_network=v2v,
                    ego_id=0,
                    update_interval_frames=config.debug_viz_interval_frames
                ))
            
            if config.csv_logging:
                csv_path = Path(config.csv_output_path) if config.csv_output_path else None
                observers.append(CSVDataLogger(output_path=csv_path))
            
            if config.compact_logging:
                observers.append(CompactLogObserver(logger))
            
            print(f"👁️  Registered {len(observers)} observers\n")
            
            # Warmup
            print("⏱️  Warming up simulation...")
            for _ in range(config.warmup_frames):
                session.world.tick()
            logger.info("Warmup complete")
            
            # Main simulation loop
            print(f"🚀 Running V2V scenario for {config.duration}s...\n")
            start_time = time.time()
            frame = 0
            
            while time.time() - start_time < config.duration:
                frame += 1
                session.world.tick()
                snapshot = session.world.get_snapshot()
                
                # Update V2V network
                if v2v and frame % config.v2v_update_interval_frames == 0:
                    v2v.update(force=True, snapshot=snapshot)
                
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
                
                # Prepare V2V data
                v2v_data = {
                    'neighbors': v2v.get_neighbors(0) if v2v else [],
                    'total_vehicles': actor_mgr.count(),
                    'lidar_points': lidar_api.get_point_count() if lidar_api else 0
                }
                
                # ===== OBSERVER PATTERN: Notify all observers =====
                for observer in observers:
                    observer.on_frame(frame, state, v2v_data)
            
            # Scenario completed
            elapsed_time = time.time() - start_time
            
            # ===== OBSERVER PATTERN: Notify completion =====
            for observer in observers:
                observer.on_complete(frame, elapsed_time)
            
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
    parser = argparse.ArgumentParser(
        description='V2V Scenario (Advanced - Observer + Builder Pattern)',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Quick test with defaults
  python %(prog)s
  
  # Full V2V + LiDAR scenario
  python %(prog)s --vehicles 20 --duration 120 --enable-lidar
  
  # Performance test (many vehicles, no LiDAR)
  python %(prog)s --vehicles 50 --duration 300 --no-lidar --no-console
  
  # Data collection with CSV logging
  python %(prog)s --vehicles 15 --csv-logging --duration 180
        """
    )
    
    # CARLA Connection
    parser.add_argument('--host', default='192.168.1.110', help='CARLA server IP')
    parser.add_argument('--port', type=int, default=2000, help='CARLA server port')
    
    # Simulation
    parser.add_argument('--duration', type=int, default=60, help='Scenario duration (seconds)')
    parser.add_argument('--vehicles', type=int, default=15, help='Number of vehicles')
    parser.add_argument('--seed', type=int, default=42, help='Random seed')
    
    # V2V
    parser.add_argument('--v2v-range', type=float, default=50.0, help='V2V range (meters)')
    parser.add_argument('--no-v2v', action='store_true', help='Disable V2V communication')
    
    # LiDAR
    parser.add_argument('--enable-lidar', action='store_true', default=False, help='Enable ego LiDAR')
    parser.add_argument('--no-lidar', action='store_false', dest='enable_lidar', help='Disable LiDAR')
    parser.add_argument('--web-port', type=int, default=8000, help='Web server port for LiDAR')
    parser.add_argument('--lidar-quality', choices=['high', 'fast'], default='high', help='LiDAR quality')
    
    # Visualization
    parser.add_argument('--no-console', action='store_false', dest='console', help='Disable console output')
    parser.add_argument('--no-debug-viz', action='store_false', dest='debug_viz', help='Disable CARLA debug viz')
    
    # Logging
    parser.add_argument('--csv-logging', action='store_true', help='Enable CSV data logging')
    parser.add_argument('--csv-output', help='CSV output path')
    
    args = parser.parse_args()
    
    # ===== BUILDER PATTERN: Clean configuration =====
    config = (ScenarioBuilder()
        .with_carla_server(args.host, args.port)
        .with_duration(args.duration)
        .with_vehicles(args.vehicles)
        .with_seed(args.seed)
        .with_v2v(enabled=not args.no_v2v, range_m=args.v2v_range)
        .with_console_output(enabled=args.console)
        .with_carla_debug(enabled=args.debug_viz)
        .build()
    )
    
    # Apply LiDAR settings
    if args.enable_lidar:
        config.lidar_enabled = True
        config.lidar_quality = args.lidar_quality
        config.lidar_web_port = args.web_port
    
    # Apply logging settings
    if args.csv_logging:
        config.csv_logging = True
        if args.csv_output:
            config.csv_output_path = args.csv_output
    
    # Run scenario
    run_v2v_scenario_advanced(config)
