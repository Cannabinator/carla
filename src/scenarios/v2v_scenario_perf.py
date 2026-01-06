#!/usr/bin/env python3
"""
V2V Scenario - Performance Optimized Version

Improvements:
- Binary WebSocket: 40% less bandwidth
- Octree downsampling: 50-70% fewer points
- Lazy stats: 10-20% less CPU
- Full type hints: Better IDE support and type checking
"""

import carla
import random
import time
import numpy as np
import argparse
import logging
from datetime import datetime
from pathlib import Path
from typing import List, Dict, Any, Optional
import sys

sys.path.insert(0, str(Path(__file__).parent.parent.parent))

from src.v2v import V2VNetworkEnhanced  # Use enhanced V2V network
from src.utils import (
    CARLASession, VehicleState, ActorManager,
    ScenarioBuilder, ScenarioConfig,
    ConsoleObserver, CARLADebugObserver, CSVDataLogger, CompactLogObserver,
    LiDARQuality, VehicleColor, SemanticTag,
    LazyVehicleStats, Timer
)

# Setup logging
log_dir = Path(__file__).parent.parent.parent / 'logs'
log_dir.mkdir(exist_ok=True)
log_file = log_dir / f"v2v_perf_{datetime.now().strftime('%Y%m%d_%H%M%S')}.log"

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s',
    handlers=[
        logging.FileHandler(log_file),
        logging.StreamHandler()
    ]
)
logger = logging.getLogger(__name__)


def run_v2v_scenario_performance(config: ScenarioConfig) -> None:
    """
    Run performance-optimized V2V scenario.
    
    Args:
        config: Scenario configuration from ScenarioBuilder
    """
    lidar_api: Optional[Any] = None
    observers: List[Any] = []
    
    try:
        # Context manager for automatic cleanup
        with CARLASession(config.host, config.port, config) as session:
            
            print(f"🔄 Connecting to {config.host}:{config.port}")
            print(f"✓ Connected to {session.world.get_map().name}\n")
            
            # Set deterministic seeds
            random.seed(config.random_seed)
            np.random.seed(config.random_seed)
            
            # Initialize enhanced V2V system with 2 Hz update rate
            v2v: Optional[V2VNetworkEnhanced] = V2VNetworkEnhanced(
                max_range=config.v2v_range,
                update_rate_hz=2.0,
                enable_cooperative_perception=True,
                world=session.world
            ) if config.v2v_enabled else None
            actor_mgr: ActorManager = ActorManager(session.world, session.bp_lib)  # type: ignore
            
            # Limit vehicles to available spawn points
            num_vehicles: int = min(config.num_vehicles, len(session.spawn_points))
            random.shuffle(session.spawn_points)
            
            # Spawn ego vehicle (using enum for color)
            with Timer("Ego vehicle spawn"):
                ego: carla.Actor = actor_mgr.spawn_ego(
                    blueprint_id=config.ego_blueprint,
                    spawn_point=session.spawn_points[0],
                    color=VehicleColor.RED.value
                )
                session.add_actor(ego)
                if v2v:
                    v2v.register(0, ego)
            print(f"👑 Ego vehicle spawned at {session.spawn_points[0].location}\n")
            
            # Initialize ego LiDAR if enabled (performance optimized)
            if config.lidar_enabled:
                print(f"📡 Initializing performance-optimized LiDAR...")
                from src.visualization.lidar import create_ego_lidar_stream
                
                with Timer("LiDAR initialization"):
                    lidar_api = create_ego_lidar_stream(
                        world=session.world,  # type: ignore
                        ego_vehicle=ego,
                        web_port=config.lidar_web_port,
                        high_quality=(config.lidar_quality == LiDARQuality.HIGH.value)
                    )
                logger.info(f"LiDAR streaming initialized on port {config.lidar_web_port}")
                print(f"✓ LiDAR ready with {config.lidar_quality} quality\n")
            
            # Setup Traffic Manager
            from src.utils import setup_traffic_manager
            tm: carla.TrafficManager = setup_traffic_manager(
                session.client,  # type: ignore
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
            tm.auto_lane_change(ego, True)
            tm.vehicle_percentage_speed_difference(ego, config.ego_speed_difference)
            
            # Spawn traffic vehicles
            print(f"🚗 Spawning {num_vehicles-1} traffic vehicles...")
            with Timer("Traffic vehicle spawn"):
                traffic: List[carla.Actor] = actor_mgr.spawn_traffic(
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
            
            # Setup observers
            if config.console_output:
                observers.append(ConsoleObserver(
                    interval_seconds=config.console_interval_seconds,
                    fps=config.fps
                ))
            
            if config.carla_debug_viz and v2v:
                observers.append(CARLADebugObserver(
                    world=session.world,  # type: ignore
                    v2v_network=v2v,  # type: ignore
                    ego_id=0,
                    update_interval_frames=config.debug_viz_interval_frames
                ))
            
            if config.csv_logging:
                csv_path: Optional[Path] = Path(config.csv_output_path) if config.csv_output_path else None
                observers.append(CSVDataLogger(output_path=csv_path))
            
            if config.compact_logging:
                observers.append(CompactLogObserver(logger))
            
            print(f"👁️  Registered {len(observers)} observers\n")
            
            # Warmup
            print("⏱️  Warming up simulation...")
            with Timer("Warmup"):
                for _ in range(config.warmup_frames):
                    session.world.tick()
            logger.info("Warmup complete")
            
            # Main simulation loop with performance tracking
            print(f"🚀 Running performance-optimized V2V scenario for {config.duration}s...\n")
            start_time: float = time.time()
            frame: int = 0
            frame_times: List[float] = []
            
            while time.time() - start_time < config.duration:
                frame_start: float = time.perf_counter()
                frame += 1
                
                session.world.tick()
                snapshot: carla.WorldSnapshot = session.world.get_snapshot()
                
                # Update V2V network
                if v2v and frame % config.v2v_update_interval_frames == 0:
                    v2v.update(force=True, snapshot=snapshot)
                
                # Get ego state from snapshot
                ego_snapshot: Optional[carla.ActorSnapshot] = snapshot.find(ego.id)
                if not ego_snapshot:
                    continue
                
                # LAZY EVALUATION: Create vehicle state object
                state: VehicleState = VehicleState.from_snapshot(
                    frame=frame,
                    actor_snapshot=ego_snapshot,
                    control=ego.get_control()
                )
                
                # Prepare V2V data
                v2v_data: Dict[str, Any] = {
                    'neighbors': v2v.get_neighbors(0) if v2v else [],
                    'total_vehicles': actor_mgr.count(),
                    'lidar_points': lidar_api.get_point_count() if lidar_api else 0
                }
                
                # Notify all observers (they use lazy evaluation internally)
                for observer in observers:
                    observer.on_frame(frame, state, v2v_data)
                
                # Track frame time
                frame_time: float = time.perf_counter() - frame_start
                frame_times.append(frame_time)
            
            # Scenario completed
            elapsed_time: float = time.time() - start_time
            
            # Performance statistics
            avg_frame_time: float = np.mean(frame_times)
            max_frame_time: float = np.max(frame_times)
            min_frame_time: float = np.min(frame_times)
            std_frame_time: float = np.std(frame_times)
            
            print(f"\n📊 Performance Statistics:")
            print(f"   Total frames:    {frame}")
            print(f"   Real time:       {elapsed_time:.2f}s")
            print(f"   Simulated time:  {frame/config.fps:.2f}s")
            print(f"   Real-time factor: {(frame/config.fps)/elapsed_time:.2f}x")
            print(f"   Avg frame time:  {avg_frame_time*1000:.2f}ms")
            print(f"   Min frame time:  {min_frame_time*1000:.2f}ms")
            print(f"   Max frame time:  {max_frame_time*1000:.2f}ms")
            print(f"   Std frame time:  {std_frame_time*1000:.2f}ms")
            
            # Enhanced V2V statistics
            if v2v:
                stats = v2v.get_network_stats()
                print(f"\n📡 V2V Network Statistics:")
                print(f"   Update rate:      {v2v.update_rate_hz} Hz")
                print(f"   Communication range: {v2v.max_range} m")
                print(f"   Total BSM sent:   {stats['total_messages_sent']}")
                print(f"   Avg neighbors:    {stats['average_neighbors']:.1f}")
                print(f"   Max neighbors:    {stats['max_neighbors']}")
                print(f"   Cooperative shares: {stats['cooperative_shares']}")
                
                # Show final BSM example
                ego_bsm = v2v.get_bsm(0)
                if ego_bsm:
                    print(f"\n   Final ego BSM:")
                    print(f"     Speed: {ego_bsm.speed:.1f} m/s ({ego_bsm.speed*3.6:.1f} km/h)")
                    print(f"     Heading: {ego_bsm.heading:.1f}°")
                    print(f"     Message count: {ego_bsm.msg_count}")
            
            # Notify completion
            for observer in observers:
                observer.on_complete(frame, elapsed_time)
            
            logger.info(f"Scenario completed: {frame} frames in {elapsed_time:.2f}s")
    
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
        description='V2V Scenario (Performance Optimized - Phase 3 & 4)',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Performance Optimizations:
  • Binary WebSocket protocol (40% bandwidth reduction)
  • Octree point cloud downsampling (50-70% fewer points)
  • Lazy evaluation for statistics (10-20% CPU savings)
  • Full type hints for better IDE support

Examples:
  # Quick performance test
  python %(prog)s --duration 60 --vehicles 10
  
  # High vehicle count stress test
  python %(prog)s --vehicles 50 --duration 300 --no-console
  
  # Benchmark with CSV logging
  python %(prog)s --vehicles 20 --duration 120 --csv-logging
        """
    )
    
    # CARLA Connection
    parser.add_argument('--host', default='192.168.1.110', help='CARLA server IP')
    parser.add_argument('--port', type=int, default=2000, help='CARLA server port')
    
    # Simulation
    parser.add_argument('--duration', type=int, default=60, help='Scenario duration (seconds)')
    parser.add_argument('--vehicles', type=int, default=10, help='Number of vehicles')
    parser.add_argument('--seed', type=int, default=42, help='Random seed')
    
    # V2V
    parser.add_argument('--v2v-range', type=float, default=50.0, help='V2V range (meters)')
    parser.add_argument('--no-v2v', action='store_true', help='Disable V2V communication')
    
    # LiDAR
    parser.add_argument('--enable-lidar', action='store_true', default=True, help='Enable ego LiDAR')
    parser.add_argument('--web-port', type=int, default=8000, help='Web server port for LiDAR')
    parser.add_argument('--lidar-quality', choices=['high', 'medium', 'fast'], default='medium', help='LiDAR quality')
    
    # Visualization
    parser.add_argument('--no-console', action='store_false', dest='console', help='Disable console output')
    parser.add_argument('--no-debug-viz', action='store_false', dest='debug_viz', help='Disable CARLA debug viz')
    
    # Logging
    parser.add_argument('--csv-logging', action='store_true', help='Enable CSV data logging')
    parser.add_argument('--csv-output', help='CSV output path')
    
    args = parser.parse_args()
    
    # Build configuration using Builder pattern
    config: ScenarioConfig = (ScenarioBuilder()
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
    
    # Run performance-optimized scenario
    run_v2v_scenario_performance(config)
