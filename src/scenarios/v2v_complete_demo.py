#!/usr/bin/env python3
"""
Complete V2V + LiDAR Demo Scenario

Demonstrates all architecture patterns:
- Context Manager Pattern (CARLASession)
- Builder Pattern (ScenarioBuilder)
- Observer Pattern (ConsoleObserver, CARLADebugObserver, etc.)
- Lazy Evaluation (LazyVehicleStats)
- Configuration Management (centralized dataclasses)

Features:
- Enhanced V2V communication with SAE J2735 BSM protocol
- Real-time LiDAR visualization (web-based 3D viewer)
- Multiple observers for different output formats
- Deterministic and reproducible simulation
- Performance optimizations (binary WebSocket, octree downsampling)
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

from src.v2v import V2VNetworkEnhanced, V2VAPI
from src.utils import (
    CARLASession, VehicleState, ActorManager,
    ScenarioBuilder, ScenarioConfig,
    ConsoleObserver, CARLADebugObserver, CSVDataLogger, CompactLogObserver,
    LiDARQuality, VehicleColor, SemanticTag,
    LazyVehicleStats, Timer, calculate_distance_3d
)
from src.config import DEFAULT_SIM_CONFIG, DEFAULT_V2V_CONFIG
import uvicorn
import threading

# Setup logging
log_dir = Path(__file__).parent.parent.parent / 'logs'
log_dir.mkdir(exist_ok=True)
log_file = log_dir / f"v2v_complete_demo_{datetime.now().strftime('%Y%m%d_%H%M%S')}.log"

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s',
    handlers=[
        logging.FileHandler(log_file),
        logging.StreamHandler()
    ]
)
logger = logging.getLogger(__name__)


def run_complete_v2v_demo(config: ScenarioConfig) -> None:
    """
    Run complete V2V + LiDAR demonstration scenario.
    
    This scenario showcases:
    - V2V communication between all vehicles within range
    - Real-time LiDAR visualization in web browser
    - V2V REST API for programmatic access
    - Multiple observation/logging methods
    - Proper resource management and cleanup
    
    Args:
        config: Scenario configuration from ScenarioBuilder
    """
    lidar_api: Optional[Any] = None
    v2v_api: Optional[V2VAPI] = None
    api_thread: Optional[threading.Thread] = None
    observers: List[Any] = []
    
    try:
        # ============================================================================
        # STEP 1: Initialize CARLA Session (Context Manager Pattern)
        # ============================================================================
        with CARLASession(config.host, config.port, config) as session:
            
            print(f"\n{'='*80}")
            print(f"🚀 COMPLETE V2V + LiDAR DEMONSTRATION")
            print(f"{'='*80}")
            print(f"🔄 Connected to CARLA: {config.host}:{config.port}")
            print(f"🗺️  Map: {session.world.get_map().name}")
            print(f"🎲 Random seed: {config.random_seed}")
            print(f"⏱️  Duration: {config.duration}s")
            print(f"🚗 Vehicles: {config.num_vehicles}")
            print(f"📡 V2V range: {config.v2v_range}m")
            print(f"{'='*80}\n")
            
            # Set deterministic seeds for reproducibility
            random.seed(config.random_seed)
            np.random.seed(config.random_seed)
            
            # ========================================================================
            # STEP 2: Initialize V2V Network (Enhanced BSM Protocol)
            # ========================================================================
            v2v: Optional[V2VNetworkEnhanced] = None
            if config.v2v_enabled:
                print(f"📡 Initializing V2V Network...")
                v2v = V2VNetworkEnhanced(
                    max_range=config.v2v_range,
                    update_rate_hz=2.0,  # SAE J2735 standard
                    enable_cooperative_perception=True,
                    world=session.world
                )
                print(f"   ✓ V2V initialized: {config.v2v_range}m range, 2 Hz update rate")
                
                # Note: V2V REST API available at separate scenario (v2v_api_scenario.py)
                # to avoid threading conflicts with LiDAR API server
                print(f"   💡 Tip: Run 'v2v_api_scenario.py' for REST API access\n")
            
            # ========================================================================
            # STEP 3: Spawn Vehicles
            # ========================================================================
            actor_mgr: ActorManager = ActorManager(session.world, session.bp_lib)
            
            # CRITICAL: Skip first spawn points (parking lots in Town10HD)
            # Use spawn points from index 10+ for better road positions
            road_spawn_points = session.spawn_points[10:]  # Skip first 10 (parking lots)
            random.shuffle(road_spawn_points)
            num_vehicles: int = min(config.num_vehicles, len(road_spawn_points))
            
            print(f"🚗 Using {len(road_spawn_points)} road spawn points (skipped first 10)")
            
            # Spawn ego vehicle with retry logic on road spawn points
            print(f"👑 Spawning ego vehicle...")
            ego: Optional[carla.Actor] = None
            for spawn_attempt in range(min(10, len(road_spawn_points))):
                try:
                    ego = actor_mgr.spawn_ego(
                        blueprint_id=config.ego_blueprint,
                        spawn_point=road_spawn_points[spawn_attempt],
                        color=VehicleColor.RED.value
                    )
                    session.add_actor(ego)
                    if v2v:
                        v2v.register(0, ego)
                    print(f"   ✓ Ego spawned on road at spawn point index {spawn_attempt + 10}")
                    break
                except RuntimeError as e:
                    if spawn_attempt < 9:
                        logger.debug(f"Spawn attempt {spawn_attempt} failed, retrying...")
                        continue
                    else:
                        raise RuntimeError(f"Failed to spawn ego after {spawn_attempt+1} attempts") from e
            
            if ego is None:
                raise RuntimeError("Failed to spawn ego vehicle")
            
            # ========================================================================
            # STEP 4: Initialize LiDAR (if enabled)
            # ========================================================================
            if config.lidar_enabled:
                print(f"\n📡 Initializing Unified Visualization Server...")
                from src.visualization.lidar.api import LiDARStreamingAPI
                
                # Create LiDAR API with V2V network integration
                lidar_api = LiDARStreamingAPI(
                    world=session.world,
                    web_port=config.lidar_web_port,
                    channels=32 if config.lidar_quality == LiDARQuality.FAST.value else 64,
                    points_per_second=500000 if config.lidar_quality == LiDARQuality.FAST.value else 1000000,
                    lidar_range=80.0,
                    downsample_factor=1 if config.lidar_quality == LiDARQuality.HIGH.value else 2,
                    v2v_network=v2v  # Pass V2V network for unified visualization
                )
                lidar_api.register_ego_only(ego)
                lidar_api.start_server(background=True)
                
                # URLs are printed by LiDAR API start_server
            
            # ========================================================================
            # STEP 5: Setup Traffic Manager (Deterministic)
            # ========================================================================
            from src.utils import setup_traffic_manager
            
            print(f"🚦 Setting up Traffic Manager...")
            tm: carla.TrafficManager = setup_traffic_manager(
                session.client,
                config.tm_port,
                config.tm_seed,
                config.use_hybrid_physics,
                config.hybrid_physics_radius
            )
            # CRITICAL: Use realistic speed settings for better movement
            # -30% is too slow and causes vehicles to stop, use -10% instead
            tm.global_percentage_speed_difference(-10.0)  # 10% slower than speed limit
            tm.set_global_distance_to_leading_vehicle(config.safety_distance)
            
            # Enable autopilot on ego with proper settings
            ego.set_autopilot(True, config.tm_port)
            tm.update_vehicle_lights(ego, True)
            tm.ignore_lights_percentage(ego, 70)  # Ignore 70% of lights for better movement
            tm.auto_lane_change(ego, True)
            tm.distance_to_leading_vehicle(ego, 2.0)  # Closer following for ego
            tm.vehicle_percentage_speed_difference(ego, 0.0)  # Ego drives at speed limit
            print(f"   ✓ Traffic Manager configured: speed=-10%, ignores most lights")
            print(f"   ✓ Ego autopilot: speed=100%, lane change enabled\n")
            
            # Spawn traffic vehicles with better error handling
            print(f"🚗 Spawning {num_vehicles-1} traffic vehicles on roads...")
            
            # Use try_spawn_actor for better spawn point handling
            vehicle_bps = [x for x in session.bp_lib.filter('vehicle.*') 
                          if int(x.get_attribute('number_of_wheels')) == 4]
            
            traffic: List[carla.Actor] = []
            spawned_count = 0
            failed_count = 0
            
            for i in range(1, num_vehicles):
                if i >= len(road_spawn_points):
                    break
                try:
                    veh_bp = random.choice(vehicle_bps)
                    # Randomize vehicle color
                    if veh_bp.has_attribute('color'):
                        color = random.choice(veh_bp.get_attribute('color').recommended_values)
                        veh_bp.set_attribute('color', color)
                    
                    veh = session.world.try_spawn_actor(veh_bp, road_spawn_points[i])
                    if veh is not None:
                        session.add_actor(veh)
                        traffic.append(veh)
                        if v2v:
                            v2v.register(i, veh)
                        veh.set_autopilot(True, config.tm_port)
                        tm.update_vehicle_lights(veh, True)
                        tm.ignore_lights_percentage(veh, 70)  # Ignore 70% of lights
                        spawned_count += 1
                    else:
                        failed_count += 1
                except RuntimeError as e:
                    failed_count += 1
                    logger.debug(f"Failed to spawn vehicle at spawn point {i}: {e}")
                    continue
            
            print(f"   ✓ Spawned {spawned_count} traffic vehicles ({failed_count} spawn failures)")
            print(f"   ✓ Total vehicles in simulation: {len(traffic) + 1}\n")
            
            # ========================================================================
            # STEP 6: Setup Observers (Observer Pattern)
            # ========================================================================
            print(f"👁️  Setting up observers...")
            
            if config.console_output:
                observers.append(ConsoleObserver(
                    interval_seconds=config.console_interval_seconds,
                    fps=config.fps
                ))
                print(f"   ✓ Console observer (every {config.console_interval_seconds}s)")
            
            if config.carla_debug_viz and v2v:
                observers.append(CARLADebugObserver(
                    world=session.world,
                    v2v_network=v2v,
                    ego_id=0,
                    update_interval_frames=config.debug_viz_interval_frames
                ))
                print(f"   ✓ CARLA debug visualization")
            
            if config.csv_logging:
                csv_path: Optional[Path] = Path(config.csv_output_path) if config.csv_output_path else None
                if csv_path is None:
                    csv_path = log_dir / f"scenario_data_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
                observers.append(CSVDataLogger(output_path=csv_path))
                print(f"   ✓ CSV data logger: {csv_path}")
            
            if config.compact_logging:
                observers.append(CompactLogObserver(logger))
                print(f"   ✓ Compact log observer")
            
            print(f"   Total: {len(observers)} observers registered\n")
            
            # ========================================================================
            # STEP 7: Warmup Period
            # ========================================================================
            print(f"⏱️  Warming up simulation ({config.warmup_frames} frames)...")
            print(f"   Initializing Traffic Manager routes...")
            for i in range(config.warmup_frames):
                session.world.tick()
                # Log ego speed during warmup to verify movement
                if i % 20 == 0 and i > 0:
                    snapshot = session.world.get_snapshot()
                    ego_snap = snapshot.find(ego.id)
                    if ego_snap:
                        vel = ego_snap.get_velocity()
                        speed_kmh = 3.6 * (vel.x**2 + vel.y**2 + vel.z**2)**0.5
                        logger.debug(f"Warmup frame {i}: ego speed={speed_kmh:.1f} km/h")
            print(f"   ✓ Warmup complete - vehicles should be moving\n")
            
            # ========================================================================
            # STEP 8: Main Simulation Loop
            # ========================================================================
            print(f"{'='*80}")
            print(f"🚀 STARTING MAIN SIMULATION LOOP")
            print(f"{'='*80}\n")
            
            start_time: float = time.time()
            frame: int = 0
            frame_times: List[float] = []
            
            try:
                while time.time() - start_time < config.duration:
                    frame_start: float = time.perf_counter()
                    frame += 1
                    
                    # CRITICAL: Tick world first, then get fresh snapshot immediately
                    session.world.tick()
                    snapshot: carla.WorldSnapshot = session.world.get_snapshot()
                    
                    # Get ego state from snapshot IMMEDIATELY after tick for fresh data
                    ego_snapshot: Optional[carla.ActorSnapshot] = snapshot.find(ego.id)
                    if not ego_snapshot:
                        continue
                    
                    # Update V2V network with fresh snapshot at 2 Hz
                    if v2v and frame % config.v2v_update_interval_frames == 0:
                        v2v.update(force=True, snapshot=snapshot)
                    
                    # Create vehicle state object (uses lazy evaluation internally)
                    state: VehicleState = VehicleState.from_snapshot(
                        frame=frame,
                        actor_snapshot=ego_snapshot,
                        control=ego.get_control()
                    )
                    
                    # Prepare V2V data for observers
                    v2v_data: Dict[str, Any] = {
                        'neighbors': v2v.get_neighbors(0) if v2v else [],
                        'threats': v2v.get_threats(0) if v2v else [],
                        'bsm': v2v.get_bsm(0) if v2v else None,
                        'total_vehicles': actor_mgr.count(),
                        'lidar_points': lidar_api.get_point_count() if lidar_api else 0
                    }
                    
                    # Notify all observers (they use lazy evaluation internally)
                    for observer in observers:
                        observer.on_frame(frame, state, v2v_data)
                    
                    # Track frame time for performance analysis
                    frame_time: float = time.perf_counter() - frame_start
                    frame_times.append(frame_time)
                    
            except KeyboardInterrupt:
                print(f"\n⚠️  Simulation interrupted by user")
                logger.warning("Simulation interrupted by user")
            
            # ========================================================================
            # STEP 9: Final Statistics
            # ========================================================================
            elapsed_time: float = time.time() - start_time
            
            print(f"\n{'='*80}")
            print(f"📊 SIMULATION STATISTICS")
            print(f"{'='*80}")
            
            # Performance statistics
            if frame_times:
                avg_frame_time: float = np.mean(frame_times)
                max_frame_time: float = np.max(frame_times)
                min_frame_time: float = np.min(frame_times)
                std_frame_time: float = np.std(frame_times)
                
                print(f"\n⏱️  Performance:")
                print(f"   Total frames:        {frame}")
                print(f"   Real time:           {elapsed_time:.2f}s")
                print(f"   Simulated time:      {frame/config.fps:.2f}s")
                print(f"   Real-time factor:    {(frame/config.fps)/elapsed_time:.2f}x")
                print(f"   Avg frame time:      {avg_frame_time*1000:.2f}ms")
                print(f"   Min frame time:      {min_frame_time*1000:.2f}ms")
                print(f"   Max frame time:      {max_frame_time*1000:.2f}ms")
                print(f"   Std frame time:      {std_frame_time*1000:.2f}ms")
            
            # V2V statistics
            if v2v:
                stats = v2v.get_network_stats()
                print(f"\n📡 V2V Network:")
                print(f"   Update rate:         {v2v.update_rate_hz} Hz")
                print(f"   Communication range: {v2v.max_range} m")
                print(f"   Total BSM sent:      {stats['total_messages_sent']}")
                print(f"   Avg neighbors:       {stats['average_neighbors']:.1f}")
                print(f"   Max neighbors:       {stats['max_neighbors']}")
                print(f"   Cooperative shares:  {stats['cooperative_shares']}")
                
                # Show final ego BSM
                ego_bsm = v2v.get_bsm(0)
                if ego_bsm:
                    print(f"\n   Final ego BSM:")
                    print(f"     Vehicle ID:    {ego_bsm.vehicle_id}")
                    print(f"     Speed:         {ego_bsm.speed:.1f} m/s ({ego_bsm.speed*3.6:.1f} km/h)")
                    print(f"     Heading:       {ego_bsm.heading:.1f}°")
                    print(f"     Message count: {ego_bsm.msg_count}")
                    
                    # Show neighbors at end
                    final_neighbors = v2v.get_neighbors(0)
                    if final_neighbors:
                        print(f"\n   Final neighbors in range:")
                        for n in final_neighbors:
                            dist = calculate_distance_3d(
                                (n.latitude, n.longitude, n.elevation),
                                (ego_bsm.latitude, ego_bsm.longitude, ego_bsm.elevation)
                            )
                            print(f"     ID {n.vehicle_id}: {n.speed*3.6:.1f} km/h at {dist:.1f}m")
            
            # LiDAR statistics
            if lidar_api:
                print(f"\n🎯 LiDAR:")
                print(f"   Total points streamed: {lidar_api.get_point_count() * frame:,}")
                print(f"   Web server port:       {config.lidar_web_port}")
            
            # Notify observers of completion
            for observer in observers:
                observer.on_complete(frame, elapsed_time)
            
            print(f"\n{'='*80}")
            print(f"✅ SIMULATION COMPLETED SUCCESSFULLY")
            print(f"{'='*80}\n")
            
            logger.info(f"Scenario completed: {frame} frames in {elapsed_time:.2f}s")
    
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


def main():
    """Parse arguments and run scenario."""
    parser = argparse.ArgumentParser(
        description='Complete V2V + LiDAR Demonstration Scenario',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
This scenario demonstrates all best practices:
  • Context Manager Pattern (automatic cleanup)
  • Builder Pattern (fluent configuration)
  • Observer Pattern (multiple output formats)
  • Lazy Evaluation (performance optimization)
  • Enhanced V2V with SAE J2735 BSM protocol
  • Real-time LiDAR visualization

Examples:
  # Basic demo with default settings
  python %(prog)s
  
  # Custom settings
  python %(prog)s --vehicles 15 --duration 90 --v2v-range 100
  
  # High performance with CSV logging
  python %(prog)s --vehicles 20 --lidar-quality high --csv-logging
  
  # Minimal (no visualization, just logging)
  python %(prog)s --no-console --no-debug-viz --csv-logging
        """
    )
    
    # CARLA Connection
    parser.add_argument('--host', default='192.168.1.110', 
                       help='CARLA server IP (default: 192.168.1.110)')
    parser.add_argument('--port', type=int, default=2000, 
                       help='CARLA server port (default: 2000)')
    
    # Simulation
    parser.add_argument('--duration', type=int, default=60, 
                       help='Scenario duration in seconds (default: 60)')
    parser.add_argument('--vehicles', type=int, default=10, 
                       help='Number of vehicles (default: 10)')
    parser.add_argument('--seed', type=int, default=42, 
                       help='Random seed for reproducibility (default: 42)')
    
    # V2V
    parser.add_argument('--v2v-range', type=float, default=50.0, 
                       help='V2V communication range in meters (default: 50.0)')
    parser.add_argument('--no-v2v', action='store_true', 
                       help='Disable V2V communication')
    
    # LiDAR
    parser.add_argument('--lidar', action='store_true', default=True,
                       help='Enable ego LiDAR visualization (default: True)')
    parser.add_argument('--no-lidar', action='store_false', dest='lidar',
                       help='Disable LiDAR visualization')
    parser.add_argument('--web-port', type=int, default=8000, 
                       help='Web server port for LiDAR viewer (default: 8000)')
    parser.add_argument('--lidar-quality', choices=['high', 'medium', 'fast'], 
                       default='medium', help='LiDAR quality preset (default: medium)')
    
    # Visualization & Logging
    parser.add_argument('--no-console', action='store_false', dest='console', 
                       help='Disable console output')
    parser.add_argument('--no-debug-viz', action='store_false', dest='debug_viz', 
                       help='Disable CARLA debug visualization')
    parser.add_argument('--csv-logging', action='store_true', 
                       help='Enable CSV data logging')
    parser.add_argument('--csv-output', 
                       help='CSV output file path (default: auto-generated in logs/)')
    
    args = parser.parse_args()
    
    # ============================================================================
    # Build Configuration using Builder Pattern
    # ============================================================================
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
    if args.lidar:
        config.lidar_enabled = True
        config.lidar_quality = args.lidar_quality
        config.lidar_web_port = args.web_port
    else:
        config.lidar_enabled = False
    
    # Apply logging settings
    if args.csv_logging:
        config.csv_logging = True
        if args.csv_output:
            config.csv_output_path = args.csv_output
    
    # Run the complete demonstration
    run_complete_v2v_demo(config)


if __name__ == '__main__':
    main()
