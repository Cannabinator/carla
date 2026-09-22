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
from src.v2x import V2XNetworkEnhanced, RoadSideUnit, VRUProfile
from src.utils import (
    CARLASession, VehicleState, ActorManager,
    ScenarioBuilder, ScenarioConfig,
    ConsoleObserver, CARLADebugObserver, CSVDataLogger, CompactLogObserver,
    SpectatorFollowObserver, V2VMessageLogger,
    RunDirectory, PotholeEventLogger,
    LiDARQuality, VehicleColor, SemanticTag,
    LazyVehicleStats, Timer, calculate_distance_3d
)
from src.config import DEFAULT_SIM_CONFIG, DEFAULT_V2V_CONFIG
import uvicorn
import threading

try:
    from src.agents.navigation.behavior_agent import BehaviorAgent
    BEHAVIOR_AGENT_AVAILABLE = True
except ImportError:
    BehaviorAgent = None  # type: ignore[assignment,misc]
    BEHAVIOR_AGENT_AVAILABLE = False

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


def run_complete_v2v_demo(config: ScenarioConfig, status_callback=None, server_module=None) -> None:
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
        status_callback: Optional callback function(frame, elapsed, v2v_msgs) for status updates
        server_module: Server module reference to avoid thread isolation issues
    """
    lidar_api: Optional[Any] = None
    v2v_api: Optional[V2VAPI] = None
    api_thread: Optional[threading.Thread] = None
    observers: List[Any] = []
    walker_controllers: List[Any] = []
    
    try:
        # ============================================================================
        # STEP 1: Initialize CARLA Session (Context Manager Pattern)
        # ============================================================================
        with CARLASession(config.host, config.port, config) as session:

            # Create timestamped run directory — all output files go here
            run_dir = RunDirectory()

            print(f"\n{'='*80}")
            print(f"🚀 COMPLETE V2V + LiDAR DEMONSTRATION")
            print(f"{'='*80}")
            print(f"🔄 Connected to CARLA: {config.host}:{config.port}")
            print(f"🗺️  Map: {session.world.get_map().name}")
            print(f"🎲 Random seed: {config.random_seed}")
            print(f"⏱️  Duration: {config.duration}s")
            print(f"🚗 Vehicles: {config.num_vehicles}")
            print(f"📡 V2V range: {config.v2v_range}m")
            print(f"🤖 Agent mode: {'BehaviorAgent' if BEHAVIOR_AGENT_AVAILABLE else 'TM Autopilot (BehaviorAgent unavailable)'}")
            print(f"📂 Run directory: {run_dir.path}")
            print(f"{'='*80}\n")
            
            # Set deterministic seeds for reproducibility
            random.seed(config.random_seed)
            np.random.seed(config.random_seed)

            # ========================================================================
            # POTHOLE MANAGER INITIALIZATION (for BSM communication)
            # ========================================================================
            # NOTE: Potholes are no longer generated automatically.
            # They should be pre-placed in your custom CARLA map.
            # Use pothole_mgr.add_pothole() to register them manually if needed.
            from src.utils.potholes import PotholeManager
            pothole_mgr = PotholeManager()
            pothole_mgr.initialize(session.world)
            print(f"🕳️  Pothole Manager: Ready for BSM pothole communication "
                  f"({len(pothole_mgr)} potholes registered)")

            # ========================================================================
            # STEP 2: Initialize V2V Network (Enhanced BSM Protocol)
            # ========================================================================
            v2v: Optional[V2VNetworkEnhanced] = None
            if config.v2v_enabled:
                print(f"📡 Initializing V2V Network...")

                # Build DSRCConfig from scenario config overrides (if any)
                from src.v2v.dsrc_channel import DSRCConfig
                dsrc_cfg = DSRCConfig(
                    channel_busy_ratio=getattr(config, 'dsrc_channel_busy_ratio', 0.15),
                    enable_nlos_model=getattr(config, 'dsrc_enable_nlos_model', True),
                )
                if getattr(config, 'dsrc_tx_power_dbm', None) is not None:
                    dsrc_cfg.tx_power_dbm = config.dsrc_tx_power_dbm

                _use_v2x = (
                    getattr(config, 'num_pedestrians', 0) > 0
                    or getattr(config, 'num_cyclists', 0) > 0
                    or getattr(config, 'v2x_rsu_enabled', False)
                )
                if _use_v2x:
                    v2v = V2XNetworkEnhanced(
                        max_range=config.v2v_range,
                        update_rate_hz=2.0,
                        enable_cooperative_perception=True,
                        world=session.world,
                        dsrc_config=dsrc_cfg,
                    )
                    print(f"   ✓ V2X network (V2V+V2I+V2P) initialized: {config.v2v_range}m range, 2 Hz, "
                          f"DSRC/WAVE channel (TX={dsrc_cfg.tx_power_dbm:.0f} dBm, "
                          f"CBR={dsrc_cfg.channel_busy_ratio:.2f})")
                else:
                    v2v = V2VNetworkEnhanced(
                        max_range=config.v2v_range,
                        update_rate_hz=2.0,  # SAE J2735 standard beacon interval
                        enable_cooperative_perception=True,
                        world=session.world,
                        dsrc_config=dsrc_cfg,
                    )
                    print(f"   ✓ V2V initialized: {config.v2v_range}m range, 2 Hz, "
                          f"DSRC/WAVE channel (TX={dsrc_cfg.tx_power_dbm:.0f} dBm, "
                          f"CBR={dsrc_cfg.channel_busy_ratio:.2f})")

                # Note: V2V REST API available at separate scenario (v2v_api_scenario.py)
                # to avoid threading conflicts with LiDAR API server
                print(f"   💡 Tip: Run 'v2v_api_scenario.py' for REST API access\n")
            
            # ========================================================================
            # STEP 3: Spawn Vehicles
            # ========================================================================
            actor_mgr: ActorManager = ActorManager(session.world, session.bp_lib)  # type: ignore
            
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
                print(f"\n📡 Initializing LiDAR for existing server...")
                from src.lidar.api import LiDARStreamingAPI
                
                # Create LiDAR API but DON'T start a new server (main server already running)
                lidar_api = LiDARStreamingAPI(
                    world=session.world,
                    web_port=config.lidar_web_port,
                    channels=32 if config.lidar_quality == LiDARQuality.FAST.value else 64,
                    points_per_second=500000 if config.lidar_quality == LiDARQuality.FAST.value else 2000000,
                    lidar_range=100.0,
                    downsample_factor=1 if config.lidar_quality == LiDARQuality.HIGH.value else 2,
                    v2v_network=v2v  # Pass V2V network for unified visualization
                )
                lidar_api.register_ego_only(ego)
                
                # Register with existing server - use passed module reference or import
                if server_module is not None:
                    # Use passed server module (from API call)
                    server_module.set_collector(lidar_api.collector)
                    server_module.set_v2v_network(v2v)
                    print(f"   ✓ LiDAR registered with main server on port {config.lidar_web_port} (via module ref)")
                else:
                    # Direct import (standalone execution)
                    from src.lidar import server as lidar_server
                    lidar_server.set_collector(lidar_api.collector)
                    lidar_server.set_v2v_network(v2v)
                    print(f"   ✓ LiDAR registered with main server on port {config.lidar_web_port}")
                
                # Don't call lidar_api.start_server() - use existing server

            # Register potholes with server (independent of whether LiDAR is enabled)
            if server_module is not None:
                server_module.set_pothole_manager(pothole_mgr)
                print(f"🕳️  Potholes registered with main server (via module ref)")
            else:
                try:
                    from src.lidar import server as lidar_server
                    lidar_server.set_pothole_manager(pothole_mgr)
                    print(f"🕳️  Potholes registered with lidar server")
                except ImportError:
                    logger.warning("Could not register pothole manager with server")

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
            # CRITICAL: Traffic manager speed percentage is negative to INCREASE speed!
            # Negative value = faster, Positive value = slower
            # CARLA docs: "If less than zero, it's a % increase. If greater, it's a % decrease."
            tm.global_percentage_speed_difference(-20.0)  # 20% FASTER than speed limit (negative = faster!)
            tm.set_global_distance_to_leading_vehicle(config.safety_distance)
            
            print(f"   ✓ Traffic Manager configured: speed=+20% (faster!), ignores most lights/signs")

            ego_agent = None
            leading_agent = None
            leading_vehicle = None

            if BEHAVIOR_AGENT_AVAILABLE:
                # Ego: BehaviorAgent with 'normal' behavior
                ego_agent = BehaviorAgent(ego, behavior=config.ego_behavior)
                ego_dest_wp = random.choice(session.world.get_map().generate_waypoints(2.0))
                ego_agent.set_destination(ego_dest_wp.transform.location)
                print(f"   ✓ Ego: BehaviorAgent (normal) - destination set")
            else:
                ego.set_autopilot(True, config.tm_port)
                tm.update_vehicle_lights(ego, True)
                tm.ignore_lights_percentage(ego, 80)  # Ignore 80% of lights to reduce stopping
                tm.ignore_signs_percentage(ego, 80)  # Ignore 80% of signs
                tm.auto_lane_change(ego, True)
                tm.distance_to_leading_vehicle(ego, 1.5)  # Closer following for ego
                tm.vehicle_percentage_speed_difference(ego, -30.0)  # Ego drives 30% FASTER (negative!)
                print(f"   ✓ Ego autopilot (BehaviorAgent not available)")
            print()
            
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
            print(f"   ✓ Total vehicles in simulation: {len(traffic) + 1}")

            # Designate first traffic vehicle as intelligent leading vehicle
            if BEHAVIOR_AGENT_AVAILABLE and traffic:
                leading_vehicle = traffic[0]
                leading_agent = BehaviorAgent(leading_vehicle, behavior=config.leading_behavior)
                lead_dest_wp = random.choice(session.world.get_map().generate_waypoints(2.0))
                leading_agent.set_destination(lead_dest_wp.transform.location)
                # Remove autopilot from leading vehicle (BehaviorAgent controls it manually)
                leading_vehicle.set_autopilot(False, config.tm_port)
                print(f"   ✓ Leading vehicle (ID {leading_vehicle.id}): BehaviorAgent (normal)")
            print()

            # ========================================================================
            # STEP 5b: Spawn VRUs (Pedestrians + Cyclists) — V2X Extension
            # VRU IDs are offset from vehicle IDs to avoid collisions:
            #   pedestrians: num_vehicles … num_vehicles+num_pedestrians-1
            #   cyclists:    num_vehicles+num_pedestrians … (+ num_cyclists-1)
            # Walker AI controllers must be started AFTER a world.tick().
            # ========================================================================
            walker_controllers: List[carla.Actor] = []
            _num_pedestrians = getattr(config, 'num_pedestrians', 0)
            _num_cyclists    = getattr(config, 'num_cyclists', 0)

            if _num_pedestrians > 0:
                print(f"🚶 Spawning {_num_pedestrians} pedestrians (VRU V2P)...")
                walker_bps   = list(session.bp_lib.filter('walker.pedestrian.*'))
                walker_ai_bp = session.bp_lib.find('controller.ai.walker')
                walkers_spawned = 0
                _ped_vru_base = num_vehicles

                for ped_idx in range(_num_pedestrians):
                    nav_loc = session.world.get_random_location_from_navigation()
                    if nav_loc is None:
                        continue
                    w_bp = random.choice(walker_bps)
                    walker = session.world.try_spawn_actor(
                        w_bp, carla.Transform(nav_loc)
                    )
                    if walker is None:
                        continue
                    session.add_actor(walker)
                    ctrl = session.world.try_spawn_actor(
                        walker_ai_bp,
                        carla.Transform(nav_loc),
                        attach_to=walker,
                    )
                    if ctrl is not None:
                        session.add_actor(ctrl)
                        walker_controllers.append(ctrl)
                    # Register as VRU (requires V2XNetworkEnhanced)
                    if isinstance(v2v, V2XNetworkEnhanced):
                        v2v.register_vru(
                            _ped_vru_base + ped_idx, walker, VRUProfile.PEDESTRIAN
                        )
                    walkers_spawned += 1

                # CRITICAL: tick once so walker physics initialise before controllers start
                session.world.tick()
                for ctrl in walker_controllers:
                    try:
                        ctrl.start()
                        dest = session.world.get_random_location_from_navigation()
                        if dest:
                            ctrl.go_to_location(dest)
                        ctrl.set_max_speed(1.4)  # ~5 km/h walking speed
                    except Exception as e:
                        logger.debug(f"Walker controller start error: {e}")

                print(f"   ✓ Spawned {walkers_spawned} pedestrians "
                      f"({_num_pedestrians - walkers_spawned} spawn failures)")

            if _num_cyclists > 0:
                print(f"🚲 Spawning {_num_cyclists} cyclists (VRU V2P)...")
                cyclist_bps = [
                    bp for bp in session.bp_lib.filter('vehicle.*')
                    if int(bp.get_attribute('number_of_wheels')) == 2
                ]
                cyclists_spawned = 0
                _cyc_vru_base = num_vehicles + _num_pedestrians

                for cyc_idx in range(_num_cyclists):
                    if not cyclist_bps:
                        logger.warning("No 2-wheeled blueprints found; skipping cyclists")
                        break
                    bp = random.choice(cyclist_bps)
                    # Cyclists use road spawn points (they need paved surface)
                    sp_offset = num_vehicles + cyc_idx
                    if sp_offset >= len(road_spawn_points):
                        break
                    cyclist = session.world.try_spawn_actor(
                        bp, road_spawn_points[sp_offset]
                    )
                    if cyclist is None:
                        continue
                    session.add_actor(cyclist)
                    cyclist.set_autopilot(True, config.tm_port)
                    tm.vehicle_percentage_speed_difference(cyclist, 60.0)  # ~15 km/h
                    if isinstance(v2v, V2XNetworkEnhanced):
                        v2v.register_vru(
                            _cyc_vru_base + cyc_idx, cyclist, VRUProfile.CYCLIST
                        )
                    cyclists_spawned += 1

                print(f"   ✓ Spawned {cyclists_spawned} cyclists "
                      f"({_num_cyclists - cyclists_spawned} spawn failures)")

            # ========================================================================
            # STEP 5c: Create RSUs from CARLA Traffic Lights — V2X Extension
            # RSU IDs use intersection_id; we limit to the first 10 intersections
            # to avoid flooding the network.
            # ========================================================================
            if getattr(config, 'v2x_rsu_enabled', False) and isinstance(v2v, V2XNetworkEnhanced):
                print(f"🚦 Creating RSUs from CARLA traffic lights...")
                all_tl = list(session.world.get_actors().filter('traffic.traffic_light'))
                # Group by intersection_id (CARLA groups lights at same junction)
                intersection_seen: dict = {}
                for tl in all_tl:
                    iid = tl.get_pole_index()  # 0-based pole index per junction
                    junction_id = id(tl) % 10000  # Stable pseudo-ID per actor
                    # Use first light per intersection for simplicity
                    if junction_id not in intersection_seen:
                        intersection_seen[junction_id] = tl
                # Register up to 10 RSUs
                rsu_count = 0
                for rsu_idx, (jid, tl) in enumerate(list(intersection_seen.items())[:10]):
                    rsu = RoadSideUnit.from_traffic_light(tl, intersection_id=rsu_idx + 1)
                    v2v.register_rsu(rsu)
                    rsu_count += 1
                print(f"   ✓ Registered {rsu_count} RSUs (MAP+SPAT, 2 Hz)")
                print()

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
                observers.append(CSVDataLogger(run_dir=run_dir))
                print(f"   ✓ Ego telemetry logger: {run_dir.ego_telemetry_path}")

            if config.compact_logging:
                observers.append(CompactLogObserver(logger))
                print(f"   ✓ Compact log observer")

            # Bird's-eye view spectator camera following ego vehicle
            if config.spectator_follow:
                observers.append(SpectatorFollowObserver(
                    world=session.world,
                    vehicle=ego,
                    height=config.spectator_height,
                    pitch=config.spectator_pitch,
                    update_interval_frames=1
                ))
                print(f"   ✓ Spectator follow camera (height={config.spectator_height}m, pitch={config.spectator_pitch}°)")

            # Detailed V2V message logging for research analysis
            if config.v2v_message_logging and v2v:
                observers.append(V2VMessageLogger(
                    v2v_network=v2v,
                    run_dir=run_dir,
                    log_interval_frames=config.v2v_update_interval_frames
                ))
                print(f"   ✓ V2V message logger: {run_dir.v2v_messages_path}")

            # Pothole proximity events — always active for data collection
            observers.append(PotholeEventLogger(run_dir=run_dir, fps=config.fps))
            print(f"   ✓ Pothole event logger: {run_dir.pothole_events_path}")

            print(f"   Total: {len(observers)} observers registered\n")
            
            # ========================================================================
            # STEP 7: Warmup Period
            # ========================================================================
            print(f"⏱️  Warming up simulation ({config.warmup_frames} frames)...")
            print(f"   Initializing Traffic Manager routes...")
            for i in range(config.warmup_frames):
                # Drive BehaviorAgent-controlled vehicles during warmup
                if ego_agent is not None:
                    try:
                        ego.apply_control(ego_agent.run_step())
                    except Exception:
                        pass
                if leading_agent is not None and leading_vehicle is not None:
                    try:
                        leading_vehicle.apply_control(leading_agent.run_step())
                    except Exception:
                        pass
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

                    # Apply intelligent agent controls BEFORE tick
                    if ego_agent is not None:
                        try:
                            ego_control = ego_agent.run_step()
                            ego.apply_control(ego_control)
                            if ego_agent.done():
                                new_wp = random.choice(session.world.get_map().generate_waypoints(2.0))
                                ego_agent.set_destination(new_wp.transform.location)
                        except Exception as e:
                            logger.debug(f"Ego agent step error: {e}")

                    if leading_agent is not None and leading_vehicle is not None:
                        try:
                            lead_control = leading_agent.run_step()
                            leading_vehicle.apply_control(lead_control)
                            if leading_agent.done():
                                new_wp = random.choice(session.world.get_map().generate_waypoints(2.0))
                                leading_agent.set_destination(new_wp.transform.location)
                        except Exception as e:
                            logger.debug(f"Leading agent step error: {e}")

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

                    # Draw pothole debug boxes in CARLA client (same interval as V2V)
                    if frame % config.v2v_update_interval_frames == 0:
                        pothole_mgr.draw_debug()

                    # Create vehicle state object (uses lazy evaluation internally)
                    state: VehicleState = VehicleState.from_snapshot(
                        frame=frame,
                        actor_snapshot=ego_snapshot,
                        control=ego.get_control(),
                        sim_time=float(snapshot.timestamp.elapsed_seconds),
                        fixed_delta_seconds=config.fixed_delta_seconds
                    )

                    neighbors = v2v.get_neighbors(0) if v2v else []
                    threats = v2v.get_threats(0) if v2v else []
                    neighbor_distance_map: Dict[int, float] = {}
                    if v2v:
                        for neighbor in neighbors:
                            neighbor_distance_map[neighbor.vehicle_id] = (
                                v2v.get_distance(0, neighbor.vehicle_id) or 0.0
                            )
                    
                    # Prepare V2V data for observers
                    v2v_data: Dict[str, Any] = {
                        'neighbors': neighbors,
                        'threats': threats,
                        'bsm': v2v.get_bsm(0) if v2v else None,
                        'neighbor_distance_map': neighbor_distance_map,
                        'total_vehicles': actor_mgr.count(),
                        'lidar_points': lidar_api.get_point_count() if lidar_api else 0,
                        'potholes': pothole_mgr.to_dict_list(),
                        'nearby_potholes': pothole_mgr.get_nearby(
                            state.position[0], state.position[1], radius=50.0
                        ),
                    }
                    
                    # Notify all observers (they use lazy evaluation internally)
                    for observer in observers:
                        observer.on_frame(frame, state, v2v_data)
                    
                    # Update status callback if provided (for web API)
                    if status_callback and frame % 10 == 0:
                        current_elapsed = state.sim_time
                        v2v_msgs = v2v.get_network_stats()['total_messages_sent'] if v2v else 0
                        status_callback(frame, current_elapsed, v2v_msgs)
                    
                    # Track frame time for performance analysis
                    frame_time: float = time.perf_counter() - frame_start
                    frame_times.append(frame_time)

                    # Real-time pacing: sleep the unused portion of the target
                    # frame period so the simulation runs at ≈ 1× real-time speed.
                    # Without this, synchronous mode ticks as fast as the CPU/GPU
                    # allows (often 5–20× faster than real time).
                    sleep_for = config.fixed_delta_seconds - frame_time
                    if sleep_for > 0.001:
                        time.sleep(sleep_for)
                    
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

                # DSRC/WAVE channel statistics
                ch_stats = v2v.get_channel_stats()
                print(f"\n   DSRC/WAVE Channel (IEEE 802.11p):")
                print(f"     Total tx attempts:  {int(ch_stats['total_broadcasts'])}")
                print(f"     Delivered:          {int(ch_stats['total_deliveries'])}")
                print(f"     Dropped:            {int(ch_stats['total_drops'])}")
                print(f"     Packet reception rate: {ch_stats['prr']:.3f}")
                print(f"     Avg SNR margin:     {ch_stats['avg_snr_margin_db']:.1f} dB")
                
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

                # V2X-specific stats (only when V2XNetworkEnhanced is active)
                if isinstance(v2v, V2XNetworkEnhanced):
                    v2x_summary = v2v.get_v2x_summary()
                    print(f"\n   V2X Extensions:")
                    print(f"     RSUs active:       {v2x_summary['rsu_count']}")
                    print(f"     VRUs tracked:      {v2x_summary['vru_count']}")
                    print(f"     Active SPAT msgs:  {v2x_summary['active_spat']}")
                    print(f"     Active VRUAM msgs: {v2x_summary['active_vruam']}")
            
            # LiDAR statistics
            if lidar_api:
                print(f"\n🎯 LiDAR:")
                print(f"   Total points streamed: {lidar_api.get_point_count() * frame:,}")
                print(f"   Web server port:       {config.lidar_web_port}")

            # Pothole statistics
            print(f"\n🕳️  Potholes (BSM communication):")
            print(f"   Total registered:  {len(pothole_mgr)}")
            if pothole_mgr.potholes:
                sev_counts = {1: 0, 2: 0, 3: 0}
                for ph in pothole_mgr.potholes:
                    sev_counts[ph.severity] += 1
                print(f"   Minor  (sev 1):  {sev_counts[1]}")
                print(f"   Moderate (sev 2):{sev_counts[2]}")
                print(f"   Severe (sev 3):  {sev_counts[3]}")
            else:
                print(f"   (No potholes registered — add them to your custom map)")
            print(f"   REST API:        /api/potholes")

            # Notify observers of completion
            for observer in observers:
                observer.on_complete(frame, elapsed_time)
            
            print(f"\n{'='*80}")
            # Write run_summary.json
            v2v_stats = v2v.get_network_stats() if v2v else {}
            run_dir.write_summary({
                'carla': {
                    'host': config.host,
                    'port': config.port,
                    'map': session.world.get_map().name,
                },
                'config': {
                    'duration_s': config.duration,
                    'num_vehicles': config.num_vehicles,
                    'v2v_range_m': config.v2v_range,
                    'lidar_enabled': config.lidar_enabled,
                    'random_seed': config.random_seed,
                },
                'potholes': {
                    'count': len(pothole_mgr),
                    'severity_1_minor':    sum(1 for p in pothole_mgr.potholes if p.severity == 1),
                    'severity_2_moderate': sum(1 for p in pothole_mgr.potholes if p.severity == 2),
                    'severity_3_severe':   sum(1 for p in pothole_mgr.potholes if p.severity == 3),
                    'locations': pothole_mgr.to_dict_list(),
                    'note': 'Potholes are now manually registered from custom maps (no auto-generation)'
                },
                'results': {
                    'total_frames': frame,
                    'real_time_s': round(elapsed_time, 2),
                    'simulated_time_s': round(frame / config.fps, 2),
                    'v2v_messages_sent': v2v_stats.get('total_messages_sent', 0),
                    'avg_neighbors': round(v2v_stats.get('average_neighbors', 0.0), 2),
                    'max_neighbors': v2v_stats.get('max_neighbors', 0),
                },
                'files': {
                    'ego_telemetry':  'ego_telemetry.csv'  if config.csv_logging else None,
                    'v2v_messages':   'v2v_messages.csv'   if config.v2v_message_logging else None,
                    'pothole_events': 'pothole_events.csv',
                },
            })
            print(f"📋 Run summary written: {run_dir.run_summary_path}")

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
        # Stop LiDAR collector but don't stop the server (it's shared)
        if lidar_api:
            # Signal server to stop streaming BEFORE cleanup
            if server_module is not None:
                logger.info("Signaling server to stop streaming...")
                server_module.set_collector(None)  # Cancel streaming task
            else:
                try:
                    from src.lidar import server as lidar_server
                    logger.info("Signaling server to stop streaming...")
                    lidar_server.set_collector(None)  # Cancel streaming task
                except ImportError:
                    pass
            
            # Small delay to allow streaming task to cancel
            time.sleep(0.2)
            
            # Now cleanup collector
            lidar_api.collector.cleanup()
            print("✓ LiDAR streaming stopped")

        # Shutdown V2V network (logs final channel stats, no external connections)
        if v2v:
            v2v.shutdown()

        # Stop walker AI controllers before session destroys actors
        for ctrl in walker_controllers:
            try:
                ctrl.stop()
            except Exception:
                pass

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
    parser.add_argument('--host', default='192.168.1.101', 
                       help='CARLA server IP (default: 192.168.1.101)')
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
    
    # Spectator Camera
    parser.add_argument('--spectator-follow', action='store_true', default=True,
                       help='Enable bird\'s-eye view spectator following (default: enabled)')
    parser.add_argument('--no-spectator-follow', action='store_false', dest='spectator_follow',
                       help='Disable spectator following')
    parser.add_argument('--spectator-height', type=float, default=50.0,
                       help='Spectator camera height above vehicle (default: 50m)')
    parser.add_argument('--spectator-pitch', type=float, default=-90.0,
                       help='Spectator camera pitch angle (default: -90 = straight down)')
    
    # V2V Message Logging
    parser.add_argument('--v2v-logging', action='store_true',
                       help='Enable detailed V2V message logging for analysis')
    parser.add_argument('--v2v-log-output',
                       help='V2V message log file path (default: auto-generated in logs/)')

    # V2X extensions
    parser.add_argument('--pedestrians', type=int, default=0,
                       help='Number of pedestrians to spawn as VRUs (default: 0)')
    parser.add_argument('--cyclists', type=int, default=0,
                       help='Number of cyclists to spawn as VRUs (default: 0)')
    parser.add_argument('--rsu', action='store_true',
                       help='Create RSUs from CARLA traffic lights (V2I)')
    
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
    
    # Apply spectator follow settings
    if args.spectator_follow:
        config.spectator_follow = True
        config.spectator_height = args.spectator_height
        config.spectator_pitch = args.spectator_pitch
    
    # Apply V2V message logging settings
    if args.v2v_logging:
        config.v2v_message_logging = True
        if args.v2v_log_output:
            config.v2v_log_output_path = args.v2v_log_output
    
    # Apply V2X settings
    if getattr(args, 'pedestrians', 0) or getattr(args, 'cyclists', 0) or getattr(args, 'rsu', False):
        config.num_pedestrians = getattr(args, 'pedestrians', 0)
        config.num_cyclists    = getattr(args, 'cyclists', 0)
        config.v2x_rsu_enabled = getattr(args, 'rsu', False)

    # Run the complete demonstration
    run_complete_v2v_demo(config)


if __name__ == '__main__':
    main()


def run_simulation_headless(
    carla_host: str = "192.168.1.101",
    carla_port: int = 2000,
    duration: int = 120,
    vehicles: int = 10,
    v2v_range: float = 75.0,
    lidar_quality: str = "high",
    csv_logging: bool = True,
    console_output: bool = True,
    spectator_follow: bool = True,
    spectator_height: float = 50.0,
    spectator_pitch: float = -90.0,
    v2v_message_logging: bool = False,
    dsrc_channel_busy_ratio: float = 0.15,
    dsrc_tx_power_dbm: float = 23.0,
    ego_behavior: str = 'normal',
    leading_behavior: str = 'normal',
    status_callback = None,
    server_module = None
):
    """
    Run simulation in headless mode (callable from API).

    Args:
        carla_host: CARLA server IP address
        carla_port: CARLA server port
        duration: Simulation duration in seconds
        vehicles: Number of vehicles
        v2v_range: V2V cooperative awareness zone in meters
        lidar_quality: LiDAR quality ('high', 'medium', 'fast')
        csv_logging: Enable CSV logging
        console_output: Enable console output
        spectator_follow: Enable bird's-eye view camera following
        spectator_height: Height of spectator camera above vehicle
        spectator_pitch: Pitch angle of spectator camera (degrees)
        v2v_message_logging: Enable detailed V2V message logging
        dsrc_channel_busy_ratio: DSRC CBR [0,1] — 0=empty, 1=saturated channel
        dsrc_tx_power_dbm: DSRC transmit power (dBm, ETSI default: 23 dBm)
        status_callback: Function called with status updates (frame, elapsed, v2v_msgs)
        server_module: Server module reference for LiDAR server integration
    """
    builder = (ScenarioBuilder()
        .with_carla_server(carla_host, carla_port)
        .with_duration(duration)
        .with_vehicles(vehicles)
        .with_seed(42)
        .with_v2v(enabled=True, range_m=v2v_range)
        .with_console_output(enabled=console_output)
        .with_carla_debug(enabled=False)
        .with_spectator_follow(enabled=spectator_follow, height=spectator_height, pitch=spectator_pitch)
        .with_v2v_message_logging(enabled=v2v_message_logging)
        .with_dsrc_channel(
            tx_power_dbm=dsrc_tx_power_dbm,
            channel_busy_ratio=dsrc_channel_busy_ratio,
        )
        .with_agent_behavior(ego_behavior=ego_behavior, leading_behavior=leading_behavior)
    )
    config: ScenarioConfig = builder.build()
    
    # Apply LiDAR settings
    config.lidar_enabled = True
    config.lidar_quality = lidar_quality
    config.lidar_web_port = 8000
    
    # Apply logging
    config.csv_logging = csv_logging
    
    # Run with status updates
    run_complete_v2v_demo(config, status_callback=status_callback, server_module=server_module)
