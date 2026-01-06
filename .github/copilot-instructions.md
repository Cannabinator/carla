# GitHub Copilot Instructions for CARLA V2V Research Platform

Production-ready V2V (Vehicle-to-Vehicle) communication framework for CARLA Simulator 0.9.16. **Remote architecture**: CARLA server on Windows (192.168.1.110:2000), Python client on Ubuntu 24.04.

⚠️ **ALWAYS** reference CARLA official documentation: https://carla.readthedocs.io/en/0.9.16/ for API usage patterns.

**Scientific Requirements**: All scenarios must be deterministic and reproducible. Always update `requirements.txt` when adding dependencies.

## Project Overview
Highly optimized research platform featuring:
- **V2V Communication**: SAE J2735 BSM protocol with 2 Hz updates, 150m range
- **Real-time LiDAR Visualization**: Web-based 3D viewer with semantic coloring
- **Performance Optimizations**: 73% bandwidth reduction (binary WebSocket), 50-70% point reduction (octree), lazy evaluation
- **Professional Architecture**: Observer pattern, builder pattern, context managers, dataclasses, type hints (90% coverage)

## Architecture Patterns

### 1. Context Manager Pattern (Guaranteed Cleanup)
**ALWAYS** use `CARLASession` for robust resource management:
```python
from src.utils import CARLASession
from src.config import DEFAULT_SIM_CONFIG

with CARLASession('192.168.1.110', 2000, DEFAULT_SIM_CONFIG) as session:
    ego = session.world.spawn_actor(bp, spawn_point)
    session.actors.append(ego)  # Tracks for auto-cleanup
    # ... scenario code ...
# Automatic cleanup: restores settings, destroys actors, handles exceptions
```

### 2. Builder Pattern (Scenario Configuration)
Use `ScenarioBuilder` for fluent, type-safe configuration:
```python
from src.utils import ScenarioBuilder, get_performance_config

config = (ScenarioBuilder()
    .with_carla_server('192.168.1.110', 2000)
    .with_duration(60)
    .with_vehicles(20)
    .with_v2v(range_m=150.0)
    .with_lidar(quality='high', port=8000)
    .build())

# Or use factory methods for common configurations
config = get_performance_config()  # High-performance preset
```

### 3. Observer Pattern (Visualization & Logging)
**Separation of concerns**: Keep scenario logic separate from visualization. Observers use lazy evaluation internally for efficiency.
```python
from src.utils import ConsoleObserver, CARLADebugObserver, CSVDataLogger, CompactLogObserver

observers = [
    ConsoleObserver(interval_seconds=2.0, fps=20),  # Console stats
    CARLADebugObserver(session.world, v2v, interval_frames=5),  # 3D visualization
    CSVDataLogger(output_path='data/scenario.csv'),  # Data export
    CompactLogObserver(logger)  # Structured logging
]

# In simulation loop
for observer in observers:
    observer.on_frame(frame, state, v2v_data)

# After completion
for observer in observers:
    observer.on_complete(total_frames, elapsed_time)
```

### 4. Lazy Evaluation (Performance)
Use lazy properties to avoid expensive computations until needed:
```python
from src.utils import LazyVehicleStats, LazyProperty

# Automatic lazy computation
stats = LazyVehicleStats(snapshot)
if condition:
    speed = stats.speed_kmh  # Only computed if accessed
```

### 5. Configuration Management
Use centralized dataclasses from `src/config.py`:
```python
from src.config import (
    DEFAULT_SIM_CONFIG,      # SimulationConfig
    DEFAULT_VIZ_CONFIG,      # VisualizationConfig  
    DEFAULT_V2V_CONFIG,      # V2VConfig
    DEFAULT_VEHICLE_CONFIG   # VehicleSpawnConfig
)

# Customize as needed
config = DEFAULT_SIM_CONFIG
config.random_seed = 123
```

## CARLA Critical Patterns

### Synchronous Mode (MANDATORY for Reproducibility)
```python
# ALWAYS use synchronous mode for reproducibility
settings = world.get_settings()
settings.synchronous_mode = True
settings.fixed_delta_seconds = 0.05  # Fixed 20 FPS
world.apply_settings(settings)

random.seed(seed)
np.random.seed(seed)

# CRITICAL: Must tick world in sync mode
while running:
    world.tick()  # Advances simulation by fixed_delta_seconds
```

### Cleanup Pattern (MANDATORY)
```python
try:
    # scenario code
finally:
    if world:
        settings = world.get_settings()
        settings.synchronous_mode = False
        settings.fixed_delta_seconds = None
        world.apply_settings(settings)
    
    if client and actors:
        client.apply_batch([carla.command.DestroyActor(x) for x in actors])
```

## V2V Communication System

### Three V2V Implementations
The codebase provides three V2V systems for different use cases:

1. **V2VNetwork** (`src/v2v/communicator.py`) - Lightweight neighbor discovery
   - Simple distance-based neighbor detection
   - Efficient state sharing with V2VState dataclass
   - Best for: Basic V2V scenarios

2. **V2VNetworkEnhanced** (`src/v2v/network_enhanced.py`) - Industry-standard BSM protocol
   - SAE J2735 Basic Safety Message (BSM) implementation
   - 2 Hz update rate (configurable)
   - Threat assessment with Time-To-Collision
   - Cooperative perception sharing
   - Best for: Research requiring standard V2V protocols

3. **V2VAPI** (`src/v2v/api.py`) - REST/WebSocket interface
   - FastAPI-based HTTP endpoints
   - Real-time WebSocket streaming
   - External system integration
   - Best for: Web dashboards, external monitoring

### Basic V2V Pattern (Lightweight)
```python
from src.v2v import V2VNetwork

# Initialize with 50m range
v2v = V2VNetwork(max_range=50.0)

# Register vehicles
v2v.register(0, ego_vehicle)
v2v.register(i, traffic_vehicle)

# Update network (every 0.2s in simulation loop)
v2v.update()  # Updates states + finds neighbors

# Get neighbors
neighbors = v2v.get_neighbors(0)  # Returns List[V2VState]
```

## CARLA 0.9.16 API Essentials

### Connection & Setup
```python
client = carla.Client(host, port)
client.set_timeout(30.0)  # Long timeout for remote
world = client.get_world()
```

### Traffic Manager (Deterministic)
```python
tm = client.get_trafficmanager(8000)
tm.set_synchronous_mode(True)  # MUST match world
tm.set_random_device_seed(seed)
# WARNING: Do NOT use hybrid physics mode - it causes zero velocity readings!
# tm.set_hybrid_physics_mode(True)  # CAUSES TELEPORTATION, NOT DRIVING
vehicle.set_autopilot(True, 8000)
```

### Getting Fresh Velocity Data (CRITICAL!)
```python
# WRONG - Returns cached/stale velocity data
vel = vehicle.get_velocity()  # One tick behind!

# CORRECT - Get snapshot immediately after tick for fresh data
world.tick()
snapshot = world.get_snapshot()  # Must be called RIGHT AFTER tick()
actor_snapshot = snapshot.find(vehicle.id)
vel = actor_snapshot.get_velocity()  # Fresh data from current tick!
```

### Spectator Camera (Follow Vehicle)
```python
spectator = world.get_spectator()
transform = vehicle.get_transform()
spectator.set_transform(carla.Transform(
    transform.location + carla.Location(x=-10, z=6),
    carla.Rotation(pitch=-20, yaw=transform.rotation.yaw)
))
```

## Developer Workflows

### Run Tests
```bash
# Unit tests (no CARLA needed)
python tests/v2v/test_network.py -v

# Reproducibility test (needs CARLA server)
python tests/test_reproducibility.py --host 192.168.1.110

# Frontend visual tests
python tests/test_frontend_visual.py --run

# V2V + LiDAR integration tests
python tests/test_v2v_lidar.py
```

### Run Scenarios
```bash
source venv/bin/activate

# High-performance V2V scenario (uses all patterns)
python src/scenarios/v2v_scenario_perf.py --host 192.168.1.110

# V2V + LiDAR visualization
./run_v2v_lidar.sh
# Or: python src/scenarios/v2v_lidar_scenario.py --carla-host 192.168.1.110

# V2V with REST API
python src/scenarios/v2v_api_scenario.py --host 192.168.1.110 --api-port 8001

# Basic scenario
python src/scenarios/run_scenario.py --host 192.168.1.110
```

### Web Interfaces
```bash
# LiDAR 3D viewer (after running v2v_lidar_scenario.py)
# Open: http://localhost:8000

# V2V REST API docs (after running v2v_api_scenario.py)
# Open: http://localhost:8001/docs
```

## Project Conventions

### Module Structure
- `src/scenarios/`: Executable scenarios with `if __name__ == "__main__"`
- `src/utils/`: Reusable utilities
- `src/v2v/`: V2V framework (lightweight, efficient)
- `src/visualization/`: Visualization tools
- `tests/`: Unit and integration tests

### Naming
- Scenarios: `*_scenario.py`
- Classes: PascalCase (e.g., `V2VNetwork`, `V2VState`)
- Functions: snake_case with docstrings
- Use dataclasses for data structures

### Reproducibility Requirements
1. **Fixed seed**: `random.seed()` and `np.random.seed()` before randomization
2. **Synchronous mode**: `settings.synchronous_mode = True` with `fixed_delta_seconds`
3. **Traffic manager seed**: `tm.set_random_device_seed(seed)`
4. **Fixed spawn points**: Use deterministic spawn point selection

## Common Pitfalls

1. **Forgetting `world.tick()`**: In sync mode, nothing happens until you tick
2. **Wrong TM port**: Traffic manager port must match in `get_trafficmanager()` and `set_autopilot()`
3. **Not checking `if world:`**: Cleanup runs even if connection fails
4. **Blocking operations**: Keep update loops fast, avoid heavy computation per frame

## Key Files to Reference

- **Architecture patterns**: `src/utils/session.py` (context manager), `src/utils/builder.py` (builder), `src/utils/observers.py` (observer)
- **V2V implementations**: `src/v2v/communicator.py` (basic), `src/v2v/network_enhanced.py` (BSM), `src/v2v/api.py` (REST API)
- **Performance**: `src/utils/lazy.py` (lazy evaluation), `src/utils/octree.py` (downsampling), `src/utils/binary_protocol.py` (binary WebSocket)
- **Visualization**: `src/visualization/lidar/api.py` (LiDAR API), `src/visualization/web/viewer.html` (3D viewer)
- **Example scenarios**: `src/scenarios/v2v_scenario_perf.py` (complete example), `src/scenarios/v2v_lidar_scenario.py` (LiDAR)
- **Configuration**: `src/config.py` (centralized configs)
- **Tests**: `tests/v2v/test_network.py` (unittest), `tests/test_reproducibility.py` (integration)

## Scientific Use Cases

This codebase supports:
- **V2V protocol research**: Efficient neighbor discovery within configurable range
- **Cooperative perception**: Vehicles share state information
- **Reproducible experiments**: Same seed → identical simulation
- **Multi-vehicle coordination**: State sharing and neighbor awareness

---

**When creating new scenarios**: Start from `src/scenarios/v2v_scenario.py`, follow connect→setup→spawn→run→cleanup pattern, always use synchronous mode, always restore settings in finally block.
