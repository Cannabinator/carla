# CARLA V2V Research Platform - AI Agent Instructions

Production-ready V2V (Vehicle-to-Vehicle) communication and real-time LiDAR visualization for CARLA Simulator 0.9.16.

## Architecture Overview

**Design Patterns in Use:**
- **Context Manager**: `CARLASession` ([src/utils/session.py](src/utils/session.py)) - guarantees cleanup of CARLA actors/settings even on exceptions
- **Builder**: `ScenarioBuilder` ([src/utils/builder.py](src/utils/builder.py)) - fluent API for spawning vehicles and sensors
- **Observer**: Multiple observers ([src/utils/observers.py](src/utils/observers.py)) - ConsoleObserver, CSVDataLogger, CompactLogObserver for different output formats
- **Lazy Evaluation**: `LazyVehicleStats` ([src/utils/lazy.py](src/utils/lazy.py)) - 10-20% CPU savings by computing only when accessed

**Core Components:**
- **V2V Network** ([src/v2v/](src/v2v/)): SAE J2735 BSM (Basic Safety Message) protocol implementation with 2 Hz update rate, neighbor discovery, and threat assessment
- **LiDAR Visualization** ([src/visualization/lidar/](src/visualization/lidar/)): FastAPI WebSocket server streaming semantic LiDAR to Three.js web viewer
- **Binary Protocol** ([src/utils/binary_protocol.py](src/utils/binary_protocol.py)): 73% bandwidth reduction vs JSON for point cloud streaming
- **Octree Downsampling** ([src/utils/octree.py](src/utils/octree.py)): 50-70% point reduction while preserving structure

## Critical Developer Workflows

### Running Scenarios
```bash
# V2V + LiDAR visualization (complete demo)
./run_v2v_lidar.sh

# Or manually:
source venv/bin/activate
python src/scenarios/v2v_complete_demo.py --carla-host 192.168.1.110 --duration 120
# Web viewer: http://localhost:8000
```

### Testing Strategy
```bash
# Unit tests (no CARLA needed) - run these FIRST
python -m pytest tests/v2v/ -v

# Integration tests (require CARLA server at 192.168.1.110:2000)
python -m pytest tests/test_v2v_lidar.py -v
python tests/test_frontend_visual.py --run  # Frontend tests with 20 automated checks
```

### CARLA Connection Pattern
**ALWAYS use** `CARLASession` context manager to prevent actor leaks:
```python
from src.utils import CARLASession
from src.config import DEFAULT_SIM_CONFIG

with CARLASession('192.168.1.110', 2000, DEFAULT_SIM_CONFIG) as session:
    # session.world, session.actors, session.bp_lib available
    ego = session.world.spawn_actor(blueprint, spawn_point)
    session.actors.append(ego)  # Automatically destroyed on exit
```

## Project-Specific Conventions

### Configuration Management
**Use centralized config dataclasses** - NO magic numbers in code:
```python
from src.config import DEFAULT_SIM_CONFIG, DEFAULT_V2V_CONFIG
# Access: DEFAULT_SIM_CONFIG.fixed_delta_seconds, DEFAULT_V2V_CONFIG.max_range
```

### Type Safety
- 90% type coverage enforced (see [pyrightconfig.json](pyrightconfig.json))
- All V2V messages use `@dataclass` with explicit types ([src/v2v/messages.py](src/v2v/messages.py))
- Pyright issues mostly suppressed for CARLA API compatibility

### Synchronous Mode Requirements
CARLA runs at **fixed 20 FPS** (`fixed_delta_seconds=0.05`):
- V2V updates enforced at **2 Hz** via internal throttling in `V2VNetworkEnhanced.update()`
- LiDAR streaming at **10 Hz** (configurable in server)
- **NEVER** use `time.sleep()` in simulation loop - use `world.tick()` and frame counting

### Observer Pattern Usage
Register multiple observers for different outputs:
```python
from src.utils import ConsoleObserver, CSVDataLogger, CompactLogObserver

observers = [
    ConsoleObserver(),  # Rich terminal output
    CSVDataLogger(log_dir / f"scenario_data_{timestamp}.csv"),  # Data collection
    CompactLogObserver()  # One-line status updates
]
for observer in observers:
    observer.on_frame_update(frame_data)
```

## Integration Points

### V2V Communication Flow
1. Register vehicles: `v2v_network.register(vehicle.id, vehicle)`
2. Simulation loop: `world.tick()` → `v2v_network.update()` (auto-throttles to 2 Hz)
3. Access data: `v2v_network.get_neighbors(ego_id)` returns list of `V2VEnhancedMessage` with BSM data

### LiDAR WebSocket Streaming
**Continuous streaming pattern** (see [src/visualization/lidar/server.py](src/visualization/lidar/server.py)):
- Streaming task starts at server startup and runs continuously
- Task dynamically checks global `_collector` reference each iteration
- Automatically waits for both collector AND WebSocket connections
- No need to restart task when collector changes - just set `_collector`
- Collector registered via `set_collector(collector)` - streaming begins automatically

### Performance Optimizations
- **Binary Protocol**: Use `BinaryProtocol.encode()` instead of JSON for 40-50% bandwidth savings
- **Octree Downsampling**: `OctreeDownsampler(voxel_size=0.5).downsample(points)` reduces point count
- **Lazy Stats**: Access vehicle stats via `LazyVehicleStats(snapshot).speed_kmh` - computed only once

## Common Pitfalls

1. **Hybrid Physics Issue**: Set `use_hybrid_physics=False` in config - hybrid mode causes zero velocity bugs
2. **Traffic Manager Port**: Must differ from web server port (TM=8001, WebServer=8000)
3. **Path Issues**: Server must add project root to `sys.path` for imports to work in threads:
   ```python
   project_root = Path(__file__).parent.parent.parent
   sys.path.insert(0, str(project_root))
   ```
4. **WebSocket Event Loop**: Store event loop reference when server starts to enable cross-thread task scheduling
5. **LiDAR Cleanup Race Condition**: ALWAYS signal server to stop streaming BEFORE collector.cleanup():
   ```python
   server_module.set_collector(None)  # Cancel streaming task
   time.sleep(0.2)  # Allow task to cancel
   collector.cleanup()  # Now safe to cleanup
   ```
   Prevents "No data" warnings from streaming loop trying to access cleaned-up collector

## Key Files for Reference

- V2V Protocol: [src/v2v/messages.py](src/v2v/messages.py) (BSMCore, BSMPartII, threat assessment)
- Network Manager: [src/v2v/network_enhanced.py](src/v2v/network_enhanced.py) (2 Hz enforcement, neighbor discovery)
- LiDAR Server: [src/visualization/lidar/server.py](src/visualization/lidar/server.py) (FastAPI WebSocket streaming)
- Complete Example: [src/scenarios/v2v_complete_demo.py](src/scenarios/v2v_complete_demo.py) (demonstrates all patterns)
- User Guides: [V2V_GUIDE.md](V2V_GUIDE.md), [V2V_IMPLEMENTATION.md](V2V_IMPLEMENTATION.md), [README.md](README.md)

## Legacy Files (kept for backwards compatibility)

- [src/v2v/communicator.py](src/v2v/communicator.py) - Legacy `V2VNetwork` class (use `V2VNetworkEnhanced` instead)
- [src/v2v/protocol.py](src/v2v/protocol.py) - Legacy `V2VState` dataclass (used by communicator.py)
