# GitHub Copilot Instructions for CARLA V2V Research Platform

ALLWAYS LOOKUP EXAMPLES AND USAGE OF CODE IN THE CARLA OFFICIAL DOCUMENTATION IF UNSURE: https://carla.readthedocs.io/en/0.9.16/
OR SEARCH CERTIFIED EXAMPLES ONLINE.

## Project Overview
Lightweight V2V (Vehicle-to-Vehicle) communication framework for CARLA Simulator 0.9.16. **Remote architecture**: CARLA server on Windows (192.168.1.110:2000), Python client on Ubuntu 24.04.

## Architecture & Critical Patterns

### CARLA Client Pattern (All Scenarios)
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

### Key Components
- **Protocol** (`src/v2v/protocol.py`): V2VState dataclass with efficient state representation
- **Network** (`src/v2v/communicator.py`): V2VNetwork for neighbor discovery and state sharing

### V2V Pattern
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
```

### Run Scenarios
```bash
source venv/bin/activate

# V2V communication demo
python src/scenarios/v2v_scenario.py --host 192.168.1.110

# Basic scenario
python src/scenarios/run_scenario.py --host 192.168.1.110
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

- **V2V framework**: `src/v2v/protocol.py` (dataclass pattern), `src/v2v/communicator.py` (neighbor discovery)
- **V2V scenario**: `src/scenarios/v2v_scenario.py` (visualization example)
- **Tests**: `tests/v2v/test_network.py` (unittest with mocks)

## Scientific Use Cases

This codebase supports:
- **V2V protocol research**: Efficient neighbor discovery within configurable range
- **Cooperative perception**: Vehicles share state information
- **Reproducible experiments**: Same seed → identical simulation
- **Multi-vehicle coordination**: State sharing and neighbor awareness

---

**When creating new scenarios**: Start from `src/scenarios/v2v_scenario.py`, follow connect→setup→spawn→run→cleanup pattern, always use synchronous mode, always restore settings in finally block.
