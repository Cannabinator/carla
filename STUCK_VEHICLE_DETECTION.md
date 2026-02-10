# Stuck Vehicle Detection and Recovery

## Overview

The stuck vehicle detection system automatically identifies and recovers traffic vehicles that have become immobile due to collisions or getting wedged in geometry.

## How It Works

### Detection Algorithm

The system uses a velocity-based detection approach:

1. **Velocity Monitoring**: Every vehicle's velocity is checked at regular intervals (default: every 20 frames / 1 second at 20 FPS)
2. **Stuck Counter**: If velocity falls below threshold (default: 0.5 m/s), a counter is incremented for that vehicle
3. **Stuck Threshold**: If the counter reaches the threshold (default: 100 frames / 5 seconds), the vehicle is marked as stuck
4. **Reset on Movement**: If the vehicle starts moving again before reaching the threshold, the counter is reset

### Recovery Strategy

When a stuck vehicle is detected, the system attempts recovery using two strategies:

1. **Physics Reset** (Primary): Resets the vehicle's linear and angular velocity to zero
2. **Teleport** (Secondary): If spawn points are available, teleports the vehicle to a new random spawn point

## Configuration

All stuck vehicle detection parameters are configurable in `src/config.py`:

```python
@dataclass
class SimulationConfig:
    # Stuck vehicle detection and recovery
    stuck_detection_enabled: bool = True  # Enable/disable stuck detection
    stuck_velocity_threshold: float = 0.5  # m/s - below this is considered stuck
    stuck_frames_threshold: int = 100  # frames - stuck for this many frames triggers recovery (5s at 20 FPS)
    stuck_check_interval_frames: int = 20  # Check every 20 frames (1s at 20 FPS)
```

### Parameters Explained

- **`stuck_detection_enabled`**: Master switch to enable/disable the feature
- **`stuck_velocity_threshold`**: Speed in m/s below which a vehicle is considered potentially stuck (default: 0.5 m/s = 1.8 km/h)
- **`stuck_frames_threshold`**: Number of consecutive frames below the threshold before triggering recovery (default: 100 frames = 5 seconds at 20 FPS)
- **`stuck_check_interval_frames`**: How often to check vehicles (default: every 20 frames = 1 second, to reduce performance overhead)

## Usage

The stuck vehicle detection is automatically integrated into the main scenario (`v2v_complete_demo.py`).

### Enabling/Disabling

To disable stuck vehicle detection, modify your scenario configuration:

```python
from src.config import SimulationConfig

config = SimulationConfig(
    stuck_detection_enabled=False  # Disable stuck detection
)
```

### Adjusting Sensitivity

To make detection more sensitive (trigger recovery sooner):

```python
config = SimulationConfig(
    stuck_velocity_threshold=1.0,  # Higher threshold (m/s)
    stuck_frames_threshold=50,     # Fewer frames needed
    stuck_check_interval_frames=10 # Check more frequently
)
```

To make detection less sensitive (allow more time before recovery):

```python
config = SimulationConfig(
    stuck_velocity_threshold=0.2,   # Lower threshold (m/s)
    stuck_frames_threshold=200,     # More frames needed
    stuck_check_interval_frames=40  # Check less frequently
)
```

## Statistics

At the end of each simulation, stuck vehicle statistics are displayed:

```
🚗 Stuck Vehicle Detection:
   Total recovered:       3
   Currently tracked:     15
   Currently stuck:       0
   Velocity threshold:    0.5 m/s
   Frames threshold:      100 frames (5.0s)
```

**Metrics Explained**:
- **Total recovered**: Number of vehicles that were recovered during the simulation
- **Currently tracked**: Number of vehicles being monitored
- **Currently stuck**: Number of vehicles that are currently below threshold but haven't been recovered yet
- **Velocity threshold**: The configured speed threshold
- **Frames threshold**: The configured frame threshold with equivalent time

## Implementation Details

### Core Class: `StuckVehicleTracker`

Located in `src/utils/carla_utils.py`, this class handles all stuck vehicle detection and recovery logic.

**Key Methods**:

- `check_and_update(vehicle, snapshot) -> bool`: Check if vehicle is stuck
- `recover_vehicle(vehicle, spawn_points, world) -> bool`: Attempt to recover stuck vehicle
- `get_stats() -> Dict`: Get tracking statistics
- `reset()`: Reset all tracking state

### Integration Points

1. **Initialization**: After Traffic Manager setup in scenario
2. **Simulation Loop**: Check every N frames during main loop
3. **Statistics**: Display at end of simulation

## Performance Considerations

The stuck vehicle detection system is designed to have minimal performance impact:

- **Lazy Checking**: Only checks vehicles every N frames (default: 20 frames)
- **Efficient Velocity Calculation**: Uses CARLA's snapshot system for fresh data
- **Per-Vehicle Tracking**: Only maintains a simple counter per vehicle
- **No Polling**: Integrated into existing simulation loop, no separate threads

Typical overhead: **< 0.1 ms per check** for 15 vehicles

## Troubleshooting

### Issue: Too many vehicles being recovered

**Cause**: Threshold too sensitive
**Solution**: Increase `stuck_frames_threshold` or decrease `stuck_velocity_threshold`

```python
config.stuck_frames_threshold = 200  # Wait longer (10s instead of 5s)
```

### Issue: Vehicles still getting stuck

**Cause**: Threshold not sensitive enough or teleport not working
**Solution**: Decrease `stuck_frames_threshold` or ensure spawn points are available

```python
config.stuck_frames_threshold = 50  # React faster (2.5s instead of 5s)
```

### Issue: Performance impact

**Cause**: Checking too frequently
**Solution**: Increase `stuck_check_interval_frames`

```python
config.stuck_check_interval_frames = 40  # Check every 2s instead of 1s
```

## Logging

The system logs all recovery attempts:

```
WARNING - Vehicle 234 detected as stuck at frame 1523
INFO - Recovered stuck vehicle 234 - teleported to new location
```

Check the log file in `logs/` directory for detailed information about stuck vehicle events.

## Future Enhancements

Potential improvements for future versions:

1. **Collision Detection**: Integrate with CARLA's collision sensors for more accurate detection
2. **Smart Teleportation**: Choose spawn points away from traffic to prevent immediate re-collisions
3. **Adaptive Thresholds**: Adjust thresholds based on traffic density and scenario complexity
4. **Stuck Clustering**: Detect multiple stuck vehicles in same area and handle as a group
5. **Observer Pattern**: Add a dedicated StuckVehicleObserver for real-time monitoring and alerts
