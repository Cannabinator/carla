# V2V Communication System Guide

## Overview

This V2V (Vehicle-to-Vehicle) communication system implements industry-standard protocols for CARLA Simulator 0.9.16, based on:
- **SAE J2735** (Basic Safety Message - BSM) for North America
- **ETSI ITS-G5** (Cooperative Awareness Message - CAM) for Europe

The system provides:
- ✅ **2 Hz update rate** (configurable, follows SAE J2735 standard)
- ✅ **BSM-based communication** with core and extended data
- ✅ **Bidirectional data sharing** for cooperative perception
- ✅ **Threat assessment** with Time-To-Collision (TTC) calculation
- ✅ **REST API** for external access to V2V data
- ✅ **One-line console output** for monitoring
- ✅ **Reproducible** with deterministic behavior

---

## Quick Start

### Basic Usage

```python
from src.v2v import V2VNetworkEnhanced, create_bsm_from_carla

# Initialize V2V network (2 Hz, 150m range)
v2v = V2VNetworkEnhanced(
    max_range=150.0,  # meters
    update_rate_hz=2.0,  # SAE J2735 standard
    world=world  # CARLA world instance
)

# Register vehicles
v2v.register(ego_vehicle.id, ego_vehicle)
v2v.register(traffic_vehicle.id, traffic_vehicle)

# In simulation loop (synchronous mode)
while running:
    world.tick()  # Advance simulation
    v2v.update()  # Update V2V network (enforces 2 Hz)
    
    # Get neighbor vehicles' BSM messages
    neighbors = v2v.get_neighbors(ego_vehicle.id)
    
    # Get threat assessment
    threats = v2v.get_threats(ego_vehicle.id)
    
    # Print one-line status
    status = v2v.get_one_line_status(ego_vehicle.id)
    print(f"\r{status}", end='')
```

### Running Example Scenario

```bash
# Activate virtual environment
source venv/bin/activate

# Run V2V scenario with API server
python src/scenarios/v2v_api_scenario.py \
    --host 192.168.1.110 \
    --api-port 8001 \
    --num-vehicles 20 \
    --v2v-range 150 \
    --duration 60 \
    --enable-coop

# Access API documentation
# http://localhost:8001/docs
```

---

## Architecture

### Components

1. **BSM Protocol** (`src/v2v/messages.py`)
   - `BSMCore`: Core safety message (SAE J2735 Part 1)
   - `BSMPartII`: Extended data (path history, emergency status)
   - `CooperativeAwarenessMessage`: European CAM standard
   - `V2VEnhancedMessage`: Combined BSM + sensor data

2. **Network Manager** (`src/v2v/network_enhanced.py`)
   - `V2VNetworkEnhanced`: Main network coordinator
   - 2 Hz update rate enforcement
   - Neighbor discovery and management
   - Threat assessment
   - Statistics tracking

3. **REST API** (`src/v2v/api.py`)
   - FastAPI-based REST endpoints
   - WebSocket for real-time updates
   - Access to BSM data, neighbors, threats

---

## BSM (Basic Safety Message)

### BSM Structure

SAE J2735 defines a two-part BSM:

#### Part I - Core Data (Transmitted every message)
```python
BSMCore(
    vehicle_id=int,           # Unique vehicle identifier
    timestamp=float,          # Message timestamp
    msg_count=int,            # Sequence counter (0-127)
    
    # Vehicle type
    vehicle_type=VehicleType,  # PASSENGER_CAR, TRUCK, BUS, etc.
    
    # Position (latitude, longitude, elevation)
    latitude=float,           # X coordinate in CARLA
    longitude=float,          # Y coordinate in CARLA
    elevation=float,          # Z coordinate in CARLA
    
    # Motion
    speed=float,              # m/s
    heading=float,            # degrees (0-360)
    steering_angle=float,     # degrees
    
    # Acceleration
    longitudinal_accel=float, # m/s²
    lateral_accel=float,      # m/s²
    vertical_accel=float,     # m/s²
    
    # Vehicle dimensions
    vehicle_length=float,     # meters
    vehicle_width=float,      # meters
    vehicle_height=float,     # meters
    
    # Brake status
    brake_status=BrakingStatus,
    brake_pressure=float,     # 0.0-1.0
    
    # Transmission
    transmission_state=str    # "drive", "reverse", "park", etc.
)
```

#### Part II - Extended Data (Optional)
```python
BSMPartII(
    path_history=List[Tuple],  # Recent positions
    emergency_status=EmergencyStatus,
    lights_status=dict,         # Headlights, turn signals, etc.
    wipers_status=dict          # Wiper state
)
```

### Creating BSM from CARLA Vehicle

```python
from src.v2v import create_bsm_from_carla

# Get fresh snapshot (CRITICAL for accurate velocity!)
world.tick()
snapshot = world.get_snapshot()
actor_snapshot = snapshot.find(vehicle.id)

# Create BSM
bsm = create_bsm_from_carla(
    vehicle=vehicle,
    vehicle_id=vehicle.id,
    msg_count=counter,
    snapshot=actor_snapshot  # Use snapshot for fresh data!
)

print(f"Speed: {bsm.speed:.1f} m/s ({bsm.speed*3.6:.1f} km/h)")
print(f"Heading: {bsm.heading:.1f}°")
print(f"Accel: {bsm.longitudinal_accel:.2f} m/s²")
```

---

## V2V Network

### Initialization

```python
from src.v2v import V2VNetworkEnhanced

v2v = V2VNetworkEnhanced(
    max_range=150.0,          # Communication range (meters)
    update_rate_hz=2.0,       # Update frequency (Hz)
    world=world,              # CARLA world instance
    enable_logging=True       # Enable debug logging
)
```

### Update Rates

SAE J2735 defines different update rates based on situation:

| Situation | Rate | Description |
|-----------|------|-------------|
| Normal driving | 2 Hz | Default transmission rate |
| Braking | 5 Hz | Increased rate during deceleration |
| Emergency | 10 Hz | Maximum rate for critical situations |

```python
# System enforces 2 Hz internally
# Call update() at any rate, it will throttle appropriately
v2v.update()  # Updates only if 0.5s elapsed since last update
```

### Vehicle Registration

```python
# Register ego vehicle
v2v.register(ego_vehicle.id, ego_vehicle)

# Register traffic vehicles
for traffic_vehicle in traffic_vehicles:
    v2v.register(traffic_vehicle.id, traffic_vehicle)

# Unregister when vehicle destroyed
v2v.unregister(vehicle.id)
```

### Getting Neighbors

```python
# Get BSM messages from vehicles within range
neighbors = v2v.get_neighbors(ego_vehicle.id)

for neighbor_bsm in neighbors:
    print(f"Neighbor {neighbor_bsm.vehicle_id}:")
    print(f"  Distance: {v2v.get_distance(ego_vehicle.id, neighbor_bsm.vehicle_id):.1f}m")
    print(f"  Speed: {neighbor_bsm.speed:.1f} m/s")
    print(f"  Heading: {neighbor_bsm.heading:.1f}°")
```

### Threat Assessment

The system calculates collision threats using Time-To-Collision (TTC):

```python
threats = v2v.get_threats(ego_vehicle.id)

for threat in threats:
    print(f"Threat from vehicle {threat['other_vehicle_id']}:")
    print(f"  Level: {threat['threat_level']}/4")
    print(f"  TTC: {threat['time_to_collision']:.1f}s")
    print(f"  Distance: {threat['distance']:.1f}m")
```

Threat levels:
- **0**: No threat (TTC > 5s or distance > 50m)
- **1**: Monitoring (3s < TTC ≤ 5s)
- **2**: Caution (2s < TTC ≤ 3s)
- **3**: Warning (1s < TTC ≤ 2s)
- **4**: Critical (TTC ≤ 1s)

### Cooperative Perception

Enable bidirectional sensor data sharing:

```python
# Enable cooperative perception for ego vehicle
v2v.enable_bidirectional_sharing(ego_vehicle.id)

# Vehicle will now share and receive sensor data when:
# 1. Another vehicle is within 50m AND
# 2. Threat level ≥ 2 (caution or higher)

# Check if cooperative perception occurred
stats = v2v.get_network_stats()
print(f"Cooperative shares: {stats['cooperative_shares']}")
```

---

## REST API

### Starting API Server

```python
from src.v2v import create_v2v_api
import uvicorn

# Create API
api = create_v2v_api(v2v_network, port=8001)

# Run server
config = uvicorn.Config(api.app, host="0.0.0.0", port=8001)
server = uvicorn.Server(config)
server.run()
```

### API Endpoints

Base URL: `http://localhost:8001`

#### GET `/vehicles`
List all vehicle IDs in network.

**Response:**
```json
[0, 1, 2, 3, 4]
```

#### GET `/vehicles/{vehicle_id}`
Get BSM data for specific vehicle.

**Response:**
```json
{
  "vehicle_id": 0,
  "timestamp": 1234567890.123,
  "msg_count": 42,
  "vehicle_type": "PASSENGER_CAR",
  "position": {"x": 100.5, "y": 50.2, "z": 0.3},
  "speed": 12.5,
  "heading": 45.0,
  "steering_angle": 10.0,
  "acceleration": {
    "longitudinal": 0.5,
    "lateral": 0.1,
    "vertical": 0.0
  },
  "dimensions": {
    "length": 4.5,
    "width": 1.8,
    "height": 1.5
  },
  "brake_status": "NO_BRAKES",
  "brake_pressure": 0.0,
  "transmission_state": "drive"
}
```

#### GET `/vehicles/{vehicle_id}/neighbors`
Get neighboring vehicles and their BSM data.

**Response:**
```json
[
  {
    "vehicle_id": 1,
    "distance": 25.5,
    "relative_speed": 3.2,
    "bsm": { /* BSM data */ }
  }
]
```

#### GET `/vehicles/{vehicle_id}/threats`
Get threat assessment for vehicle.

**Response:**
```json
[
  {
    "other_vehicle_id": 2,
    "threat_level": 2,
    "time_to_collision": 2.5,
    "distance": 30.0,
    "timestamp": 1234567890.123
  }
]
```

#### GET `/network/stats`
Get network statistics.

**Response:**
```json
{
  "total_vehicles": 20,
  "total_messages_sent": 1000,
  "average_neighbors": 3.5,
  "max_neighbors": 8,
  "cooperative_shares": 15,
  "update_rate_hz": 2.0,
  "max_range_m": 150.0
}
```

#### WebSocket `/ws/v2v`
Real-time V2V updates.

**Message Format:**
```json
{
  "timestamp": "2024-01-15T10:30:00",
  "vehicles": 20,
  "bsm_messages": [ /* Array of BSM data */ ]
}
```

### API Documentation

Interactive API docs available at:
- **Swagger UI**: `http://localhost:8001/docs`
- **ReDoc**: `http://localhost:8001/redoc`

---

## Console Output

### One-Line Status

The system provides a concise one-line status output:

```python
status = v2v.get_one_line_status(ego_vehicle.id)
print(f"\r{status}", end='')
```

**Output format:**
```
V2V: 15.3m/s (55.1km/h) | Heading:45.2° | Neighbors:3 | Threats:1(L2) | Msgs:127
```

Fields:
- **Speed**: Current speed in m/s and km/h
- **Heading**: Vehicle heading in degrees (0-360)
- **Neighbors**: Count of vehicles within range
- **Threats**: Count and highest threat level (L0-L4)
- **Msgs**: Total BSM messages sent

---

## Integration with CARLA

### Synchronous Mode Setup (Required)

V2V system requires synchronous mode for reproducibility:

```python
# Enable synchronous mode
settings = world.get_settings()
settings.synchronous_mode = True
settings.fixed_delta_seconds = 0.5  # 2 Hz simulation (matches V2V rate)
world.apply_settings(settings)

# Set random seed for reproducibility
random.seed(42)
np.random.seed(42)

# Traffic manager must also be synchronous
tm = client.get_trafficmanager(8000)
tm.set_synchronous_mode(True)
tm.set_random_device_seed(42)
```

### Getting Fresh Velocity Data (CRITICAL!)

**WRONG** - Returns stale/cached data:
```python
world.tick()
vel = vehicle.get_velocity()  # One tick behind!
```

**CORRECT** - Get snapshot immediately after tick:
```python
world.tick()
snapshot = world.get_snapshot()  # Must be RIGHT AFTER tick()
actor_snapshot = snapshot.find(vehicle.id)
vel = actor_snapshot.get_velocity()  # Fresh data!
```

### Main Loop Pattern

```python
try:
    # Setup...
    
    while running:
        # 1. Tick world (advances 0.5s in 2 Hz mode)
        world.tick()
        
        # 2. Update V2V network
        v2v.update()  # Enforces 2 Hz internally
        
        # 3. Get V2V data
        neighbors = v2v.get_neighbors(ego_vehicle.id)
        threats = v2v.get_threats(ego_vehicle.id)
        
        # 4. Your logic here
        # ...
        
        # 5. Print status
        status = v2v.get_one_line_status(ego_vehicle.id)
        print(f"\r{status}", end='')

finally:
    # Cleanup...
```

---

## Configuration

### Update Rates

```python
# 2 Hz (default, SAE J2735 normal)
v2v = V2VNetworkEnhanced(update_rate_hz=2.0)

# 5 Hz (braking scenario)
v2v = V2VNetworkEnhanced(update_rate_hz=5.0)

# 10 Hz (emergency scenario)
v2v = V2VNetworkEnhanced(update_rate_hz=10.0)
```

### Communication Range

```python
# Short range (urban, 100m)
v2v = V2VNetworkEnhanced(max_range=100.0)

# Medium range (suburban, 150m - default)
v2v = V2VNetworkEnhanced(max_range=150.0)

# Long range (highway, 300m)
v2v = V2VNetworkEnhanced(max_range=300.0)
```

### Cooperative Perception Thresholds

Edit `src/v2v/network_enhanced.py`:

```python
# Distance threshold for cooperative sharing (default: 50m)
self.coop_distance_threshold = 50.0

# Threat level threshold for cooperative sharing (default: 2)
self.coop_threat_threshold = 2
```

---

## Testing

### Unit Tests

```bash
# Test V2V network (no CARLA needed)
python -m pytest tests/v2v/ -v

# Test with coverage
python -m pytest tests/v2v/ --cov=src/v2v
```

### Integration Tests

```bash
# Run V2V scenario (needs CARLA server)
python src/scenarios/v2v_api_scenario.py --host 192.168.1.110 --duration 30
```

### Reproducibility Test

```bash
# Verify deterministic behavior
python tests/test_reproducibility.py --host 192.168.1.110
```

---

## Troubleshooting

### Zero Velocity Readings

**Problem**: `vehicle.get_velocity()` returns zero even though vehicle is moving.

**Solution**: Use snapshot method:
```python
world.tick()
snapshot = world.get_snapshot()
actor_snapshot = snapshot.find(vehicle.id)
vel = actor_snapshot.get_velocity()  # Correct!
```

### Y-Axis Mirroring

**Problem**: Left/right reversed in visualization.

**Solution**: Negate Y coordinate when rendering:
```python
positions[i * 3 + 1] = -data.points.y[i]
```

### V2V Not Updating

**Problem**: `v2v.update()` not triggering updates.

**Cause**: Called too frequently, internal throttling active.

**Solution**: Ensure 2 Hz rate:
```python
# In 2 Hz mode, update() should be called after each tick
while running:
    world.tick()  # Advances 0.5s
    v2v.update()  # Will update because 0.5s elapsed
```

### API Server Not Starting

**Problem**: Port already in use.

**Solution**: Use different port:
```bash
python src/scenarios/v2v_api_scenario.py --api-port 8002
```

---

## Standards Compliance

### SAE J2735 (North America)

The BSM implementation follows SAE J2735 standard:
- Part I: Core data (transmitted every message)
- Part II: Extended data (optional, event-driven)
- Message size: ~50-100 bytes per BSM
- Update rates: 2 Hz (normal), 5 Hz (braking), 10 Hz (emergency)

### ETSI ITS-G5 (Europe)

CAM (Cooperative Awareness Message) support:
- Similar to BSM but with European standards
- Frequency: 1-10 Hz depending on dynamics
- Range: Up to 300m

---

## Performance

### Network Overhead

- BSM size: ~100 bytes
- 20 vehicles @ 2 Hz: 4 KB/s
- 100 vehicles @ 2 Hz: 20 KB/s

### CPU Usage

- V2V update: ~1-2ms per update
- Neighbor discovery: O(n²) but optimized with spatial indexing
- Threat assessment: O(n) per vehicle

### Optimization Tips

1. **Reduce range** for dense scenarios: `max_range=100.0`
2. **Lower update rate** if real-time not critical: `update_rate_hz=1.0`
3. **Limit cooperative perception** to high-threat scenarios only

---

## Examples

### Example 1: Basic V2V Communication

```python
from src.v2v import V2VNetworkEnhanced

# Initialize
v2v = V2VNetworkEnhanced(max_range=150.0, update_rate_hz=2.0, world=world)

# Register vehicles
for vehicle in all_vehicles:
    v2v.register(vehicle.id, vehicle)

# Main loop
while True:
    world.tick()
    v2v.update()
    
    # Get neighbors
    neighbors = v2v.get_neighbors(ego_vehicle.id)
    print(f"Neighbors: {len(neighbors)}")
```

### Example 2: Collision Avoidance

```python
# Get threats
threats = v2v.get_threats(ego_vehicle.id)

for threat in threats:
    if threat['threat_level'] >= 3:  # Warning or critical
        print(f"COLLISION RISK! TTC: {threat['time_to_collision']:.1f}s")
        
        # Take evasive action
        # ... your collision avoidance logic ...
```

### Example 3: Cooperative Perception

```python
# Enable bidirectional sharing
v2v.enable_bidirectional_sharing(ego_vehicle.id)

# Share sensor data when needed
if v2v.should_share_data(ego_vehicle.id, other_vehicle_id):
    # Share LiDAR, camera, radar data
    shared_data = get_sensor_data(ego_vehicle)
    # ... transmission logic ...
```

---

## Future Enhancements

Planned features:
- [ ] Message compression (reduce bandwidth)
- [ ] Multi-hop communication (relay messages)
- [ ] V2I (Vehicle-to-Infrastructure) support
- [ ] Security (message authentication)
- [ ] Channel modeling (realistic packet loss)
- [ ] Frontend visualization of V2V data

---

## References

- **SAE J2735**: DSRC Message Set Dictionary
- **ETSI ITS-G5**: European V2V standard
- **CARLA Documentation**: https://carla.readthedocs.io/
- **Project Guide**: `/home/workstation/carla/.github/copilot-instructions.md`

---

## Contact

For issues or questions, refer to:
- Project README: `/home/workstation/carla/README.md`
- CARLA Documentation: https://carla.readthedocs.io/en/0.9.16/
