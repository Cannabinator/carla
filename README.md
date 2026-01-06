# CARLA V2V Research Platform

Lightweight V2V (Vehicle-to-Vehicle) communication framework for CARLA 0.9.16.


### Run V2V Scenario
```bash
# Unit tests (no CARLA needed)
python tests/v2v/test_network.py

# V2V communication demo
python src/scenarios/v2v_scenario.py --host 192.168.1.110
```

### Run Basic Scenario
```bash
python src/scenarios/run_scenario.py --host 192.168.1.110
```

## 🎯 V2V Framework

Lightweight implementation for efficient neighbor discovery:

### Core Components

**V2VState** - Compact vehicle state representation
```python
from src.v2v import V2VState

state = V2VState.from_vehicle(vehicle, vehicle_id=0)
# Contains: id, location, velocity, speed, timestamp
```

**V2VNetwork** - Neighbor discovery and state sharing
```python
from src.v2v import V2VNetwork

network = V2VNetwork(max_range=50.0)
network.register(0, ego_vehicle)
network.register(1, traffic_vehicle)

network.update()  # Updates states and finds neighbors
neighbors = network.get_neighbors(0)  # Get ego's neighbors
```

### Usage Example

```python
# Initialize network
v2v = V2VNetwork(max_range=50.0)

# Register vehicles
v2v.register(0, ego_vehicle)
for i, vehicle in enumerate(traffic_vehicles, 1):
    v2v.register(i, vehicle)

# In simulation loop
v2v.update()  # Call every 0.2s

neighbors = v2v.get_neighbors(0)
print(f"Ego has {len(neighbors)} neighbors")
```

## 🧪 Testing

```bash
# V2V tests (9 tests)
python tests/v2v/test_network.py -v

# Reproducibility test (requires CARLA server)
python tests/test_reproducibility.py --host 192.168.1.110
```

## 📡 V2V Scenario Features

- 50m communication range (configurable)
- Real-time neighbor discovery
- Visual indicators (green range circle, yellow connection lines)
- Spectator follows ego vehicle
- Synchronous mode for reproducibility

## 🔧 Dependencies

```bash
pip install carla==0.9.16 numpy
# Optional for visualization:
pip install open3d opencv-python
```

## 🏗️ Architecture

### Remote Setup
- **Server**: Windows @ 192.168.1.110:2000
- **Client**: Ubuntu 24.04 (this machine)
- **Mode**: Synchronous @ 20 FPS

### Key Patterns
- Lightweight dataclasses (no JSON overhead)
- Numpy for efficient distance calculations
- Minimal state storage (only what's needed)
- Update-on-demand (not every frame)

## 📝 Example Output

```
🔄 Connecting to 192.168.1.110:2000
✓ Connected to Town03

👑 Ego vehicle spawned (RED)
✓ Spawned 14 traffic vehicles
✓ Total V2V nodes: 15

🚀 Running V2V scenario for 60s...
[  20] Ego neighbors:  3 | Total vehicles: 15
[  40] Ego neighbors:  5 | Total vehicles: 15
[  60] Ego neighbors:  2 | Total vehicles: 15

✓ Scenario completed (1200 frames)
```
  --duration 120 \
  --vehicles 20 \
  --range 50.0  # Communication range in meters
```

**V2V Features:**
- 📡 Leader vehicle broadcasts to neighbors
- 📱 Vehicles respond and share sensor data
- 🎨 Visual range indicators in CARLA
- 📊 Real-time communication statistics
- 🔄 Automatic neighbor discovery

See [V2V_QUICKSTART.md](V2V_QUICKSTART.md) and [V2V_WORKFLOW.md](V2V_WORKFLOW.md) for details.

### run_scenario.py (Recommended)
Lightweight, network-optimized scenario:
- Minimal sensors by default (collision + IMU only)
- Optional cameras and LIDAR
- Best for remote Windows server setup
- Spectator camera follows vehicle

```bash
python src/scenarios/run_scenario.py \
  --host 192.168.1.110 \
  --duration 60 \
  --vehicles 30 \
  --pedestrians 20 \
  --enable-cameras \
  --enable-lidar
```

### full_sensor_scenario.py
Complete sensor suite for comprehensive data collection:
- RGB camera (800x600)
- Semantic segmentation
- LIDAR (32 channels)
- Collision detector
- IMU
- GNSS

```bash
python src/scenarios/full_sensor_scenario.py \
  --host 192.168.1.110 \
  --duration 60 \
  --no-render  # For better performance
```

## 🔬 Testing Reproducibility

Verify that scenarios produce identical results:

```bash
python tests/test_reproducibility.py --host 192.168.1.110 --runs 3 --duration 30
```

This will run the scenario 3 times and compare:
- Vehicle positions
- Collision events
- Sensor data checksums

## 📺 Visualization

### Real-time Viewer
View LIDAR point clouds and camera feeds in real-time:

```bash
python src/visualization/realtime_viewer.py --host 192.168.1.110 --duration 300
```

**Features:**
- Open3D 3D LIDAR point cloud viewer
- OpenCV camera feed display
- Interactive controls (mouse to rotate LIDAR view)
- Real-time FPS monitoring

**Options:**
- `--no-lidar` - Disable LIDAR visualization
- `--no-camera` - Disable camera visualization
- `--duration` - Set duration in seconds

## 🔧 Installation

### Install Visualization Dependencies

```bash
# For LIDAR visualization
pip install open3d

# For camera feeds
pip install opencv-python

# For data analysis
pip install matplotlib pandas
```

Or install all at once:
```bash
pip install -r requirements.txt
```

## 🌐 Network Optimization

Since your CARLA server is on a Windows machine, bandwidth matters:

**Data transmission rates:**
- No sensors: ~5 MB/s
- Cameras only: ~40 MB/s
- LIDAR only: ~30 MB/s
- All sensors: ~80 MB/s

**Tips:**
- Use `run_scenario.py` without sensors for fastest performance
- Enable only needed sensors with `--enable-cameras` or `--enable-lidar`
- Use `--no-render` flag for data collection without visualization

## 📖 Command Line Options

### Common Options (all scenarios)
- `--host` - CARLA server IP (default: 192.168.1.110)
- `--port` - CARLA server port (default: 2000)
- `--seed` - Random seed for reproducibility (default: 42)
- `--duration` - Scenario duration in seconds (default: 60)
- `--vehicles` - Number of traffic vehicles (default: 30-50)
- `--pedestrians` - Number of pedestrians (default: 20-30)

### run_scenario.py Specific
- `--enable-cameras` - Enable camera sensors
- `--enable-lidar` - Enable LIDAR sensor
- `--no-follow` - Disable spectator camera following

### full_sensor_scenario.py Specific
- `--no-render` - Disable rendering for better performance

## 🔄 Reproducibility

Running with the same `--seed` value will produce **identical** scenarios:
- Same vehicle spawn locations
- Same pedestrian paths
- Same weather conditions
- Same ego vehicle route

Change `--seed` to get different (but still reproducible) scenarios:
```bash
python src/scenarios/run_scenario.py --seed 100
python src/scenarios/run_scenario.py --seed 200
```

## 💾 Data Collection

To save sensor data, integrate the data collector:

```python
from src.utils.data_collector import DataCollector

collector = DataCollector(output_dir='./data')
# Use collector methods to save RGB, LIDAR, etc.
```

## 📝 Notes

- First run may take longer as traffic spawns
- Press `Ctrl+C` to stop early
- All actors are automatically cleaned up on exit
- World settings are restored to defaults after scenario

## 🐛 Troubleshooting

**Slow performance?**
- Reduce number of vehicles and pedestrians
- Disable sensors you don't need
- Use `--no-render` flag
- Check network bandwidth between machines

**Connection timeout?**
- Verify CARLA server is running on Windows
- Check Windows firewall allows port 2000
- Test network: `ping <windows-ip>`

**Visualization not working?**
- Install dependencies: `pip install open3d opencv-python`
- Check OpenGL support for Open3D
