# V2V Data Visualization & Logging Guide

## 🎯 Complete Solution Overview

Your V2V platform now includes **3 powerful ways** to visualize and analyze V2V communication data:

### 1. 🌐 **Real-Time Web Dashboard** (NEW!)
- **URL**: http://localhost:8001/dashboard
- **Live Updates**: Real-time V2V network visualization
- **Features**:
  - Network statistics (total vehicles, messages sent, avg neighbors)
  - Ego vehicle telemetry (speed, heading, position, acceleration)
  - Nearby vehicles list with distances
  - Threat assessment with time-to-collision (TTC)
  - Interactive network map showing vehicle positions and V2V connections
  - WebSocket updates every second

### 2. 📊 **Enhanced CSV Data Logging** (IMPROVED!)
- **Auto-generated**: `logs/v2v_data_YYYYMMDD_HHMMSS.csv`
- **Detailed V2V Data**:
  ```
  frame, timestamp, pos_x, pos_y, pos_z, vel_x, vel_y, vel_z,
  speed_kmh, speed_ms, yaw, pitch, roll,
  throttle, brake, steer,
  v2v_neighbors,           ← Neighbor count
  neighbor_ids,            ← NEW: Comma-separated IDs
  neighbor_distances,      ← NEW: Distances in meters
  threats,                 ← NEW: High-priority threat count
  min_ttc,                 ← NEW: Minimum time-to-collision
  bsm_heading,             ← NEW: BSM heading data
  bsm_accel,               ← NEW: BSM acceleration
  lidar_points
  ```

### 3. 🔌 **REST API** (ENHANCED!)
- **Base URL**: http://localhost:8001
- **API Docs**: http://localhost:8001/docs (Swagger UI)
- **Endpoints**:
  ```
  GET  /dashboard                     ← NEW: Web dashboard
  GET  /vehicles                      List all vehicle IDs
  GET  /vehicles/{id}                 Get BSM for vehicle
  GET  /vehicles/{id}/neighbors       Neighbors with distance/speed
  GET  /vehicles/{id}/threats         Threat assessment (TTC)
  GET  /bsm                           All BSM messages
  GET  /bsm/{id}                      Specific BSM
  GET  /network/stats                 Network statistics
  WS   /ws/v2v                        Real-time WebSocket
  ```

---

## 🚀 Usage

### Run Complete Demo with All Features

```bash
cd /home/workstation/carla
source venv/bin/activate

# Run with all visualization features enabled
python3 src/scenarios/v2v_complete_demo.py \
    --duration 120 \
    --vehicles 15 \
    --v2v-range 75 \
    --lidar-quality fast \
    --csv-logging
```

### Access Visualization Tools

**1. V2V Dashboard** (Real-time monitoring):
```
http://localhost:8001/dashboard
```

**2. LiDAR 3D Viewer** (Point cloud visualization):
```
http://localhost:8000
```

**3. REST API Documentation** (Interactive API):
```
http://localhost:8001/docs
```

**4. CSV Data** (Scientific analysis):
```
logs/v2v_data_YYYYMMDD_HHMMSS.csv
```

---

## 📈 What Each Tool Shows

### Real-Time Dashboard
Perfect for **live monitoring** during scenarios:
- See network health at a glance
- Monitor ego vehicle state
- Identify threats immediately
- Visualize V2V communication range
- Track neighbor count changes

**Use when**: Running experiments, debugging, demonstrations

### CSV Logger
Perfect for **scientific analysis**:
- Full trajectory data for all frames
- V2V neighbor relationships over time
- Threat evolution (TTC trends)
- BSM protocol data
- Import into Python/MATLAB/R for analysis

**Use when**: Publishing research, data analysis, reproducibility

### REST API
Perfect for **custom integrations**:
- Build your own visualization
- Connect external monitoring tools
- Real-time data streaming
- Automated testing

**Use when**: Custom dashboards, external tools, automation

---

## 📊 Sample Use Cases

### Research Analysis Workflow
```python
import pandas as pd
import matplotlib.pyplot as plt

# Load CSV data
df = pd.read_csv('logs/v2v_data_20260106_120000.csv')

# Analyze neighbor count over time
plt.figure(figsize=(12, 6))
plt.plot(df['frame'], df['v2v_neighbors'])
plt.xlabel('Frame')
plt.ylabel('V2V Neighbors')
plt.title('V2V Network Connectivity Over Time')
plt.savefig('v2v_connectivity.png')

# Analyze threats
threat_frames = df[df['threats'] > 0]
print(f"Threat events: {len(threat_frames)}")
print(f"Min TTC recorded: {df['min_ttc'].min():.2f}s")
```

### Real-Time Monitoring
1. Start scenario with `--csv-logging`
2. Open dashboard: http://localhost:8001/dashboard
3. Open LiDAR viewer: http://localhost:8000
4. Watch both simultaneously for comprehensive view

### API Integration Example
```python
import requests

# Get current network stats
stats = requests.get('http://localhost:8001/network/stats').json()
print(f"Total vehicles: {stats['total_vehicles']}")
print(f"Avg neighbors: {stats['average_neighbors']}")

# Get ego vehicle neighbors
neighbors = requests.get('http://localhost:8001/vehicles/0/neighbors').json()
for n in neighbors:
    print(f"Vehicle {n['vehicle_id']}: {n['distance']:.1f}m away")
```

---

## 🎨 Dashboard Features

### Network Statistics Card
- Total vehicles in simulation
- BSM messages sent
- Average neighbors per vehicle
- Maximum neighbors observed
- Update rate (2 Hz SAE J2735)
- V2V communication range

### Ego Vehicle Card
- Current speed (km/h)
- Heading (degrees)
- Position coordinates
- Longitudinal acceleration
- Active neighbor count

### Threat Assessment Card
- Real-time collision warnings
- Time-to-collision (TTC) calculations
- Threat level indicators (Low/Medium/High)
- Distance to threatening vehicles
- Color-coded severity

### Network Map
- 2D bird's-eye view
- Ego vehicle (red dot)
- Other vehicles (green dots)
- V2V connections (green lines)
- Auto-scaling based on vehicle positions

---

## 🔧 Configuration Options

### Enable/Disable Features

```bash
# Full logging and visualization
python3 src/scenarios/v2v_complete_demo.py \
    --csv-logging \
    --enable-lidar \
    --lidar-quality high

# API only (no CSV)
python3 src/scenarios/v2v_complete_demo.py \
    --no-csv-logging

# Longer duration for more data
python3 src/scenarios/v2v_complete_demo.py \
    --duration 300 \
    --csv-logging
```

---

## 📁 Output Files

After running a scenario, you'll find:

```
carla/
├── logs/
│   ├── v2v_data_20260106_120000.csv        ← Detailed V2V CSV log
│   ├── v2v_complete_demo_20260106.log      ← Scenario log file
```

---

## 🎯 Best Practices

### For Live Monitoring
1. Start scenario
2. Open dashboard (http://localhost:8001/dashboard)
3. Keep browser window visible
4. Monitor threat alerts in real-time

### For Research Data Collection
1. Use `--csv-logging` flag
2. Run longer durations (60-300 seconds)
3. Use fixed seeds for reproducibility
4. Analyze CSV in post-processing

### For Demonstrations
1. Open dashboard AND LiDAR viewer
2. Use medium vehicle count (8-15)
3. Shorter duration (30-60 seconds)
4. Screen record for presentations

---

## 🚨 Troubleshooting

### Dashboard Not Loading
- Ensure scenario is running
- Check http://localhost:8001/dashboard (not 8000)
- Verify no firewall blocking port 8001

### CSV Empty or Missing Neighbor Data
- Ensure `--csv-logging` flag is used
- Check vehicles are actually spawning
- Verify V2V range is appropriate (75m recommended)

### API Connection Refused
- Wait 2-3 seconds after starting scenario
- Check terminal for "V2V API running" message
- Try http://localhost:8001/docs to verify API is up

---

## 🎉 Summary

You now have a **complete V2V data visualization suite**:

✅ **Real-time dashboard** for live monitoring  
✅ **Enhanced CSV logging** with full V2V data  
✅ **REST API** for programmatic access  
✅ **WebSocket streaming** for live updates  
✅ **Network visualization** map  
✅ **Threat assessment** with TTC  
✅ **BSM protocol data** logging  

**Everything you need for V2V research!** 🚀
