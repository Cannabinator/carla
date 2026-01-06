# V2V & Vehicle Movement Fixes - Summary

## Issues Fixed

### 1. ✅ V2V Neighbor Discovery Bug  
**Problem**: Ego vehicle only detected neighbors in one direction (forward in spawn list)

**Root Cause**: Algorithm used `for vid2 in vehicle_ids[i+1:]` which only checked vehicles after current vehicle in list

**Solution**: Changed to `for vid2 in vehicle_ids: if vid1 == vid2: continue` to check ALL vehicles bidirectionally

**File**: `src/v2v/network_enhanced.py` - `_discover_neighbors()` method

**Test Results**: All 7 unit tests passing (`tests/test_v2v_neighbor_discovery.py`)

---

### 2. ✅ Speed Display Shows 0 km/h
**Problem**: Vehicle speed always displayed as 0.0 km/h despite vehicle movement

**Root Cause**: Used `actor.get_velocity()` which returns cached data from previous tick

**Solution**: Get snapshot IMMEDIATELY after `world.tick()` for fresh velocity data:
```python
world.tick()  # Advance simulation
snapshot = world.get_snapshot()  # Get fresh data
actor_snapshot = snapshot.find(vehicle.id)
vel = actor_snapshot.get_velocity()  # Current tick velocity
```

**File**: `src/scenarios/v2v_scenario_perf.py`

---

### 3. ✅ Vehicles Spawning in Parking Lots and Not Moving
**Problem**: All vehicles spawned at Location(x=15.143111, y=16.681334) in parking lot with 0 km/h

**Root Cause**: Town10HD spawn points 0-9 are in parking lots, not roads. Vehicles respect traffic rules and stay parked.

**Solution**: Skip first 10 spawn points and use spawn points from index 10+ (actual roads):
```python
# CRITICAL: Skip first spawn points (parking lots in Town10HD)
road_spawn_points = session.spawn_points[10:]  # Skip first 10
random.shuffle(road_spawn_points)
```

**File**: `src/scenarios/v2v_complete_demo.py`

**Result**:
- ✅ Vehicles now spawn on roads  
- ✅ Speed: 15-30 km/h (realistic movement)
- ✅ V2V communication working (detecting 1+ neighbors)
- ✅ Position changing every frame

---

## Verified Working Scenarios

### v2v_complete_demo.py
```bash
cd /home/workstation/carla
source venv/bin/activate
python3 src/scenarios/v2v_complete_demo.py --duration 30 --vehicles 8 --v2v-range 75 --lidar-quality fast
```

**Features**:
- ✅ V2V communication with SAE J2735 BSM protocol
- ✅ LiDAR 3D visualization (http://localhost:8000)
- ✅ All architecture patterns: Context Manager, Builder, Observer, Lazy Evaluation
- ✅ Realistic traffic with moving vehicles
- ✅ Bidirectional neighbor discovery

---

## Test Suite

### Unit Tests
```bash
python tests/v2v/test_network.py -v
```
**Result**: All 7 tests passing
- ✅ Bidirectional neighbor detection
- ✅ Distance accuracy  
- ✅ Cluster scenarios
- ✅ Edge cases (single vehicle, no neighbors)

### Reproducibility Test
```bash
python tests/test_reproducibility.py --host 192.168.1.110
```

---

## Key Learnings

1. **Traffic Manager Pitfall**: `tm.ignore_lights_percentage(ego, 0)` means vehicle RESPECTS all traffic lights. If spawned at red light, will stay stopped!

2. **Snapshot Timing**: ALWAYS get snapshot immediately after `world.tick()` for fresh data. Cached data is one tick behind.

3. **Spawn Point Selection**: First 10 spawn points in Town10HD are parking lots. Skip them for road scenarios.

4. **V2V Algorithm**: Must check ALL vehicles in both directions, not just forward in list.

---

## Configuration Best Practices

### Good Traffic Settings
```python
tm.global_percentage_speed_difference(-10.0)  # 10% slower
tm.ignore_lights_percentage(ego, 70)  # Ignore most lights for better flow
tm.auto_lane_change(ego, True)
tm.vehicle_percentage_speed_difference(ego, 0.0)  # Ego at speed limit
```

### Bad Traffic Settings (causes vehicles to stop)
```python
tm.global_percentage_speed_difference(-30.0)  # Too slow!
tm.ignore_lights_percentage(ego, 0)  # Respects ALL lights, will stop!
```

---

## Next Steps

1. ✅ All critical bugs fixed
2. ✅ Scenarios running with realistic traffic
3. ✅ V2V communication working bidirectionally
4. ✅ LiDAR visualization functional

Ready for V2V research experiments! 🚀
