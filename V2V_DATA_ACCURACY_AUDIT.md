# V2V Protocol Data Accuracy Audit Report
**Date:** 2026-01-14  
**Author:** AI Code Auditor  
**Scope:** Scientific accuracy verification against CARLA 0.9.16 official documentation

---

## Executive Summary

**Status:** ⚠️ **CRITICAL DATA ACCURACY ISSUES FOUND**

The V2V protocol (SAE J2735 BSM) is **implemented and enabled**, but the data extraction from CARLA contains **scientifically inaccurate** methods that violate CARLA's best practices for synchronous simulation.

**Impact:** Research data collected may be **one simulation tick behind** for velocity/acceleration, leading to inaccurate threat assessment and time-to-collision (TTC) calculations.

---

## Issues Found

### 🔴 CRITICAL Issue #1: Cached Velocity Data
**File:** [src/v2v/messages.py](src/v2v/messages.py#L219)  
**Problem:** Using `vehicle.get_velocity()` which returns **CACHED data from previous tick**

**CARLA Documentation Evidence:**
```
carla.Actor.get_velocity(self)
  Returns the actor's velocity vector the client received during LAST TICK.
  The method does NOT call the simulator.
```

**Current Code (INCORRECT):**
```python
def create_bsm_from_carla(vehicle, prev_velocity=None, delta_time=0.05):
    velocity = vehicle.get_velocity()  # ❌ CACHED - one tick behind!
```

**Correct Approach:**
```python
def create_bsm_from_carla(vehicle, snapshot, prev_velocity=None, delta_time=0.05):
    actor_snapshot = snapshot.find(vehicle.id)
    velocity = actor_snapshot.get_velocity()  # ✅ FRESH snapshot data
```

**CARLA Docs Reference:**
```
carla.ActorSnapshot.get_velocity(self)
  Returns the velocity vector registered for an actor in THAT TICK.
  Return: carla.Vector3D - m/s
```

**Impact:**
- Speed calculations off by one tick (~50ms at 20 FPS)
- Acceleration calculations use mismatched velocity data
- TTC (Time To Collision) calculations scientifically invalid

---

### 🟡 MEDIUM Issue #2: Generic Steering Angle Approximation
**File:** [src/v2v/messages.py](src/v2v/messages.py#L261)  
**Problem:** Using hardcoded `70°` generic approximation instead of vehicle-specific max steering angle

**Current Code (INACCURATE):**
```python
steering_angle = control.steer * 70  # ❌ Generic approximation
```

**Correct Approach:**
```python
physics_control = vehicle.get_physics_control()
max_steer_angle = physics_control.wheels[0].max_steer_angle
steering_angle = control.steer * max_steer_angle  # ✅ Vehicle-specific
```

**CARLA Docs Reference:**
```
carla.WheelPhysicsControl
  max_steer_angle (float - degrees)
    Maximum angle in degrees that the wheel can steer.
```

**Impact:**
- Steering angle values inaccurate for different vehicle types
- BSM Part II data not representative of actual vehicle physics

---

### 🟡 MEDIUM Issue #3: Missing Lateral Acceleration
**File:** [src/v2v/messages.py](src/v2v/messages.py#L267)  
**Problem:** Hardcoded to `0.0` instead of calculated from velocity changes

**Current Code:**
```python
lateral_accel = 0.0  # ❌ Placeholder value
```

**Correct Approach:**
Calculate from velocity vector and heading changes:
```python
# Get velocity components
vel_forward = velocity.dot(forward_vector)
vel_lateral = velocity.dot(right_vector)

# Calculate lateral acceleration from centripetal force
lateral_accel = (vel_lateral**2) / turn_radius if turn_radius > 0 else 0.0
```

**Impact:**
- BSM Part II incomplete for advanced cooperative perception
- Cannot accurately model vehicle dynamics in curves

---

### 🟡 MEDIUM Issue #4: Missing Yaw Rate
**File:** [src/v2v/messages.py](src/v2v/messages.py#L269)  
**Problem:** Hardcoded to `0.0` instead of using angular velocity

**Current Code:**
```python
yaw_rate = 0.0  # ❌ Placeholder value
```

**Correct Approach:**
```python
actor_snapshot = snapshot.find(vehicle.id)
angular_velocity = actor_snapshot.get_angular_velocity()
yaw_rate = angular_velocity.z  # ✅ Z-axis rotation rate (deg/s or rad/s)
```

**CARLA Docs Reference:**
```
carla.ActorSnapshot.get_angular_velocity(self)
  Returns the angular velocity vector registered for an actor in that tick.
  Return: carla.Vector3D - rad/s
```

**Impact:**
- Cannot model vehicle rotation dynamics
- Incomplete data for trajectory prediction

---

## Data Sources Verified

### ✅ CORRECT Implementations

1. **Speed Calculation** (Line 222):
   ```python
   speed_ms = math.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2)  # ✅ Correct formula
   ```

2. **Heading Calculation** (Line 260):
   ```python
   heading = transform.rotation.yaw % 360  # ✅ Correct extraction
   ```

3. **Vehicle Dimensions** (Lines 263-265):
   ```python
   bbox = vehicle.bounding_box
   length = bbox.extent.x * 2  # ✅ Correct from bounding box
   width = bbox.extent.y * 2
   ```

4. **Acceleration Calculation** (Line 227):
   - Formula is correct IF velocity data is fresh
   - Currently fails due to Issue #1 (cached velocity)

---

## Current Scenario Configuration

### ✅ V2V Protocol Status
- **Enabled:** YES (verified at [v2v_complete_demo.py](src/scenarios/v2v_complete_demo.py#L112))
- **Update Rate:** 2 Hz (SAE J2735 standard) ✅
- **Network Class:** `V2VNetworkEnhanced` ✅

### ✅ Synchronous Mode Configuration
- **Fixed Delta:** 0.05s (20 FPS) ✅
- **Snapshot Usage:** Fresh snapshot obtained at line 326 ✅
- **Snapshot Passed to V2V:** YES (line 348) ✅

### ⚠️ Data Extraction Issues
While the scenario correctly:
1. Uses synchronous mode
2. Gets fresh snapshot each tick
3. Passes snapshot to V2V update

The **BSM creation function ignores the snapshot** and uses cached actor methods!

---

## Fix Implementation Plan

### Priority 1: Fix Velocity Data Source
**File:** [src/v2v/messages.py](src/v2v/messages.py)

**Required Changes:**
1. Modify `create_bsm_from_carla()` signature to accept `snapshot` parameter
2. Use `snapshot.find(vehicle.id).get_velocity()` instead of `vehicle.get_velocity()`
3. Use `snapshot.find(vehicle.id).get_angular_velocity()` for yaw rate
4. Use `snapshot.find(vehicle.id).get_acceleration()` if needed

**Affected Callers:**
- [src/v2v/network_enhanced.py](src/v2v/network_enhanced.py#L175) - `_create_bsm()` method
  - Already receives `snapshot` parameter ✅
  - Just needs to pass it to `create_bsm_from_carla()`

### Priority 2: Fix Steering Angle
**File:** [src/v2v/messages.py](src/v2v/messages.py#L261)

**Change:**
```python
# Get vehicle-specific max steering angle
try:
    physics_control = vehicle.get_physics_control()
    max_steer_angle = physics_control.wheels[0].max_steer_angle
    steering_angle = control.steer * max_steer_angle
except Exception:
    # Fallback to approximation if physics control unavailable
    steering_angle = control.steer * 70.0
```

### Priority 3: Implement Lateral Acceleration
**Calculation approach:**
```python
# Get vehicle orientation vectors
forward_vector = transform.rotation.get_forward_vector()
right_vector = transform.rotation.get_right_vector()

# Project velocity onto lateral axis
lateral_velocity = velocity.x * right_vector.x + velocity.y * right_vector.y

# Calculate lateral acceleration (centripetal)
# a_lat = v²/r, can approximate from steering angle and speed
if abs(control.steer) > 0.01:
    # Ackermann steering approximation
    wheelbase = 2.5  # meters (vehicle-specific, get from bbox)
    turn_radius = wheelbase / abs(math.tan(math.radians(steering_angle)))
    lateral_accel = (speed_ms ** 2) / turn_radius * math.copysign(1, control.steer)
else:
    lateral_accel = 0.0
```

### Priority 4: Implement Yaw Rate
**Direct from snapshot:**
```python
angular_velocity = actor_snapshot.get_angular_velocity()
yaw_rate = math.degrees(angular_velocity.z)  # Convert rad/s to deg/s if needed
```

---

## Testing Requirements

After implementing fixes:

1. **Unit Tests:**
   ```bash
   python -m pytest tests/v2v/test_v2v_basic.py -v
   python -m pytest tests/v2v/test_network.py -v
   ```

2. **Integration Test:**
   ```bash
   python -m pytest tests/test_v2v_lidar.py -v
   ```

3. **Data Validation:**
   - Compare velocity values between `vehicle.get_velocity()` and `snapshot.find().get_velocity()`
   - Log steering angles from different vehicle types
   - Verify acceleration calculations match expected physics

4. **Scenario Run:**
   ```bash
   ./run_v2v_lidar.sh
   # Check CSV logs for realistic values
   ```

---

## References

- **CARLA 0.9.16 Python API:** https://carla.readthedocs.io/en/latest/python_api/
- **carla.Actor Documentation:** Focus on "does NOT call the simulator" warnings
- **carla.ActorSnapshot Documentation:** Fresh data from specific tick
- **carla.WorldSnapshot Documentation:** Immutable snapshot of all actors
- **SAE J2735 BSM Standard:** Basic Safety Message format

---

## Conclusions

1. **V2V is Enabled:** ✅ Protocol correctly implemented and active
2. **Data Accuracy:** ❌ Critical issues with cached vs. fresh data
3. **Scientific Validity:** ⚠️ Current implementation may produce incorrect research results
4. **Fix Complexity:** LOW - Changes isolated to `messages.py` and one call site
5. **Backward Compatibility:** MAINTAINED - Function signature extension only

**Recommendation:** Implement Priority 1 (velocity fix) immediately. This is a **scientifically critical** issue for research data validity.
