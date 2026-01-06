# 🔧 Error Fixes Applied

## Issues Fixed

### 1. ✅ Traffic Manager Port Conflict
**Problem:** Both web server and Traffic Manager used port 8000
```
RuntimeError: trying to create rpc server for traffic manager; 
but the system failed to create because of bind error.
```

**Solution:**
- Changed Traffic Manager port from `8000` → `8001` in [src/config.py](src/config.py)
- Added retry logic in `setup_traffic_manager()` to handle port conflicts
- Falls back to `port + 1` if initial port is busy

### 2. ✅ LiDAR Sensor Cleanup Warnings
**Problem:**
```
WARNING: sensor object went out of scope but sensor is still alive
```

**Solution:**
- Updated `LiDARDataCollector.cleanup()` to properly stop sensors before destroying
- Added error handling for sensor cleanup failures
- Clear sensor dictionaries after cleanup

### 3. ✅ Web Server Always Running
**Problem:** User wanted frontend to remain accessible even if scenario fails

**Solution:**
- Moved server startup outside try-except block (runs independently)
- Added `--keep-alive` flag to keep server running after scenario ends
- Created standalone web server: `src/visualization/web/server.py`

## New Features

### Standalone Web Server
Run the frontend without CARLA:
```bash
python src/visualization/web/server.py --host 0.0.0.0 --port 8000
```

### Keep-Alive Mode
Keep web server running after scenario ends:
```bash
python src/scenarios/v2v_lidar_scenario.py --keep-alive
```

## Updated Files

1. **src/config.py**
   - Traffic Manager port: `8000` → `8001`

2. **src/utils/carla_utils.py**
   - Added port conflict retry logic
   - Falls back to alternate port if needed

3. **src/visualization/lidar/collector.py**
   - Improved sensor cleanup with error handling

4. **src/scenarios/v2v_lidar_scenario.py**
   - Server starts outside scenario try-except
   - Added `--keep-alive` flag
   - Better error messages showing server status

5. **src/visualization/web/server.py** (NEW)
   - Standalone web server for development/testing

## Port Configuration

| Service | Port | Purpose |
|---------|------|---------|
| Web Server | 8000 | Three.js viewer |
| Traffic Manager | 8001 | CARLA autopilot |
| CARLA Server | 2000 | Main simulation |

## Usage

### Run with auto-cleanup (default)
```bash
./run_v2v_lidar.sh
```

### Run with persistent web server
```bash
python src/scenarios/v2v_lidar_scenario.py --keep-alive
```

### Run only web frontend
```bash
python src/visualization/web/server.py
```

## Testing

```bash
python tests/test_v2v_lidar.py
# ✅ 15/15 tests passing
```
