# Architecture

> Agent orientation for the CARLA V2V Research Platform. Read before touching server / V2V / V2X code.

## Big picture
A **persistent CARLA world** runs in a background thread on the host. A **FastAPI web
server** (port 8000) exposes admin + client APIs and serves static HTML frontends. Clients
join over HTTP/WebSocket, each controlling exactly one "leading vehicle". The world ticks
at a fixed timestep and continuously runs a V2V/V2X message pipeline (BSM broadcast,
neighbor discovery, threat assessment, cooperative perception) plus optional V2I (RSU) and
V2P (VRU) extensions.

```
                 ┌─────────────────────────────────────────────┐
   CARLA 0.9.16  │  PersistentCARLAServer (bg thread)          │
   (RPC:2000) ◄──┤  - connect + fixed-dt tick loop             │
                 │  - spawns/despawns client leading vehicles  │
                 │  - V2VNetworkEnhanced / V2XNetworkEnhanced  │
                 └───────────────┬─────────────────────────────┘
                                 │ BSM / MAP / SPAT / VRUAM
                 ┌───────────────▼─────────────────────────────┐
                 │  FastAPI web server (port 8000)             │
                 │  admin_routes.py  (token-gated control)     │
                 │  client_routes.py (join / control vehicle)  │
                 │  v2v/api.py        (V2V REST + WebSocket)   │
                 │  visualization/web/*.html (dashboards)      │
                 └───────────────┬─────────────────────────────┘
                                 │ HTTP / WebSocket
                       Clients (laptop / raspi / workstation)
```

## Core modules

### `src/server/persistent_world.py` — the heart
`PersistentCARLAServer` (768 lines; the biggest module, recently reworked +270 lines in the
last GitLab pull). Responsibilities:
- Connect to CARLA, configure synchronous mode (`fixed_delta_seconds` = 0.05 → 20 FPS),
  Traffic Manager (port 8001), substepping.
- Background **tick loop**; thread-safe actor mutations via `_lock`.
- Per-client session → spawns exactly one leading vehicle.
- Drives the V2V/V2X network update (2 Hz) and LiDAR collectors.
- Graceful `stop()` = despawn actors + cleanup.

### `src/server/session_manager.py`
`SessionManager` + `ClientSession`. Tracks which client owns which vehicle; expiry/despawn
logic (`_check_expired_sessions`) re-acquires `_lock`.

### `src/server/admin_routes.py` & `client_routes.py`
FastAPI routers. **Admin** routes are token-gated (env `ADMIN_TOKEN`) and control world
traffic/pedestrians/scenarios. **Client** routes let a device join and steer its vehicle.

### `src/v2v/` — the V2V stack
- `messages.py` — SAE J2735-style messages: `BSMCore`, `DENM`, `VehicleType`, `BrakingStatus`,
  plus `create_cam_from_bsm()`.
- `network_enhanced.py` — `V2VNetworkEnhanced`: neighbor discovery, threat assessment
  (TTC, threat level 0–4), cooperative perception.
- `dsrc_channel.py` — `DSRCConfig` / IEEE 802.11p channel model (log-distance path loss +
  lognormal shadowing, CSMA/CA CBR, LOS/NLOS via 2-D occlusion, PRR sigmoid). Calibrated to
  ETSI EN 302 663 and Paier et al. (2008).
- `api.py` — `V2V REST API` (FastAPI): BSM/neighbor/threat endpoints + WebSocket stream.

### `src/v2x/` — V2X extension (V2I + V2P)
`V2XNetworkEnhanced` **inherits** `V2VNetworkEnhanced` (backward compatible) and adds:
- **V2I** — `rsu.py` (`RoadSideUnit`) broadcasts **MAP** (intersection geometry, cached) and
  **SPAT** (live signal phase/timing from CARLA `TrafficLight`).
- **V2P** — `VRUAM` beacons for pedestrian/cyclist actors (ETSI TS 103 300-3); `CPMMessage`
  for collective perception.

### `src/config.py` — single source of tunables
Dataclasses: `SimulationConfig`, `VisualizationConfig`, `V2VConfig`, `VehicleConfig`,
`DSRCConfig`. Export `DEFAULT_*` instances. **Edit here, not in module code.**

Other utils: `builder.py` (vehicle/world building), `carla_utils.py`, `session.py`,
`observers.py`, `potholes.py` (`PotholeManager`), `binary_protocol.py`, `data_collector.py`.

### `src/lidar/` & `src/visualization/`
- `lidar/server.py` — `run_server()` (the web server entry) and `set_persistent_world()`
  (wires the world into the web layer). `collector.py` — LiDAR data collection.
- `visualization/web/*.html` — frontend: `admin_panel.html`, `client_join.html`,
  `live_viewer.html`, `viewer.html`, `v2v_map.html`. `visualization/lidar/server.py` — LiDAR
  viewer server.

### `src/scenarios/v2v_complete_demo.py`
Self-contained demo scenario (useful as a reference for how a full V2V run is wired).

## Data flow (one tick)
1. `PersistentCARLAServer` ticks CARLA (fixed dt) → updates each leading vehicle's state.
2. V2V/V2X network samples vehicle states → builds BSMs → applies DSRC channel (range/LoS/
   PRR) → computes neighbor lists + threat assessments.
3. V2X also emits MAP/SPAT (from RSUs / traffic lights) and VRUAM (from VRUs).
4. Web layer reads network state → serves REST/WebSocket + renders HTML dashboards.
5. Client commands (steering/throttle) flow back via client routes → applied to its vehicle.
