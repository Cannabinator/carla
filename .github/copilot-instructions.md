# Project Guidelines

CARLA V2V research platform for CARLA 0.9.16. This workspace combines V2V SAE J2735 messaging, LiDAR streaming, and a web dashboard.

## Architecture

- `src/v2v/`: V2V protocol, network manager, REST/WebSocket API.
- `src/visualization/lidar/`: LiDAR collection, binary encoding, WebSocket streaming server.
- `src/scenarios/`: runnable scenario scripts (see `v2v_complete_demo.py` for end-to-end pattern).
- `src/utils/`: shared patterns and infrastructure (session, builder, observers, octree, binary protocol).

Primary patterns used across the codebase:
- Context manager: `CARLASession` in `src/utils/session.py` for guaranteed actor/settings cleanup.
- Builder: `ScenarioBuilder` in `src/utils/builder.py` for fluent scenario setup.
- Observer: output sinks in `src/utils/observers.py`.
- Lazy evaluation: `LazyVehicleStats` in `src/utils/lazy.py`.

## Build And Test

Use `venv` and install dependencies from `requirements.txt` for local development.

Local run:
```bash
source venv/bin/activate
python src/scenarios/v2v_complete_demo.py --carla-host 192.168.1.110 --duration 120
```

Convenience script:
```bash
./run_v2v_lidar.sh
```

Unit tests first (no CARLA required):
```bash
python -m pytest tests/v2v/ -v
python -m pytest tests/test_lidar_api.py -v
```

Integration/system tests (CARLA server required at `192.168.1.110:2000` unless overridden):
```bash
python -m pytest tests/test_v2v_lidar.py -v
python tests/test_frontend_visual.py --run
python tests/test_complete_system.py
```

Docker (frontend + services; CARLA server is external):
```bash
docker-compose build
docker-compose up -d
docker-compose logs -f
docker-compose down
```

## Conventions

- Use config dataclasses in `src/config.py`; avoid hardcoded simulation constants.
- CARLA loop is synchronous at fixed 20 FPS (`fixed_delta_seconds=0.05`).
- Do not use `time.sleep()` inside simulation loops; use `world.tick()` and frame counting.
- V2V updates are throttled to 2 Hz inside `V2VNetworkEnhanced.update()`.
- Register spawned actors in session-managed lists so cleanup is automatic.
- Prefer batched CARLA operations (`apply_batch` or `apply_batch_sync`) for actor lifecycle operations.
- Keep type hints explicit on protocol/message dataclasses (`src/v2v/messages.py`).

## Pitfalls

- Set `use_hybrid_physics=False`; hybrid mode can yield zero-velocity artifacts.
- Keep Traffic Manager and web server ports distinct (typically `8001` vs `8000`).
- In threaded server startup paths, ensure project root is on `sys.path` for `src` imports.
- LiDAR cleanup order matters: unset collector first, then cleanup collector.
- In Town10HD, skip the first 10 spawn points for road traffic scenarios.
- `docker-compose.yml` currently defaults `CARLA_HOST=192.168.1.101`, while docs/scripts often use `192.168.1.110`; align host values before running cross-host scenarios.

## Key References

- `src/scenarios/v2v_complete_demo.py`: end-to-end integration example.
- `src/v2v/network_enhanced.py`: update throttling, neighbor discovery, threat assessment.
- `src/visualization/lidar/server.py`: continuous streaming and collector integration.
- `src/utils/session.py`: canonical CARLA lifecycle handling.
- `V2V_GUIDE.md`: user-level V2V usage.
- `V2V_IMPLEMENTATION.md`: implementation details.
- `README.md`: setup and runtime overview.
