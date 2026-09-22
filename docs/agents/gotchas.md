# Gotchas

> Known footguns. Check here before debugging "weird" behavior.
> For the broader scientific-accuracy review (channel model, stale audit,
> citation claims) see [`audit.md`](audit.md) — it is the tracked change backlog.

## Simulation / physics
- **`use_hybrid_physics` must stay `False`.** Enabling hybrid physics triggers a known
  "zero velocity" issue (vehicles report 0 speed). Kept off in `src/config.py`.
- **Substepping constraint:** `fixed_delta_seconds <= max_substep_delta_time * max_substeps`.
  Defaults (0.05 <= 0.01 × 10 = 0.10) satisfy this. If you raise `fixed_delta_seconds`
  above 0.10, increase `max_substeps` or physics breaks.
- **Synchronous mode + Traffic Manager:** TM uses port **8001** and must differ from the
  web port (8000). Mismatched ports cause silent no-traffic behavior.
- **CARLA seed reproducibility:** `random_seed`/`tm_seed = 42` are defaults for repeatable
  scenarios (a project goal — see `LaurinNotes.md`). Change deliberately.

## Server / runtime
- **Run from repo root.** `src.` absolute imports only resolve because `start_server.py`
  adds the root to `sys.path`. Running a `src/` module directly (e.g. `python src/...py`)
  will fail with `ModuleNotFoundError: src`.
- **venv required.** System Python is PEP 668 externally-managed; use `./venv`.
- **`requirements.txt` vs `requirements.docker.txt`:** the Docker image installs the
  `.docker` variant. If deps differ between local and container, that's why.
- **Admin token:** if you don't pass `--admin-token` (or `ADMIN_TOKEN`), the server
  generates a random one and logs it. The `/admin` panel then needs that token. The
  docker-compose hardcodes `ADMIN_TOKEN=201295` — rotate it for any real deployment.

## Concurrency
- **Actor lifecycle is lock-guarded.** `PersistentCARLAServer._lock` protects vehicle dict
  mutations. Never mutate actor lists outside the lock or from the wrong thread — despawn
  races cause CARLA "actor already destroyed" errors or leaked actors.
- **LiDAR cleanup** runs in a daemon thread after a session ends (`_cleanup_lidar_collector`).
  Don't block the tick loop waiting on it.

## V2V / V2X specifics
- **Two range concepts:** `V2VConfig.max_range` defaults to **50 m**, but `start_server.py`
  defaults `--v2x-range` to **75 m**. Be explicit which one your code path uses.
- **V2X backward compat:** `V2XNetworkEnhanced` subclasses `V2VNetworkEnhanced`. Code that
  expects V2V behavior must keep working when swapped to V2X — don't break that contract.
- **NLOS model:** DSRC path loss uses 2-D geometric occlusion for LOS/NLOS. In open maps
  everything is LOS; in towns with buildings you'll see NLOS penalties (higher loss, lower PRR).

## Git
- **Don't `git pull` from `origin` expecting team changes** — team work is on `gitlab`.
  The `developer` branch tracks `gitlab/developer`.
- Windows line-ending / access quirks are handled by `test_windows_access.sh` — run it if
  a Windows CARLA server isn't reachable.
