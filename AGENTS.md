# AGENTS.md

Orientation for AI agents and new contributors. Humans: start with `README.md`. Everything here was checked against
the code on the day it was written; if you find a mismatch, fix this file in the same change.

## What this is

A research platform for V2X (vehicle, roadside unit, pedestrian) communication on top of CARLA 0.9.16, with LiDAR on
request. One server owns the simulation; browsers and scripts are clients. The V2X radio is **simulated on the
server**: clients only see what their station received.

## Rules for research code (read before touching `src/v2x`)

* A wrong number is worse than no number. Do not invent citations, constants or standard section numbers; mark
  what you could not verify `TODO(verify)` and list it in `docs/science/limitations.md`.
* Separate refactors from behaviour changes (different commits). Every change that can alter results gets an entry in
  `docs/science/CHANGELOG.md` with **before/after numbers**. Golden tests (`tests/golden`) pin outputs; update them
  only on purpose (`UPDATE_GOLDEN=1`) and explain why.
* Never fabricate SPAT timing, timestamps or positions; never fall back to wall-clock time inside the model.
* Never disable TLS verification, skip or weaken a test to get green, or add a workaround for a denied permission.

## Architecture

```
 browsers / scripts ──HTTP──► FastAPI (src/server/app.py) ──► routes (admin, client, lidar, api, hub)
        ▲  └──WebSocket /ws (topic hub, ws_hub.py)                   │ submit(fn) → Future
        │                                                             ▼
        │            immutable snapshots            ┌────────── simulation thread ──────────┐
        └─────────  Published / V2XFrame  ◄─────────┤ PersistentCARLAServer (persistent_world.py)
              (hub only reads, never writes)        │ tick CARLA → V2X update (2 Hz) → LiDAR collect →
                                                    │ publish → run hooks / recorder        │
                                                    └──────────────┬────────────────────────┘
                                                                   ▼
                                                     CARLA 0.9.16  (or src/testing/fake_carla)
```

* **One thread touches CARLA.** All world changes go through `PersistentCARLAServer.submit(fn)` (a command queue
  drained between ticks, `sim_executor.py`); REST handlers wait on the returned Future (`route_utils.on_sim_thread`).
  Never call `carla` from the event loop.
* **Clients read snapshots.** `Published` (versioned, immutable) and `V2XFrame` are produced by the simulation
  thread; the hub (`ws_hub.py`, `topics.py`) delivers them latest-wins per client and topic (gaps are visible in
  `seq`). A slow client never slows the simulation.
* **Roles** (`roles.py`): `admin` (X-Admin-Token), `vehicle` (X-Session-Token of its own session), `observer` (public,
  read-only, if `observer_public`). Vehicles see only their received V2X view and their own state.
* **Configuration**: `WorldConfig` (`server/world_config.py`) is the only place for tunables; a subset is
  hot-changeable via `PATCH /api/config` (`HOT_FIELDS`, all-or-nothing, logged with the frame it took effect).
  Channel parameters: `DSRCConfig` in `src/v2x/channel/`.
* **Time**: everything in data files uses CARLA frame and simulation time. Wall-clock appears only in
  `events.jsonl` and live telemetry.

### Source map

| Path | Purpose |
|---|---|
| `start_server.py`, `src/cli.py` | entry points (`serve`, `run scenario.yaml`) |
| `src/server/` | web app, routes, hub, `persistent_world.py` (the simulation owner), `world_config.py`, `session_manager.py`, `events.py`, `api_docs.py` |
| `src/v2x/` | `network.py` (stations, update loop, DENM/RSU/VRU), `channel/` (log-distance DSRC model, per-link `LinkResult`), `messages/`, `rsu.py`, `threat.py`, `stats.py`, `frame.py` (`V2XFrame`) |
| `src/lidar/` | `sensor.py` (`LidarManager`: request/budget/ownership/state machine), `protocol.py` (wire format v3), `stream.py` (profiles), `transforms.py`, `config.py` |
| `src/recording/`, `src/scenario.py` | run recorder + manifest; scripted headless scenarios |
| `src/testing/fake_carla/` | deterministic CARLA stand-in for tests and `--fake`; not a simulator |
| `src/web/` | static pages and plain ES modules (`static/js`), vendored Three.js / uPlot, **no build step** |
| `tests/` | `unit`, `contract` (API against a fake world), `golden`, `v2x`, `e2e` (Playwright), `js` (Node), `integration` and `server` (`carla` marker / deployed server) |
| `docs/` | `running.md`, `api.md` + `openapi.json` (generated), `ER_DATABASE_MODEL.md` (run layout), `science/` (model, changelog, limitations) |
| `scenarios/` | scenario files for `python -m src.cli run` |

## Commands

```bash
ruff check src tests start_server.py && pyright                     # both must stay at zero
python -m pytest tests -m "not carla" -q                            # offline suite, about 2 minutes
python start_server.py --fake --port 8123 --admin-token t0k         # server on the fake world
python -m src.cli run scenarios/crossing.yaml --fake                # headless recorded run
python -m src.server.api_docs                                       # regenerate docs/api.md and docs/openapi.json
```

More (CARLA, Docker, the manual live check): `docs/running.md`.

## Conventions

* Absolute imports from `src` (`from src.v2x.network import V2XNetwork`); run from the repository root.
* `carla` is imported lazily or under `TYPE_CHECKING` so that everything but live code imports without it.
* Pydantic models for every request/response; every route has a summary (a test enforces it). Change an endpoint →
  regenerate the API docs (a test fails otherwise).
* Frontend: plain ES modules, no bundler, no CDN at runtime (libraries are vendored); pages render on change, not per
  frame; texts and units in one place (`units.js`).
* Tests that need a browser use `start_server.py --fake`; tests of timing use simulation frames, not sleeps, where
  possible. Browser tests can fail from machine load: re-run alone before suspecting code.
* Git: work on feature branches; do not push to `main`; open pull requests only when asked.

## Known gaps (details in `docs/science/limitations.md`)

* The channel parameters are not calibrated against measurements; the contention term is experimental.
* Reproducibility of runs on real CARLA is unverified; on the fake world it is byte-identical.
* Not yet run against a real CARLA: `tests/integration`, `tests/server` (marker `carla`), the manual script in
  `docs/running.md`, the browser frame rate with three LiDAR views.
* Standards references S15/S16 (DENM cause codes, RSU power classes) are unverified.
