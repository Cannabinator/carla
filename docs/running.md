# Running, testing, deploying

Every command below was executed while writing this page, except those marked **not executed**. Run everything from
the repository root.

## Install

```bash
python -m venv venv && source venv/bin/activate
pip install -r requirements.txt              # includes the carla==0.9.16 client
pip install pytest httpx ruff pyright playwright   # development tools (or: pip install -e ".[dev]")
```

The `carla` Python package is only needed for a real simulation. The fake world, the unit/contract tests and the
browser tests run without it.

## Start a real CARLA server

Download CARLA 0.9.16 from <https://github.com/carla-simulator/carla/releases> and start it with
`./CarlaUE4.sh -carla-rpc-port=2000` (Linux) or `CarlaUE4.exe -carla-rpc-port=2000` (Windows; allow the port in the
firewall if the web server runs on another machine). **Not executed here: no CARLA server in the development container.**

## Start the platform

```bash
export ADMIN_TOKEN=choose-a-secret                   # without it a random token is generated and printed
python start_server.py --live --carla-host 192.168.1.110
```

`--live` connects to CARLA at start; without it the server starts empty and an admin connects it from `/admin`
(**not executed** against real CARLA). Pages: `/` (index), `/observer`, `/vehicle` (join), `/admin` (token), `/docs`
(OpenAPI). `python start_server.py --help` lists all options (`--v2x-rate`, `--awareness-range`, `--v2x-penetration`,
`--no-v2x`, `--tm-port`, `--port`, ...). `python -m src.cli serve <same options>` does the same.

### Without CARLA: the fake world

```bash
python start_server.py --fake --port 8123 --admin-token t0k
curl -s localhost:8123/api/status
```

The fake world is a deterministic synthetic city with ring-following vehicles and a ray-cast LiDAR
(`src/testing/fake_carla/`). It exists for frontend work and tests; it is **not a simulator** and says so in the
status (`"fake": true`) and in the run manifest. Never use its numbers as results.

## Record a run

Headless, scripted and seeded (the fake world here; drop `--fake` for CARLA):

```bash
python -m src.cli run scenarios/crossing.yaml --fake --runs-dir runs
```

Prints the run id and a line per file (rows, bytes, sha256). The directory layout is described in
`docs/ER_DATABASE_MODEL.md`. Running it twice gives identical sha256 for `stations`, `v2x_links`, `messages` and
`lidar_meta` on the fake world; on real CARLA reproducibility is **not verified** (`docs/science/limitations.md`).
A scenario file is a pydantic model, `src/scenario.py`; `scenarios/live_repro.yaml` is the one for real CARLA.

Interactive, on a running server (admin token):

```bash
curl -s -X POST -H "X-Admin-Token: t0k" -H 'content-type: application/json' \
     -d '{"name":"manual","record":["links","stations"]}' localhost:8123/api/runs
curl -s -X POST -H "X-Admin-Token: t0k" localhost:8123/api/runs/current/stop
curl -s -H "X-Admin-Token: t0k" localhost:8123/api/runs
```

The admin page has a Run card for the same. `runs/` is git-ignored.

## Tests

```bash
ruff check src tests start_server.py
pyright
python -m pytest tests -m "not carla" -q          # offline: unit, contract, golden, Node tests, browser tests (about 2 min)
node --test tests/js/*.test.mjs                   # the JavaScript unit tests alone
```

* `tests/unit`, `tests/contract`, `tests/v2x`, `tests/golden`: no CARLA, no network. Golden tests pin the channel
  and BSM outputs; regenerate only deliberately with `UPDATE_GOLDEN=1` and describe why in `docs/science/CHANGELOG.md`.
* `tests/e2e`: Playwright (Chromium, preinstalled in the CI image) against `start_server.py --fake`; skipped if
  Playwright is missing. A browser test can time out when the machine is heavily loaded; re-run it alone before
  suspecting the code.
* `-m carla` selects tests that need a live CARLA 0.9.16 (`tests/integration`, which reads `CARLA_HOST`/`CARLA_PORT`,
  and the `carla`-marked tests in `tests/server`). **Not executed here.**
* `tests/server` run against an already deployed server: `SERVER_URL=http://host:8000 ADMIN_TOKEN=... python -m pytest tests/server -v`.
  Without `SERVER_URL` they are skipped.

Regenerate the API documents after changing an endpoint (a test fails if they are stale):

```bash
python -m src.server.api_docs        # writes docs/openapi.json and docs/api.md
```

## Docker

```bash
cp .env.example .env                 # set ADMIN_TOKEN
docker-compose build
docker-compose up -d
docker-compose logs -f
docker-compose down
```

**Not executed:** there is no Docker daemon in the development container. The compose file refuses to start
without `ADMIN_TOKEN`. The image contains `src/`, `scenarios/` and `start_server.py`; the CARLA host is given with
`CARLA_HOST`/`CARLA_PORT` in `docker-compose.yml` and then connected from `/admin`. More in `DOCKER.md`.

## Manual check against live CARLA

Run this after any change that touches the simulation thread, LiDAR or the V2X pipeline, on the lab machine, with a
CARLA 0.9.16 server running. Every step should end as described; a deviation is a bug or a `TODO(verify)` to
settle (write the result in `docs/science/CHANGELOG.md`).

```bash
python start_server.py --live --carla-host <carla-host> --admin-token t0k --port 8000
```

1. `curl -s localhost:8000/api/status` shows `"connected": true`, a map name, `"fake": false`, and a `frame` that grows.
2. Open `/admin` and log in. Spawn 20 vehicles and 5 pedestrians (Population card). The status card shows
   `real_time_factor` close to 1.0; note the number.
3. Open `/observer`: stations move on the map, the links table fills at 2 Hz, the channel panel shows delivery ratios.
4. Open `/vehicle`, join. The map shows only what the vehicle's radio received, not other vehicles' true positions.
5. On the vehicle page request a LiDAR sensor. The badge goes `requested → spawning → warming_up → active` and points
   render; reconfigure it and it keeps rendering; the observer can watch the same sensor.
6. On `/admin` inject a DENM by clicking the map near the vehicle. It appears in the vehicle's inbox and in the
   observer's link log, with its channel outcome.
7. Disable an RSU in the RSU card: its SPAT stops arriving at vehicles within the next update.
8. Change the awareness range in the Configuration card: the change is applied at the frame shown in the Events list.
9. Start a run (Run card), let it run 30 s, stop it, and check `runs/<id>/manifest.json`: `status` is `finished`, file
   sizes are non-zero, `"fake": false`.
10. Close the vehicle tab: the vehicle disappears after the 30 s grace period, its LiDAR sensor is destroyed (the LiDAR
    card shows none) and CARLA has no leftover actors.
11. Reproducibility: `CARLA_HOST=<carla-host> python -m pytest tests/integration/test_reproducibility_live.py -m carla -s` prints the first
    diverging line if two runs differ; record the result in `docs/science/limitations.md`.
12. Automatic checks for the same things: `CARLA_HOST=<carla-host> python -m pytest tests/integration -m carla -v`.
