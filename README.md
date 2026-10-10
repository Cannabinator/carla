# CARLA V2X Research Platform

One server runs CARLA, simulates the V2X radio (BSM, CAM, DENM, SPAT/MAP from RSUs, pedestrian beacons) and creates
LiDAR sensors on request. Browsers and scripts are clients: an **observer** page (map, links, statistics, LiDAR), a
**vehicle** page (what one car's radio received, its LiDAR) and an **admin** page (population, configuration, RSUs,
DENM injection, recorded runs). Runs can also be scripted, seeded and recorded headless.

```
 browser / script ── HTTP + WebSocket ──► web server ──► simulation thread ──► CARLA 0.9.16
 (observer, vehicle, admin)               (FastAPI)       (owns CARLA, V2X model, LiDAR, recorder)
```

The radio model is a **simplified, uncalibrated** 802.11p-style model; read `docs/science/model.md` and
`docs/science/limitations.md` before using results.

## Quick start without CARLA

```bash
pip install -r requirements.txt            # on a machine without the CARLA wheel: numpy fastapi "uvicorn[standard]" websockets pydantic
python start_server.py --fake --port 8123 --admin-token t0k
```

Open <http://localhost:8123/observer>, `/vehicle` or `/admin` (token `t0k`). The fake world is a synthetic test double,
not a simulator.

## With CARLA 0.9.16

1. Download CARLA 0.9.16 (<https://github.com/carla-simulator/carla/releases>) and start it with
   `-carla-rpc-port=2000` (`./CarlaUE4.sh` on Linux, `CarlaUE4.exe` on Windows; open the port in the firewall when the
   web server runs on another machine).
2. `export ADMIN_TOKEN=choose-a-secret && python start_server.py --live --carla-host <carla-host>`
3. Open <http://localhost:8000/>.

Docker, tests, headless runs and the manual check against live CARLA: [`docs/running.md`](docs/running.md) and
[`DOCKER.md`](DOCKER.md).

## Headless recorded run

```bash
python -m src.cli run scenarios/crossing.yaml --fake
```

Writes `runs/<id>/` (links, stations, messages, LiDAR metadata, `manifest.json` with git state, seeds and checksums).
Layout: [`docs/ER_DATABASE_MODEL.md`](docs/ER_DATABASE_MODEL.md).

## Documentation

| | |
|---|---|
| [`docs/running.md`](docs/running.md) | install, run, test, deploy, manual live check |
| [`docs/api.md`](docs/api.md), [`docs/openapi.json`](docs/openapi.json) | REST and WebSocket reference (generated) |
| [`docs/ER_DATABASE_MODEL.md`](docs/ER_DATABASE_MODEL.md) | recorded run layout |
| [`docs/science/model.md`](docs/science/model.md) | the channel model, with machine-checked numbers |
| [`docs/science/limitations.md`](docs/science/limitations.md) | what the results do not mean |
| [`docs/science/CHANGELOG.md`](docs/science/CHANGELOG.md) | every change that alters results, with before/after numbers |
| [`AGENTS.md`](AGENTS.md) | architecture and conventions for contributors and AI agents |
| [`ACKNOWLEDGMENTS.md`](ACKNOWLEDGMENTS.md), [`LICENSE`](LICENSE) | third-party software, licence |
