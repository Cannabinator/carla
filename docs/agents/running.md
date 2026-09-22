# Running, Testing, Deploying

> Agent orientation. Commands assume you're at the repo root (`/home/workstation/carla`).

## Prerequisites
- **CARLA 0.9.16** running somewhere (see `README.md`). Start it with:
  `CarlaUE4.exe -carla-rpc-port=2000` (Windows) or the Linux equivalent.
- Python venv exists at `./venv`. There is a `PEP 668` (externally-managed) system Python,
  so **always use the venv**, not system `pip`/`python`.

## Run the web server (frontend only — no simulation)
```bash
source venv/bin/activate
python start_server.py
# → http://localhost:8000   (admin /, client /client, view /view, v2v /v2v)
```

## Run live (connects to a real CARLA server)
```bash
source venv/bin/activate
python start_server.py --live \
    --carla-host 192.168.1.110 \
    --admin-token mysecret
```
Key flags (see `start_server.py` `--help`):
- `--live` — start the persistent world on launch.
- `--carla-host` / `--carla-port` — CARLA location (default 192.168.1.110:2000).
- `--admin-token` — secret for `/admin`. **If omitted, a random token is generated and
  logged** (server isn't accidentally open). Can also be set via `ADMIN_TOKEN` env.
- `--port` — web port (default 8000). `--host` bind (default 0.0.0.0).
- `--no-v2x` — disable V2X network. `--v2x-range M` — cooperative range (default 75 m).
- `--tm-port` — Traffic Manager port (default 8001; must differ from web port).

## Environment variables
| Var | Meaning | Default |
|-----|---------|---------|
| `ADMIN_TOKEN` | Admin API token (overridden by `--admin-token`) | random if unset |
| `CARLA_HOST` | Used by docker-compose | 192.168.1.101 |
| `CARLA_PORT` | CARLA RPC port | 2000 |

## Docker (recommended for clean setup)
```bash
docker-compose up -d      # build + start (frontend only by default)
docker-compose logs -f
docker-compose down
```
- Frontend at `http://localhost:8000`.
- `docker-compose.yml` sets `ADMIN_TOKEN=201295` — **change before deploying**.
- Full guide: `DOCKER.md`. Image built from `Dockerfile`; deps from `requirements.docker.txt`
  (note: `README.md` mentions `requirements.txt` but the container uses the `.docker` variant).

## Tests
`tests/` contains:
- `tests/v2v/test_v2v_basic.py`, `tests/test_v2v_neighbor_discovery.py`
- `tests/test_v2v_lidar.py`

Run from repo root (so `src` imports resolve):
```bash
source venv/bin/activate
python -m pytest tests/ -q
```

## Useful shell scripts (repo root)
- `run_v2v_lidar.sh` — run the LiDAR/V2V scenario.
- `start_web_viewer.sh` — start the web viewer.
- `test_windows_access.sh` — Windows reachability checks.
