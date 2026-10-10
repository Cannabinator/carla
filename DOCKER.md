# Docker Deployment Guide

This guide explains how to run the CARLA V2X Research Platform frontend using Docker.

## Quick Start

### Using Docker Compose (Recommended)

```bash
# Build and start the container
docker-compose up -d

# View logs
docker-compose logs -f

# Stop the container
docker-compose down
```

### Using Docker directly

```bash
# Build the image
docker build -t carla-v2v-frontend .

# Run the container
docker run -d \
  --name carla-v2v-frontend \
  -p 8000:8000 \
  -v $(pwd)/logs:/app/logs \
  -v $(pwd)/data:/app/data \
  -v $(pwd)/runs:/app/runs \
  -e ADMIN_TOKEN=choose-a-secret \
  carla-v2v-frontend

# View logs
docker logs -f carla-v2v-frontend

# Stop and remove
docker stop carla-v2v-frontend && docker rm carla-v2v-frontend
```

## Accessing the Frontend

Once running, access the web interface at:

| Page | URL | Description |
|------|-----|-------------|
| **Home** | http://localhost:8000/ | Links to the pages below |
| **Admin** | http://localhost:8000/admin | Connect to CARLA, traffic, configuration, RSUs, DENM, LiDAR, runs (`ADMIN_TOKEN` required) |
| **Observer** | http://localhost:8000/observer | Map, V2X links, statistics, LiDAR of active sensors (read-only) |
| **Vehicle** | http://localhost:8000/vehicle | Join as a vehicle; sees what its radio received |
| **API docs** | http://localhost:8000/docs | OpenAPI |

The container starts without a simulation: connect it to CARLA from the admin page (the CARLA host defaults to
`CARLA_HOST`).

## Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                    Docker Container                              │
│  ┌─────────────────────────────────────────────────────────────┐ │
│  │              FastAPI Web Server (Port 8000)                 │ │
│  │  ┌─────────────┐  ┌──────────────┐  ┌─────────────────────┐ │ │
│  │  │ Admin panel │  │ Client join  │  │   REST / WebSocket  │ │ │
│  │  └─────────────┘  └──────────────┘  └─────────────────────┘ │ │
│  └─────────────────────────────────────────────────────────────┘ │
└───────────────────────────┬─────────────────────────────────────┘
                            │ Network (TCP)
                            ▼
┌─────────────────────────────────────────────────────────────────┐
│                 CARLA Server (External Machine)                  │
│  ┌─────────────────────────────────────────────────────────────┐ │
│  │              CarlaUE4.exe (Port 2000)                       │ │
│  │  - Windows/Linux host with GPU                              │ │
│  │  - Running CARLA 0.9.16                                     │ │
│  └─────────────────────────────────────────────────────────────┘ │
└─────────────────────────────────────────────────────────────────┘
```

## Configuration

### Environment Variables

| Variable | Default | Description |
|----------|---------|-------------|
| `ADMIN_TOKEN` | none, required by `docker-compose.yml` | Admin token; put it in `.env` (`.env.example`) |
| `CARLA_HOST` | `192.168.1.101` in `docker-compose.yml` | Default value of `--carla-host` |
| `CARLA_PORT` | `2000` | Default value of `--carla-port` |

### Docker Compose Configuration

Edit `docker-compose.yml` to customize:

```yaml
services:
  frontend:
    environment:
      - CARLA_HOST=your.carla.server.ip
      - CARLA_PORT=2000
    ports:
      - "8080:8000"  # Change host port if needed
```
