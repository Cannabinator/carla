# V2V API Reference

Base URL: `http://localhost:8001`  
Interactive docs: `http://localhost:8001/docs` (Swagger UI)

All responses are JSON. Positions and coordinates use CARLA local space (metres). Timestamps are simulation seconds (`elapsed_seconds`).

---

## BSM — Basic Safety Message (SAE J2735)

### `GET /bsm`
Returns BSMs for all vehicles currently in the network.

**Response** `200 OK` — array of BSMResponse

```json
[
  {
    "vehicle_id": 0,
    "timestamp": 12.5,
    "msg_count": 4,
    "vehicle_type": "PASSENGER_CAR",
    "position": { "x": 100.0, "y": 50.0, "z": 0.2 },
    "position_accuracy": 0.5,
    "speed": 8.3,
    "heading": 90.0,
    "steering_angle": -2.1,
    "acceleration": { "longitudinal": 0.3, "lateral": 0.1, "vertical": 0.0 },
    "yaw_rate": -0.5,
    "dimensions": { "length": 4.5, "width": 1.8, "height": 1.5 },
    "brake_status": "OFF",
    "brake_pressure": 0.0,
    "transmission_state": "forward",
    "throttle_confidence": 100.0,
    "brake_confidence": 100.0,
    "steering_confidence": 100.0
  }
]
```

---

### `GET /bsm/{vehicle_id}`
Returns the BSM for a single vehicle.

| Parameter | Type | Description |
|-----------|------|-------------|
| `vehicle_id` | int | Vehicle ID assigned at registration |

**Response** `200 OK` — BSMResponse (same shape as above)  
**Error** `404` — vehicle not found

---

### `GET /vehicles`
Returns the list of all registered vehicle IDs.

**Response** `200 OK`

```json
[0, 1, 2, 3]
```

---

### `GET /vehicles/{vehicle_id}`
Returns the BSM for a vehicle (alias for `/bsm/{vehicle_id}`).

**Response** `200 OK` — BSMResponse  
**Error** `404` — vehicle not found

---

### `GET /vehicles/{vehicle_id}/neighbors`
Returns all neighbors within V2V range, with distance, relative speed, and their BSM.

**Response** `200 OK`

```json
[
  {
    "vehicle_id": 1,
    "distance": 42.7,
    "relative_speed": 1.2,
    "bsm": { ... }
  }
]
```

**Error** `404` — vehicle not found

---

### `GET /vehicles/{vehicle_id}/threats`
Returns threat assessments between this vehicle and its neighbors.

**Response** `200 OK`

```json
[
  {
    "other_vehicle_id": 2,
    "threat_level": 3,
    "time_to_collision": 3.8,
    "distance": 28.1,
    "timestamp": 12.5
  }
]
```

| `threat_level` | Meaning |
|----------------|---------|
| `0` | No threat |
| `1` | Low |
| `2` | Medium |
| `3` | High |
| `4` | Imminent collision |

**Error** `404` — vehicle not found

---

### `GET /vehicles/overview`
Returns a combined snapshot for every vehicle: BSM + neighbors + threats + enhanced metadata.

**Response** `200 OK`

```json
[
  {
    "vehicle_id": 0,
    "bsm": { ... },
    "neighbors": [ ... ],
    "threats": [ ... ],
    "enhanced": {
      "transmission_time": 12.5,
      "reception_time": 12.51,
      "link_quality": 95.0,
      "hop_count": 0,
      "priority": 0
    }
  }
]
```

---

## CAM — Cooperative Awareness Message (ETSI EN 302 637-2)

CAMs are derived live from the current BSMs. No separate store — they always reflect the latest BSM state.

### `GET /cam`
Returns a CAM for every vehicle in the network.

**Response** `200 OK`

```json
[
  {
    "station_id": 0,
    "generation_time": 12.5,
    "reference_position": { "x": 100.0, "y": 50.0, "z": 0.2 },
    "heading": 90.0,
    "speed": 8.3,
    "drive_direction": "forward",
    "vehicle_role": "default",
    "vehicle_length": 4.5,
    "vehicle_width": 1.8
  }
]
```

---

### `GET /cam/{station_id}`
Returns the CAM for a single ITS station (vehicle).

| Parameter | Type | Description |
|-----------|------|-------------|
| `station_id` | int | Same as `vehicle_id` |

**Response** `200 OK` — CAMResponse  
**Error** `404` — station not found

---

## DENM — Decentralized Environmental Notification Message (ETSI EN 302 637-3)

DENMs are hazard/event alerts. They are stored independently of the BSM cycle and can be published, updated, and cancelled at any time.

### `GET /denm`
Returns all currently active DENM alerts.

**Response** `200 OK`

```json
[
  {
    "station_id": 1,
    "action_id": 42,
    "detection_time": 200.0,
    "reference_time": 200.0,
    "event_position": { "x": 10.0, "y": 20.0, "z": 0.0 },
    "cause_code": 2,
    "cause_code_name": "ACCIDENT",
    "subcause_code": 1,
    "information_quality": 5,
    "relevance_distance": 100.0,
    "relevance_traffic_direction": "allTrafficDirections",
    "event_speed": null,
    "event_heading": null,
    "termination": null
  }
]
```

---

### `GET /denm/{station_id}`
Returns all active DENMs published by a specific station.

| Parameter | Type | Description |
|-----------|------|-------------|
| `station_id` | int | Originating ITS station ID |

**Response** `200 OK` — array of DENMResponse  
**Error** `404` — no DENMs found for that station

---

### `POST /denm`
Publishes a new DENM alert. If a DENM with the same `(station_id, action_id)` already exists it is overwritten (update / keep-alive).

**Request body**

```json
{
  "station_id": 1,
  "action_id": 42,
  "detection_time": 200.0,
  "reference_time": 200.0,
  "event_position": { "x": 10.0, "y": 20.0, "z": 0.0 },
  "cause_code": 97,
  "subcause_code": 0,
  "information_quality": 7,
  "relevance_distance": 150.0,
  "relevance_traffic_direction": "allTrafficDirections",
  "event_speed": 5.0,
  "event_heading": 90.0
}
```

| Field | Required | Default | Notes |
|-------|----------|---------|-------|
| `station_id` | ✓ | — | Originating vehicle ID |
| `action_id` | ✓ | — | Sequence number; unique per station |
| `detection_time` | ✓ | — | Simulation seconds when event was detected |
| `reference_time` | ✓ | — | Simulation seconds of this update |
| `event_position` | ✓ | — | `{x, y, z}` in CARLA local space |
| `cause_code` | | `0` | See cause code table below |
| `subcause_code` | | `0` | Standard-defined per cause_code |
| `information_quality` | | `0` | 0 (unavailable) – 7 (certain) |
| `relevance_distance` | | `150.0` | Metres |
| `relevance_traffic_direction` | | `"allTrafficDirections"` | |
| `event_speed` | | `null` | m/s at event location |
| `event_heading` | | `null` | Degrees at event location |

**Response** `201 Created` — DENMResponse  
**Error** `422` — unknown `cause_code`

---

### `DELETE /denm/{station_id}/{action_id}`
Cancels (terminates) a DENM and removes it from the active store.

| Parameter | Type | Description |
|-----------|------|-------------|
| `station_id` | int | Originating station |
| `action_id` | int | Action sequence number |

**Response** `204 No Content`  
**Error** `404` — DENM not found

---

## Network & Utility

### `GET /network/stats`
Returns aggregate network statistics.

**Response** `200 OK`

```json
{
  "total_vehicles": 4,
  "total_messages_sent": 320,
  "average_neighbors": 2.5,
  "max_neighbors": 3,
  "cooperative_shares": 12,
  "update_rate_hz": 2.0,
  "max_range_m": 150.0
}
```

---

### `GET /dashboard`
Serves the built-in V2V HTML dashboard.

---

### `WebSocket /ws/v2v`
Real-time stream of V2V state, pushed every `1 / update_rate_hz` seconds (default 0.5 s).

**Frame payload**

```json
{
  "timestamp": "2026-04-22T10:00:00.123456",
  "vehicles": 4,
  "bsm_messages": [ { ... }, { ... } ]
}
```

Connect with any WebSocket client, e.g.:

```python
import websockets, asyncio, json

async def main():
    async with websockets.connect("ws://localhost:8001/ws/v2v") as ws:
        async for msg in ws:
            print(json.loads(msg))

asyncio.run(main())
```

---

## DENM Cause Codes

| Code | Name |
|------|------|
| 0 | `RESERVED` |
| 1 | `TRAFFIC_CONDITION` |
| 2 | `ACCIDENT` |
| 3 | `ROAD_WORKS` |
| 6 | `ADVERSE_WEATHER_ADHESION` |
| 9 | `HAZARDOUS_SURFACE` |
| 10 | `OBSTACLE_ON_ROAD` |
| 11 | `ANIMAL_ON_ROAD` |
| 14 | `HUMAN_PRESENCE_ON_ROAD` |
| 17 | `WRONG_WAY_DRIVING` |
| 18 | `RESCUE_RECOVERY_IN_PROGRESS` |
| 26 | `SLOW_VEHICLE` |
| 91 | `VEHICLE_BREAKDOWN` |
| 92 | `POST_CRASH` |
| 93 | `HUMAN_PROBLEM` |
| 94 | `STATIONARY_VEHICLE` |
| 95 | `EMERGENCY_VEHICLE_APPROACHING` |
| 96 | `DANGEROUS_CURVE` |
| 97 | `COLLISION_RISK` |
| 98 | `SIGNAL_VIOLATION` |
| 99 | `DANGEROUS_END_OF_QUEUE` |

---

## Quick Reference

| Method | Path | Protocol | Description |
|--------|------|----------|-------------|
| `GET` | `/bsm` | BSM | All BSMs |
| `GET` | `/bsm/{vehicle_id}` | BSM | BSM for one vehicle |
| `GET` | `/vehicles` | BSM | List of vehicle IDs |
| `GET` | `/vehicles/{vehicle_id}` | BSM | BSM for one vehicle |
| `GET` | `/vehicles/{vehicle_id}/neighbors` | BSM | Neighbors + distance |
| `GET` | `/vehicles/{vehicle_id}/threats` | BSM | Threat assessments |
| `GET` | `/vehicles/overview` | BSM | Full per-vehicle snapshot |
| `GET` | `/cam` | CAM | All CAMs |
| `GET` | `/cam/{station_id}` | CAM | CAM for one station |
| `GET` | `/denm` | DENM | All active alerts |
| `GET` | `/denm/{station_id}` | DENM | Alerts by station |
| `POST` | `/denm` | DENM | Publish / update alert |
| `DELETE` | `/denm/{station_id}/{action_id}` | DENM | Cancel alert |
| `GET` | `/network/stats` | — | Network statistics |
| `GET` | `/dashboard` | — | HTML dashboard |
| `WS` | `/ws/v2v` | BSM | Real-time BSM stream |
