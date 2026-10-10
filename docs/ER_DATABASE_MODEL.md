# Run data model

A recorded run is a directory `runs/<run_id>/` (see `docs/running.md` for how to make one). There is no
database: every file is plain text or a documented binary frame, written by `src/recording/recorder.py`
and described by `manifest.json` (schema version 1).

```mermaid
erDiagram
    RUN ||--o{ STATION_SNAPSHOT : "stations.jsonl"
    RUN ||--o{ V2X_LINK : "v2x_links.jsonl"
    RUN ||--o{ MESSAGE_BATCH : "messages.jsonl"
    RUN ||--o{ LIDAR_FRAME : "lidar_meta.jsonl"
    RUN ||--o{ EVENT : "events.jsonl"
    LIDAR_FRAME ||--o| RAW_LIDAR : "lidar/SENSOR/FRAME.bin"
    STATION_SNAPSHOT ||--o{ V2X_LINK : "same frame"
    V2X_LINK }o--|| MESSAGE_BATCH : "tx_id, frame"

    RUN {
        string run_id PK "manifest.json"
        string name
        string status
        int start_frame
        int end_frame
        float fixed_delta_seconds
        string git_sha
        bool git_dirty
        json seeds
        json config
        json channel
        json config_changes
        json files "bytes, lines, sha256"
    }
    STATION_SNAPSHOT {
        int frame PK
        float sim_time
        json stations "id, kind, x, y, z, heading_deg, speed_mps, tx_power_dbm"
    }
    V2X_LINK {
        int frame
        float sim_time
        int tx_id
        int rx_id
        string msg_type
        string link_type
        int msg_id
        float distance_m
        bool los
        float tx_power_dbm
        float path_loss_db
        float shadowing_db
        float rx_power_dbm
        float rx_margin_db
        float p_channel
        float p_mac
        float p_total
        bool delivered
        string drop_cause
    }
    MESSAGE_BATCH {
        int frame
        float sim_time
        int tx_id
        json messages "BSM, CAM, DENM, MAP, SPAT"
    }
    LIDAR_FRAME {
        int frame
        float sim_time
        int sensor_id
        int ego_actor_id
        int num_points
        json location
        json rotation_pyr
        bool raw_recorded
    }
    RAW_LIDAR {
        int frame
        int sensor_id
        bytes frame "LiDAR protocol v3 frame"
    }
    EVENT {
        int seq
        string kind
        int frame
        float sim_time
        float wall_time "differs between repeated runs"
        json data
    }
```

## Files

| File | One line per | Written when `record` contains | Reproducible byte for byte |
|---|---|---|---|
| `manifest.json` | run | always | no (wall-clock start/end, git state) |
| `events.jsonl` | event (run start/stop, DENM, LiDAR, config) | always | no (`wall_time`) |
| `stations.jsonl` | V2X update (2 Hz by default) | `stations` | yes |
| `v2x_links.jsonl` | evaluated sender→receiver link | `links` | yes |
| `messages.jsonl` | sender and V2X update | `messages` | yes |
| `lidar_meta.jsonl` | LiDAR frame | `lidar_meta` | yes |
| `lidar/<sensor>/<frame>.bin` | LiDAR frame | `raw_lidar` | yes (wall time is written as 0) |

Reproducible means: on the fake CARLA backend, two runs of the same scenario give identical sha256 for those
files (tested in `tests/e2e/test_scenarios.py`). On real CARLA this is **not verified**; see
`docs/science/limitations.md`, "Reproducibility of runs".

`manifest.json` holds, among others: `git.sha`/`git.dirty`, `seeds` (world, DSRC channel, penetration RNG),
the full `config` and DSRC `channel` parameters, every hot `config_changes` entry with the frame it took
effect, library versions, and `files` with bytes, line count and sha256 of each written file.

## Link record fields

`delivered` is the outcome after drawing against `p_total = p_channel × p_mac`. `drop_cause` is `null` when
delivered, otherwise `channel`, `mac` or `out_of_awareness_range`. `los` is the line-of-sight decision the channel used. Units: metres, dBm, dB, seconds.

## Previous version

Earlier revisions of this file described a relational schema (`SIMULATION_RUN`, `DSRC_CONFIG`, …) that no
code ever implemented. It was replaced by the file layout above (decision D6 of the refactor).
