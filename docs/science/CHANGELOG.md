# Science / model changelog

Every change that can alter results gets an entry with before/after numbers.
Pure refactors (bit-identical outputs) are listed only as such.

## Phase 0 — baseline (2026-10-09, branch `claude/refactor-handoff`, base `29bca10`)

Pure refactor, no behaviour change:
- `carla` is imported lazily (`TYPE_CHECKING` / inside functions) in
  `src/v2v/network_enhanced.py`, `src/visualization/lidar/collector.py`,
  `src/visualization/lidar/api.py`; `src/utils/__init__.py` exports are lazy (PEP 562).
- Added `pytest.ini` with the `carla` marker.

Test baseline without `carla` installed (`pytest tests/v2v tests/v2x tests/test_v2v_neighbor_discovery.py tests/test_v2v_lidar.py tests/test_lidar_api.py -m "not carla"`):
187 passed, 8 skipped, 5 xfailed (strict).

Known-stale tests, now `xfail(strict=True)` (REFACTOR_HANDOFF §3.5):
- `tests/test_v2v_neighbor_discovery.py` (3): inject BSMs without `broadcast_all()`.
- `tests/v2v/test_v2v_basic.py::test_scalability_100_vehicles_all_neighbors`,
  `tests/v2v/test_scientific_integrity.py::test_cumulative_avg_neighbors_converges`:
  assume lossless delivery, but the CSMA term drops packets at N=100.

Golden tests added (`tests/golden/`, regenerate only deliberately with `UPDATE_GOLDEN=1`):
- `DSRCChannel`, seed 42, 5 consecutive ticks: 2-vehicle pairs (30/250/400 m), 10-vehicle line
  (NLOS blockers, with and without the NLOS model), 50- and 100-vehicle grids.
- `create_bsm_from_carla` with a mock snapshot (normal, and braking/reverse ambulance).
- These pin current behaviour including known defects (S2 yaw rate, S8 x/y in lat/lon fields).
- `test_scientific_fixes.py` (root, asserted nothing) replaced by `tests/unit/test_scientific_fixes.py`
  with real asserts; its pothole test is dropped (D1: pothole code is to be deleted).

Not yet run: modules slated for deletion (`test_complete_system`, `test_frontend*`,
`test_reproducibility`), which need a live server, Selenium or `websocket`.

## Phase 1 — repo cleanup (2026-10-10, no behaviour change in the V2X/DSRC models)

- Removed the monolithic/V2V plane: `src/scenarios`, `src/agents`, `src/lidar`, `src/web`,
  `src/visualization/lidar/{server,api}.py`, `src/v2v/{api.py,dashboard.html}`, all of `src/utils`
  except `binary_protocol.py`, six plane-A HTML pages, spare logo variants, legacy shell scripts,
  `docs/V2V_API_REFERENCE.md`, and the plane-A / Selenium / live-server tests.
- D1 (default): pothole code, README section and `scripts/lidar_pothole_detection_check.py` removed.
- New `src/server/app.py` holds the app wiring (`/`, `/admin`, `/client`, routers, `/assets`);
  `start_server.py` imports from it.
- Removed unused requirements (selenium, open3d, opencv-python, matplotlib, pandas).
- `docker-compose.yml` reads `ADMIN_TOKEN` from `.env` (`.env.example` added).
- Tests split per handoff §3.5: protocol/collector-schema cases moved to
  `tests/unit/test_lidar_protocol.py`; DENM/CAM endpoint tests, `ConnectionManager` tests and
  `LiDARStreamingAPI` tests dropped with their code.
- Test result without `carla`: 161 passed, 8 skipped, 5 xfailed.
- Not done yet from the Phase 1 list: `pyproject.toml`, ruff, pyright, pre-commit.

## Server tests (2026-10-10, no model change)

- `tests/contract/` (53 tests, no CARLA): the real FastAPI app with a `FakeWorld` that uses the
  real `SessionManager`. Pages, admin auth (missing/wrong/unconfigured token), join -> telemetry
  -> leave, kick, WebSocket telemetry/ping/expiry/disconnect cleanup, traffic/pedestrian
  endpoints, request validation. Mutation-checked: breaking auth or `leave` makes them fail.
- `tests/server/` (marker `server`, skipped without `SERVER_URL`): the same life cycle against a
  RUNNING server over real HTTP/WebSocket. Needs `SERVER_URL`, `ADMIN_TOKEN`; simulation tests
  (marker `carla`) also use `CARLA_HOST`/`CARLA_PORT` and connect the server if it is idle.
- Measured against the real app code path with a fake world: telemetry WebSocket rate 2.48 Hz
  (documented 5 Hz) - confirms handoff defect C6. Not yet verified against live CARLA.

## Phase 1 tooling (2026-10-10, no model change)

- `pyproject.toml` (deps, extras `dev`/`analysis`, ruff config), `.pre-commit-config.yaml`
  (ruff on commit, offline pytest on push), `pyrightconfig.json` (basic mode on `src/` and
  `start_server.py`).
- Ruff (`E9`,`F`) clean: removed 36 unused imports and unused variables. Rule set is
  deliberately conservative; widen during the restructuring.
- Pyright basic: 0 errors. `reportOptionalMemberAccess` is switched off in the three server files
  (42 findings from module-global/None-initialised state) until Phase 2 replaces that design.
- Test fix: `test_different_seeds_may_differ` computed `all_same` but asserted nothing, so it could
  never fail; it now asserts (seeds 1 and 999 give different delivery sets, deterministic).
- `vulture` (confidence 80): one hit, the unused `sensor_data` parameter of `V2VNetworkEnhanced.enable_bidirectional_sharing` (nothing calls it; goes away with the Phase 3 network merge). Every remaining source file is imported somewhere.
- Still open from Phase 1: live smoke test by a human (admin connect -> client join -> telemetry; `tests/server` covers it when pointed at a live server); archive tag push (HTTP 403 from this environment).

## Phase 2 - single simulation thread (2026-10-10, behaviour-preserving architecture change)

No V2X/channel model change (golden tests unchanged). Fixes handoff defects:
- C1/C2/C3/C4/C5: only the "carla-tick" thread calls CARLA after `start()`. Handlers use
  `world.submit(fn)` (`SimThreadExecutor`) and await the result off the event loop. All
  `apply_batch_sync` calls use `do_tick=False`; the one explicit tick (pedestrian controllers)
  goes through `_tick()`. Mutating methods raise if called from another thread.
- C1: `_frame` is CARLA's own frame id (previously a counter that drifted with foreign ticks).
- C10: telemetry (per frame) and the per-client V2X payload (per V2X update) are published by the
  sim thread, stamped with the snapshot frame (+ `sim_time`); readers make no RPC.
- C6: separate receive task, no double wait. Measured telemetry WebSocket rate 2.48 Hz -> 4.97 Hz
  (target 5 Hz) with the real app on a recording `carla` stub. Not yet measured on live CARLA.
- C8: spawn points / blueprints use a seeded `random.Random(config.seed)` (was unseeded global).
- C9: `WorldConfig` (pydantic, cross-field validation: substepping rule, tm_port clashes, whole-frame
  V2X rate). Defaults reproduce earlier behaviour, including `global_speed_pct = -20.0` (20 % FASTER
  than the limit) which contradicts `src/config.py` ("30 % slower"); contradiction recorded, not changed.
- C12: `/admin/connect` validates the config (422), rejects a concurrent connect (409), and reports
  `connect_state` / `connect_error` in `/admin/status`.
- C13: timezone-aware datetimes; `SessionManager` has a lock.
- Robustness (new): an exception in the V2X update or session expiry is logged and the simulation
  keeps ticking (previously any non-RuntimeError ended the tick thread and destroyed all actors).
- V2X update gate is now `frame - last >= every_n` (was `frame % 10`), so extra ticks cannot skip it.
  The second, time-based gate inside the network (defect C7) is Phase 3.
- LiDAR collector cleanup now runs on the sim thread (was a helper thread making CARLA calls); its
  internal `time.sleep(0.1)` (L9) briefly stalls the tick loop when a LiDAR session ends (Phase 4).
- Verification: stub-`carla` tests record the calling thread of every CARLA call (before: 33 of 38
  calls off-thread, 4 batches ticked implicitly, frame 6 vs real 11; after: 0, 0, equal). Mutation
  checks confirm the tests fail if `do_tick=True` or the thread guard is restored.
- NOT verified against a real CARLA server: that actors spawned by `apply_batch_sync(..., False)` are
  visible to `world.get_actor` before the next tick, and that destroy batches without a tick behave.
  TODO(verify) - run `tests/server` with CARLA_HOST/CARLA_PORT (traffic/pedestrian/join tests).

## Phase 3, step 1 - structure (2026-10-10, PURE REFACTOR: outputs bit-identical)

Proof: the golden tests (DSRCChannel seed 42, 7 scenarios x 5 ticks; BSM extraction x 2) pass unchanged.
For the BSM field rename the golden files were compared after renaming the keys: values identical.
- `src/v2v/` deleted. One `V2XNetwork` class (was `V2VNetworkEnhanced` + subclass `V2XNetworkEnhanced`):
  the subclass's `update()` wrapped the base `update()`; the base is now `_update_bsm()` and the public
  `update()` calls it, then the RSU/VRU parts. Same order of operations.
- Messages split (verbatim copy by AST range) into `v2x/messages/{bsm,cam,denm,map,spat,cpm,vruam}.py`,
  plus `v2x/extract.py` (the only CARLA-reading module), `v2x/threat.py`, `v2x/constants.py`.
  `V2VEnhancedMessage` -> `EnhancedBSMMessage`; `V2V_RANGE_*` -> `RANGE_*_M`.
- Channel moved to `v2x/channel/dsrc_logdistance.py`. Duplicate `DSRCConfig` (and the whole of
  `src/config.py`, otherwise unused) deleted; fields and defaults were identical.
- S8: BSM `latitude/longitude/elevation` -> `x_m/y_m/z_m` (they always held CARLA metres).
- Tests consolidated into `tests/v2x/` (not `tests/unit/v2x/` as the handoff suggests, to keep the
  relative paths used by several files); `v2v` removed from names.
- Behaviour fix (not a model change): the admin page sent `v2v_range`/`v2v_enabled` while the server
  reads `v2x_range`/`v2x_enabled`, so the range box was silently ignored (always 75 m, V2X on). The page
  now sends the right keys.
- Left as is: docker container/network names (`carla-v2v-*`) so existing deployments are unaffected;
  stale `docs/agents/*` and `.instructions.md` (replaced in Phase 8); `.gitignore` entries.

## Phase 3, model change S2 - BSM yaw rate (2026-10-10)

`create_bsm_from_carla` applied `math.degrees()` to `get_angular_velocity().z`, but CARLA already
returns deg/s, so `yaw_rate` was inflated by 57.3x.
- Before: input angular velocity z = 0.5 -> `yaw_rate` = 28.64788975654116
- After:  input angular velocity z = 0.5 -> `yaw_rate` = 0.5
- Golden diff: only `yaw_rate` in `bsm_from_snapshot` and `bsm_braking_reverse`; channel goldens unchanged
  (the channel does not use yaw rate). No other consumer of `yaw_rate` exists in the code.
- TODO(verify): the unit rests on the CARLA 0.9.16 API reference (deg/s); confirm with a live yaw test.

## Phase 3, model change S10 - no wall-clock / RPC fallback in BSMs (2026-10-10)

`create_bsm_from_carla` used `time.time()` and `vehicle.get_transform()/get_velocity()` as defaults
and kept them if the vehicle was missing from the snapshot, so a BSM could carry a WALL-CLOCK
timestamp next to simulation-time BSMs.
- Before: vehicle missing from snapshot -> BSM built from RPC state with `timestamp = time.time()`
  (about 1.79e9 s) while the others carry simulation seconds (e.g. 12.35).
- After: returns `None`; `V2XNetwork` skips the station for that frame, removes its previous BSM (no
  stale re-transmission) and counts it in `stats['skipped_not_in_snapshot']`. A snapshot is required.
- Normal case (vehicle in snapshot): values unchanged; goldens unchanged.
- Tests that relied on the fallback (an empty mock snapshot) now pass the actors in the snapshot.

## Phase 3, S11 - cache per-vehicle constants (2026-10-10, outputs identical)

`get_physics_control()` (a blocking server round-trip) ran for every vehicle at every V2X update; its
`max_steer_angle` never changes. It, the bounding box and the vehicle type are now read once in
`register()` (`describe_vehicle` -> `VehicleStatic`).
- Before: 1 RPC per vehicle per update (N vehicles at 2 Hz = 2N RPC/s, on the simulation thread).
- After: 1 RPC per vehicle in total. BSM values unchanged (goldens unchanged).

## Phase 3, model change S9 - BSM accelerations (2026-10-10)

Before: `longitudinal_accel` = change of the speed MAGNITUDE over the update interval (contains
lateral and vertical motion, low-passed by the 0.5 s interval); `lateral_accel` = v^2 / turning radius
estimated from the steering input and 60 % of the vehicle length (non-zero whenever the wheel is turned,
whatever the vehicle actually does).
After: both are the snapshot's acceleration vector (`ActorSnapshot.get_acceleration()`) projected on the
vehicle's forward and right axes (`v2x/geometry.py`, CARLA transform-matrix convention).
Golden numbers (same input: acceleration (1, 2, 0) m/s^2, yaw -135 deg):
- scenario 1: longitudinal 2.0 -> -2.1213, lateral 9.0336 -> -0.7071
- scenario 2: longitudinal -4.0 -> -2.1213, lateral -25.3698 -> -0.7071
  (the old values depended on speed history and steering, the new ones only on the snapshot; the
  hand calculation of the new values is cos/sin(-135 deg) = -0.7071: 1*(-0.7071)+2*(-0.7071) = -2.1213)
Removed: `prev_velocity`/`delta_time` parameters, the network's `prev_speeds` history.
`vertical_accel` stays 0.0 (not part of this change). `steering_angle` unchanged.
TODO(verify): `ActorSnapshot.get_acceleration()` units (m/s^2) and the axis convention against CARLA:
`tests/integration/test_geometry_live.py` compares `v2x/geometry.py` with `carla.Transform` (needs `carla`).

## Phase 3, model change S14 - threat assessment from velocity vectors (2026-10-10)

`calculate_threat_level` built each vehicle's velocity as `speed * (cos, sin)(heading)`, i.e. assumed
the vehicle moves where its nose points. Wrong when reversing or sliding.
- `BSMCore` gains `vx_mps`, `vy_mps` (world frame, from the snapshot velocity); the closing speed uses them.
- Example (reversing car, heading 0, vx = -5 m/s, stationary car 50 m ahead):
  before: closing 5 m/s, TTC 10 s, threat level 2; after: closing -5 m/s, TTC inf, threat level 0.
- Example (sliding car, heading 0, velocity (0, 5), stationary car 20 m to its side): before: no threat
  (moving "forward", away from it); after: TTC 4.0 s, level 3.
- Hand-made BSMs without a velocity vector (tests) fall back to speed along the heading, as before.
- Golden diff: only the two added fields (`vx_mps`=6.0, `vy_mps`=8.0 for velocity (6, 8, 0)).
- Not changed: centre-to-centre distance (extents ignored, TTC overestimated) and the thresholds
  (10 / 5 / 2 s, 100 m), now labelled PROJECT-DEFINED in the docstring (no standard cited).

## Phase 3, model change S17 - SPAT timing is never fabricated (2026-10-10)

Before: if the CARLA timing query failed, or the light was Off/Unknown, `_remaining_time` returned 5.0 s
and the SPAT carried `likely_time_s = now + 5`; every known remaining time was also wrapped in an
invented window, `min = now + remaining - 1`, `max = now + remaining + 1`.
After: `_remaining_time` returns `None` when it cannot be determined; then `likely_time_s`,
`min_end_time_s` and `max_end_time_s` are all `None` (the admin API and the client payload show null).
When known, the three values are equal: CARLA's signal program is deterministic, so there is no
uncertainty window to report.
- Example (Green, elapsed 2 s of 10 s, t = 100 s): before likely 108.0, min 107.0, max 109.0;
  after 108.0 / 108.0 / 108.0. Example (query fails): before likely 105.0; after null.
- Assumption to check on live CARLA: a light that is externally frozen or switched by a script does not
  follow its program, so the remaining time can then be wrong. TODO(verify).

## Phase 3, model change C7 + S12 - one exact update gate, simulation time only (2026-10-10)

IMPORTANT for any results produced by the persistent server before this commit.
Before: a V2X update was gated twice: the world loop (`frame % 10 == 0`) AND inside the network
(`sim_time - last_sim_time < 0.5 s` -> skip), with `sim_time` accumulated as floats (0.05 per frame).
After 10 steps the difference is 0.4999999... and the update was skipped.
- Measured by replaying the logic for 100 simulated seconds at 20 FPS, 2 Hz target:
  before: 126 of 200 updates performed (74 skipped), effective **1.26 Hz**;
  after: 200 of 200, **2.00 Hz**. (The handoff's separate replay of 7200 updates gave 23 % skipped.)
- Effect: BSM refresh, neighbour tables, threat levels and all channel statistics were computed at about
  63 % of the nominal rate, and unevenly (the skips depend on float rounding, not on the scenario).
After: the network has no internal timing decision by default; the world loop's exact integer frame
gate is the only one. An optional `update_every_n_frames` (exact frame counts) exists for standalone use.
S12: no wall clock in the model. `should_update`, `last_update_time` and the wall-clock fallback for
timestamps are removed; `measured_update_hz` is now per SIMULATED second (it is exactly the target for a
regular schedule; before it measured how fast the test or server ran). Wall-clock real-time factor
belongs to the simulation loop (reported with the recorder in Phase 7).
Config validation (already in `WorldConfig`) guarantees the update rate is a whole number of frames.

## Phase 3, channel honesty S1 / S5 / S6 (2026-10-10, no change of any computed value)

Documentation and naming only; the channel computes exactly what it did.
- New `docs/science/model.md`: every equation and parameter, each source marked unverified, and tables of
  mean rx power and delivery probability per distance. `tests/unit/test_channel_model_doc.py` recomputes the
  tables from the code (P_rx within 0.01 dB, E[p_channel] within 0.001, p_csma within 1e-4), so the document
  cannot drift again. `docs/science/limitations.md` lists known limitations.
- S1: the documented calibration (-71 / -77 / -87 dBm; "meaningful PRR variation within 10-150 m") was
  numerically false; the code gives -56.9 / -63.9 / -78.5 dBm. Comments in the code were replaced by pointers
  to `model.md`; `docs/agents/audit.md` carries a SUPERSEDED banner (it claimed "P2 RESOLVED").
- S5: `snr_margin` -> `rx_margin_db` (it is power above sensitivity, not an SNR), stat key
  `avg_snr_margin_db` -> `avg_rx_margin_db` (golden values compared after the key rename: identical).
  The double randomisation (sampled margin -> logistic -> Bernoulli) is documented, not changed.
- S6: the CSMA term is labelled EXPERIMENTAL / UNCALIBRATED; its docstring table contradicted the code and
  is replaced by the values the code produces.
- A check made while writing this: Gauss-Hermite quadrature gave 0.7933 for E[p_channel] at 150 m NLOS and was
  WRONG (logistic functions converge badly under it). Fine-grid integration (0.7905), a 20-million-sample
  Monte Carlo (0.7905) and the code path (0.7904) agree; the table uses the grid value.
- Not done: recalibration (decision D5: keep parameters, fix documentation), correlated shadowing, a real MAC
  model, a decision threshold instead of the logistic.

## Phase 3, model change S3 - background traffic can be V2X stations (decision D2, 2026-10-10)

IMPORTANT: this changes what the persistent server simulates by default.
Before: only the joined client vehicles were V2X stations. Background traffic (and its autopilot
neighbours) never transmitted, so the channel saw only the clients: contender count N = number of clients
(typically 1-5), neighbour tables existed only between clients, and the NLOS blocker set was the clients.
After: each spawned background vehicle becomes a station with probability `v2x_penetration_rate`
(default **1.0**, i.e. all equipped; `0.0` reproduces the old behaviour). The choice uses a seeded stream
of its own (`seed + 1`), so changing the rate does not change which vehicles spawn. Clients are always
stations. Option: `WorldConfig.v2x_penetration_rate`, `/admin/connect` field `v2x_penetration_rate`,
CLI `--v2x-penetration`, status fields `v2x_stations`, `v2x_penetration_rate`.
Effect on the contention factor (model.md, `p_csma(N)`), N = stations within 300 m of the transmitter:
- 1 client + 0 equipped background: N = 1 -> 1.000 (before)
- 1 client + 25 equipped background: N = 26 -> 0.882
- 1 client + 50 equipped background: N = 51 -> 0.779
- 1 client + 100 equipped background: N = 101 -> 0.607
(the contention term is experimental, see model.md). Also: more stations means more BSMs per update, more
neighbours, more NLOS blockers, and roughly N^2 channel evaluations per update (cost grows quadratically).
Not changed: pedestrians remain VRUs registered as before (no penetration rate for them yet).

## Phase 3, performance of the channel (needed by S3), outputs identical (2026-10-10)

With every background vehicle a station (S3 default), the NLOS blocker search (for each pair, a Python
loop over all other stations) made one V2X update cost O(N^3) interpreted steps on the simulation thread.
Measured `DSRCChannel.broadcast_all` (random stations in a 400 m square, one update):
| stations | before | after |
|---|---|---|
| 10 | 0.8 ms | 0.8 ms |
| 25 | 7.6 ms | 1.8 ms |
| 50 | 48.5 ms | 7.4 ms |
| 100 | 311 ms | 34 ms |
| 150 | 925 ms | 126 ms |
| 300 | not measured | 1315 ms |
(One tick is 50 ms and an update runs every 10th tick; before, 100 stations stalled the loop for most of
each 500 ms window.) The blocker search is now vectorised with numpy per transmitter (`_nlos_row`), same
arithmetic and order as the scalar `_point_to_segment_dist`. Proof of identical output: channel goldens
unchanged, and `tests/unit/test_nlos_vectorised.py` compares it with the scalar `_is_nlos` on 40 random
layouts, 10 collinear/grid layouts and boundary/duplicate-position cases. The scalar function stays as the
reference. Limit: beyond about 150 stations the remaining per-pair Python loop still dominates
(300 stations: 1.3 s per update); a server with that many stations needs a lower update rate, a lower
penetration rate, or a further rewrite of `broadcast_all`.
