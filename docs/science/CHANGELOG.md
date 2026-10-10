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

## Phase 3, S13 - one meaning for "range" (2026-10-10, same numbers by default)

The code had five unrelated "range" values (library default 150 m, server/CLI/API default 75 m, the
deleted config's 50 m, RSU 300 m, VRU 100 m) and the one called `max_range` / `v2x_range` was an
APPLICATION-LAYER cutoff applied after the radio model, which hid that fact.
- Renamed to `awareness_range_m` everywhere: `V2XNetwork(awareness_range_m=...)`,
  `WorldConfig.awareness_range_m`, `/admin/connect` field, status field, admin page, CLI
  `--awareness-range` (was `--v2x-range`) and new `--no-awareness-limit`.
- `None` now means "no cutoff: the radio model alone decides who is a neighbour". With default radio
  parameters this makes line-of-sight neighbours out to several hundred metres (model.md).
- Defaults unchanged (library 150 m, server 75 m); both are now documented as such.
- RSU coverage (`RSUConfig.broadcast_range_m`, 300 m) and the 100 m VRU cutoff in the client payload are
  separate settings, not yet radio-modelled (S4).
- Also fixed: a nested f-string in `start_server.py` was invalid on Python 3.10 (the Docker image).

## Phase 3, S18 / D8 - V2X rate: attribution corrected and configurable (2026-10-10, default unchanged)

The code said the 2 Hz BSM rate follows "SAE J2735 §7.1 minimum beacon interval". J2735 is a message
dictionary and does not set transmission rates; BSM rates are specified elsewhere (SAE J2945/1, nominally
10 Hz, and ETSI EN 302 637-2 for CAMs, 1-10 Hz and event-triggered). Both attributions are unverified here,
so the docstrings now say: 2 Hz is this project's default.
- The rate is now an option: `/admin/connect` field `v2x_update_rate_hz`, CLI `--v2x-rate`. It must divide
  the 20 Hz simulation rate into a whole number of frames (2, 4, 5, 10, 20 Hz...); otherwise `/admin/connect`
  answers 422 (3 Hz would be every 6.67 frames and could not be scheduled exactly, see C7).
- Section numbers cited for MAP/SPAT (J2735 "§7.4/§7.18") and the RSU 33 dBm / "ETSI TR 101 607" attribution are
  marked unverified (S16 stays open: needs the documents).
- Not done: ETSI CAM generation rules (event-triggered rate) as an optional mode.

## Phase 3, S7 - RSUs per intersection (2026-10-10)

Before: one RSU per traffic-light ACTOR, keyed by `id(tl) % 10000` (a Python object id, not a stable
identity), only the first 10 lights in arbitrary order got an RSU, and every SPAT therefore had exactly
one signal group.
After: lights are grouped with CARLA's own `TrafficLight.get_group_traffic_lights()` (transitively merged,
so asymmetric answers cannot split an intersection; a failed query makes a light its own group), one RSU
per group at the mean position of its lights, groups ordered by smallest actor id, lights inside a group by
actor id (signal group id = index). All intersections get an RSU by default; `rsu_max_count` (config,
`/admin/connect`) caps it. Intersection ids are 1..n and stable between runs.
- Effect: SPAT messages now carry all signal groups of an intersection; the number of RSUs equals the
  number of light groups of the map (not 10).
- TODO(verify) on live CARLA: that `get_group_traffic_lights()` returns the lights of one physical
  intersection (and includes the light itself). `tests/server::test_rsus_are_per_intersection...` prints
  the counts for your map and fails if no intersection has more than one signal group.
- RSU coverage radius (300 m) and TX power (33 dBm) are unchanged and still not radio-modelled (S4).

## Phase 3, S4 step 1 - per-link records from the channel (2026-10-10, deliveries identical)

New `LinkResult` (immutable NamedTuple, `v2x/channel/base.py`): frame, sim_time, tx_id, rx_id, msg_type,
link_type, distance_m, los, tx_power_dbm, path_loss_db, shadowing_db, rx_power_dbm, rx_margin_db,
p_channel, p_mac, p_total, delivered, drop_cause. `DSRCChannel.evaluate_link(...)` decides one attempt;
`broadcast_all` is now a loop over it and returns the list (also `last_links`).
- Deliveries are unchanged: same random draws in the same order (one gaussian, then one uniform), the
  channel goldens pass untouched.
- `drop_cause`: with one uniform draw u, a loss is "channel" if u >= p_channel, and "mac" if
  p_total <= u < p_channel (the radio would have delivered it, contention cost it). "out_of_awareness_range"
  is set by the network when the application filter removes a radio-delivered packet.
- Property tests: with a fixed seed, +10 dB TX power gives +10.000 dB rx power and identical path loss and
  shadowing; the path-loss exponent moves path loss by exactly 10*dn*log10(d/d0); doubling sigma doubles the
  logged shadowing sample; sensitivity moves the margin one-to-one.
- Counters: `stats` (legacy) still counts V2V links only; new `stats_by_link_type`.
- Cost of recording: `broadcast_all` for 50 / 100 / 150 stations is 16 / 72 / 223 ms with records
  (14 / 34 / 126 ms in the entry above, which had no records). Still far below the 311 / 925 ms before the
  vectorisation, but 100 stations now takes more than one 50 ms frame per V2X update.
- Next steps (not done yet): V2XFrame, RSU/VRU/DENM messages through the channel with per-station power.

## Phase 3, S4 step 2 - the V2XFrame (2026-10-10, no change of any computed value)

`V2XNetwork.update()` now ends by building an immutable `V2XFrame` (`get_frame()`, `v2x/frame.py`):
frame number and simulation time (from the snapshot), ground-truth `stations` (kind OBU/RSU/VRU; RSU ids are
`-intersection_id`), `sent` (what each station transmitted), `inbox` (what each receiver actually got, with the
link that delivered it), every `LinkResult` of the update, `threats` computed from the receiver's inbox, and
per-frame `ChannelStats` (by link type: total, delivered, channel/mac/awareness drops; stations skipped).
- Ground truth and the received view are separate: a station can be in `stations` and absent from a
  receiver's `inbox`.
- The application's awareness filter is recorded as the drop cause "out_of_awareness_range" on the
  radio-delivered link (it used to be invisible); inbox == neighbour tables is tested.
- Deviation from the handoff's sketch: `inbox` maps a receiver to a TUPLE of `ReceivedMessage` (a transmitter
  can send several message types), not to one message per transmitter.
- Frames are built from the same state as before; neighbours, threats and statistics are unchanged.

## Phase 3, S4 steps 3-4 - RSU and pedestrian messages through the radio model (2026-10-10)

IMPORTANT: changes what vehicles "know" about signals and pedestrians.
Before: a vehicle "received" an RSU's SPAT if it was within the RSU's 300 m radius, and a pedestrian's VRUAM
if within 100 m: a geometric test, always 100 % within range, regardless of buildings/blockers, transmit power or
channel (`RSUConfig.tx_power_dbm` was never used). Only vehicle BSMs used the radio model.
After: RSU MAP and SPAT (link type I2V, 33 dBm from `RSUConfig`) and pedestrian VRUAM (P2V, 23 dBm,
`vru_tx_power_dbm`) are separate transmission attempts evaluated by the same `evaluate_link` as BSMs: path
loss, one shadowing draw, LOS/NLOS from vehicles on the segment, the contention factor, one delivery draw.
The coverage radii (RSU 300 m, pedestrian 100 m) are kept as hard limits: no attempt is evaluated beyond them.
Each delivered message lands in the receiver's inbox (`V2XNetwork.get_received`, `V2XFrame.inbox`); the client
payload (`spat_nearby`, `vru_nearby`) now lists only messages that were received, with `rx_power_dbm`, and
carries `v2x_frame` / `v2x_sim_time`.
Expected delivery probability per attempt (shadowing averaged, 1 contender), from the model of docs/science/model.md:
| link | LOS | NLOS |
|---|---|---|
| RSU 33 dBm, 100 m | 1.0000 | - |
| RSU 33 dBm, 150 m | - | 0.9798 |
| RSU 33 dBm, 300 m | 0.9999 | 0.8034 |
| VRU 23 dBm, 50 m | 1.0000 | 0.9966 |
| VRU 23 dBm, 100 m | 0.9999 | 0.9342 |
So behind a blocker at the edge of coverage an RSU message is lost about one time in five, where it was never
lost before; in line of sight almost nothing changes.
- Reproducibility: infrastructure and pedestrian links use their own seeded streams (seed + 101 / + 102):
  adding RSUs or pedestrians does not change a single vehicle-to-vehicle outcome (tested).
- Assumption without a source: the pedestrian device transmits at 23 dBm (the OBU value).
- Not yet done: DENM through the channel (next step), antenna height (recorded nowhere; the log-distance model
  has no height term), SPAT/MAP/VRUAM `NLOS` blockers are other V2X vehicles only (as for BSMs).

## Phase 3, S4 step 5 - DENM through the radio model (2026-10-10)

Before: DENMs sat in an in-memory store that nothing delivered (only tests read it).
After: every active DENM is transmitted at each V2X update by its originating station: the vehicle
(`station_id` = actor id, V2V link, OBU power), an RSU (`station_id` = -intersection_id, I2V link, the RSU's
`tx_power_dbm`), or - if the id is neither - a fixed roadside transmitter at the event position (I2V link, OBU
power). Receivers are the vehicles within `relevance_distance` of the EVENT (the relevance area), except the
originator; each is a separate attempt with the usual path loss, shadowing, NLOS and contention, using the
DENM's own seeded stream (seed + 103), so DENM traffic changes no BSM outcome (tested). Several DENMs of one
station are told apart by `LinkResult.msg_id` (= action_id). Cancelled DENMs stop being transmitted.
Received DENMs are in the receiver's inbox and in the client payload (`denm_received`: cause, distance to the
event, rx power).
- Decisions without a source: relevance area is a circle around the event position; the transmitter for an
  unregistered originator is placed at the event; DENMs are repeated every V2X update (no ETSI repetition /
  validity-duration handling).
- Still missing: an admin API/UI to inject a DENM (handoff F10, decision D11: Phase 5/6).

## Phase 3 - cost of a complete V2X update (measured at the end of S4, 2026-10-10)

`V2XNetwork.update()` on mock vehicles (no CARLA RPC), awareness 75 m, default channel, one call:
| vehicles | RSUs | pedestrians | links (V2V / I2V / P2V) | update |
|---|---|---|---|---|
| 25 | 5 | 10 | 600 / 136 / 20 | 14 ms |
| 50 | 10 | 20 | 2450 / 498 / 84 | 48 ms |
| 100 | 20 | 40 | 9900 / 1906 / 271 | 136 ms |
| 100 | 40 | 100 | 9900 / 4214 / 783 | 153 ms |
A tick is 50 ms and an update runs every 10th tick on the simulation thread, so with about 100 stations the
loop falls behind real time for a couple of frames at every update. Levers: `v2x_penetration_rate`,
`v2x_update_rate_hz`, `awareness_range_m`, `rsu_max_count`. Real CARLA adds per-vehicle calls
(`get_control`) on top. Measure on the real server before relying on 100+ stations in real-time mode.

## Phase 4 - LiDAR on request (2026-10-10)

Before: a LiDAR sensor could only be created at join, with hard-coded settings, and it was never switched off;
the per-session collector transformed points with a hand-rolled matrix, kept every 2nd point (stride), stamped
frames with the wall clock, and read whichever measurement had arrived last. After: sensors exist only while
requested, are configured per request, are collected frame-synchronously and are streamed in protocol v3.
(`src/lidar/`, `src/server/lidar_routes.py`; handoff defects L1-L9.)

- **L1 transform** (`src/lidar/transforms.py`): the old third matrix row was `[-sin p, cos p sin r, cos p cos r]`;
  CARLA's own basis (forward/right/up vectors, tested against analytic values; a live test against the bounding box of a real
  vehicle is written but not yet run) has `[sin p, -cos p sin r, cos p cos r]`. Error of the old matrix, 50 m ahead of the sensor:
  yaw 30 / pitch 0 / roll 0: 0.00 m; pitch 5: 8.72 m; pitch 10 + roll 5: 17.36 m; 20 m to the side at pitch 10 +
  roll 5: 3.43 m. On flat ground nothing changed, on slopes every point cloud was wrong.
- **L2/L8 protocol v3** (`src/lidar/protocol.py`, JS decoder `src/web/static/js/lidar-decoder.js`): header carries
  CARLA frame id, simulation time, wall time, sensor id, ego actor id, field mask, and says in which frame
  (sensor/world) the points are; the sensor pose is sent as 6 doubles (the old one was 6 float32, and `tag`/`object_idx` were float32 per point; v3 uses u8/u32, 21 B per point against 20 B). The
  JS decoder is tested against bytes produced by the Python encoder (Node 22).
- **L3 frame synchronisation** (`LidarManager.collect`): after `world.tick()` returned frame F only the
  measurement with `frame == F` is used; older ones are discarded, a missing one is a counted miss (40 in a row
  put the sensor in `error`). A sensor spawned at that boundary is not waited for (found with the CARLA stub: it
  would have stalled the simulation thread for `frame_timeout_s` = 0.5 s).
- **L4/L5 runtime control, budget, ownership**: states `requested -> spawning -> warming_up -> active`,
  `reconfiguring`, `stopping -> destroyed`, `error`, each transition logged with its frame. Over budget:
  `409 {"error": "lidar_budget_exceeded", "requested_pps", "available_pps", ...}` (`lidar_max_sensors` = 4,
  `lidar_max_points_per_second` = 2 000 000 by default: unmeasured, tune on the real server). A sensor without
  viewers is released after `lidar_keep_alive_s` (10 s, wall clock) unless pinned or recording raw. Vehicles can
  create sensors only on their own actor, observers only view, the admin anything (D9). Sensors are destroyed
  before the vehicle they are attached to.
- **Display subsampling** (`src/lidar/stream.py`): was every 2nd point of the array (CARLA orders points by
  channel, so a stride keeps whole rings and drops others); now a fixed seeded random choice per sensor, order
  preserved, and only for streaming, never for recordings. Profiles are quantised (10k / 25k / 50k / all points,
  `max_hz` to a divisor of the simulation rate) so a frame is encoded once per profile and every viewer on it
  gets the same bytes (tested).
- Cost of encoding one frame on synthetic random data (a dev machine, not the target): 25 000 points, 21 B each:
  0.3 ms uncompressed / 512 kB, 18 ms with zlib level 1 / 429 kB (random data compresses poorly; real clouds
  should do better, measure). 100 000 points: 1.4 ms / 2.0 MB, 78 ms with zlib. At the default 500 000 points per
  second and 20 Hz one unsubsampled viewer is about 10 MB/s; the default WebSocket profile is therefore the full
  stream (explicit and unsurprising) and UIs should ask for less: the join page asks for 10 000 points at 1 Hz.
- Removed: `src/visualization/lidar/collector.py`, `src/utils/binary_protocol.py` (v2: 28-byte header + 24-byte pose) and their tests; the cases that still mattered (rotation, round trip, header size) live in
  the new tests.
- Not verified on a real CARLA yet (`TODO(verify)`; the live tests in `tests/integration/test_lidar_live.py` and
  `tests/server/test_deployed_server.py::test_lidar_lifecycle_budget_and_two_viewers` settle them): that
  measurement coordinates need no y flip, that `object_idx` of the semantic sensor is the hit actor id, the
  semantic tag table (0..28) and the attribute names for 0.9.16, spawning a sensor in synchronous mode between two
  ticks, and the measurement callback timing relative to `tick()`.

## Phase 5 - API and WebSocket hub (2026-10-10)

No change to any model number. What the server now guarantees (`src/server/ws_hub.py`, `topics.py`, `roles.py`,
`api_routes.py`, `docs/api.md`, `docs/openapi.json`):

- **Roles and tokens**: admin (`ADMIN_TOKEN`), vehicle (`session_token`, a separate secret returned by join; the
  session id in the admin list is not a credential for the new API), observer (nobody; read-only, switchable with
  `observer_public`). WebSocket auth is the first message, never the URL. Nothing an observer can read contains a
  session id or token (tested for world.state and the LiDAR sensor list; the LiDAR owner is the client name).
- **Ground truth is not for vehicles** (D10): `world.state`, `v2x.frame`, `v2x.links`, `v2x.stats`, `events` and
  `run.status` are refused for a vehicle token, a vehicle reads its own telemetry and its own inbox only. A test per
  ground-truth topic, and mutation checks (removing the role check or the filter rule fails the suite).
- **`v2x.links` needs a filter** and is capped (`v2x_links_max_rows` = 2000, `truncated` says so). Measured: 2000
  link rows as keyed objects are 668 kB of JSON, as `columns` + `rows` 208 kB (3.2x smaller, so that is the format).
- **Slow readers** (acceptance of the phase): delivery is latest-wins per connection and topic, `seq` gaps show the
  skipped updates, a send that stalls for `send_timeout_s` closes the connection (1013). With the real world on the
  recording stub, a client that never finishes a read leaves the simulation at the same frame rate (>= 90 % of the
  frames of an undisturbed run in 2 s; the hub only reads published snapshots).
- **C11 disconnect grace (D4)**: closing the telemetry socket no longer removes the vehicle at once. The simulation
  thread removes a session that has had no WebSocket and no sign of life (REST poll, hub auth) for
  `session_disconnect_grace_s` = 30 s (0 restores the old behaviour); reconnecting in time keeps the session.
  Session/expiry checks run every second now, not every 5 s.
- **Published views** are built on the simulation thread: `world.state` (all vehicles and pedestrians at
  `world_state_rate_hz` = 10, RSUs), V2X summary and `v2x.stats` after each V2X update, `run.status` at 1 Hz with
  tick duration p50/p95/max, V2X update duration and the real-time factor. Cost per V2X update of the extra work on
  12,000 links: statistics 8.8 ms, summary 0.01 ms (the update itself is 136 ms at that size, see above).
- **Channel statistics** (`src/v2x/stats.py`): PRR per 20 m distance bin, separately LOS/NLOS and per link type,
  drop causes, receive-margin histogram, over the last 20 updates. Links removed by the awareness filter are not
  counted in the PRR (the radio did not decide them); this is printed with the numbers.
- **DENM injection (F10, D11)**: `POST /api/v2x/denm` (admin) puts a DENM at a map position with cause code,
  relevance distance, optional originator and optional duration in simulation seconds; without originator a fixed
  roadside transmitter at the event sends it. It goes through the channel model like every other message
  (tested: the receiver sees an I2V link).
- **`/api/map/geometry`**: lane polylines (a point about every 2 m) and junction boxes, built once on the
  simulation thread and cached, ETag + gzip. TODO(verify) on CARLA 0.9.16: waypoint stepping, junction box rotation,
  cost on the real maps (the live test prints lanes/points/seconds).
- Event log fed by sessions, population changes, DENMs and every LiDAR transition, readable as the `events` topic.
- `docs/openapi.json` and `docs/api.md` are generated and a test fails when they are out of date, so an API change is
  always a reviewable diff.

## Phase 6 - Frontend (2026-10-10)

No model number changed. What a researcher now sees, and what the server does so that the browser cannot mislead
(`src/web/`, `src/testing/fake_carla/`, `tests/e2e`, `tests/js`):

- **Pages**: `/` status, `/observer` (map in ground-truth and perspective mode, station inspector, link log,
  channel analytics, LiDAR views), `/vehicle` (join, my radio, LiDAR on request, own-perspective map), `/admin`
  (connection, population, hot configuration, RSUs, DENM injection by clicking the map, sessions, LiDAR budget and
  sensors, WebSocket clients). The three old pages are gone; `/client` redirects to `/vehicle`.
- **Display rules implemented and tested**: every live panel carries its frame and simulation time; rounding only in
  `units.js`; a missing value is an em dash with the reason in a tooltip (SPAT end time "unknown", never a number);
  link outcome is colour *and* dash pattern; stale data is marked; nothing is interpolated.
- **Perspective mode** draws only what the station received, at the positions in the received messages, with the age of
  that information; stations not heard in the latest update remain as ghosts. The vehicle page has no ground-truth
  subscription - checked from the server side (`GET /api/hub/connections`) by the browser test.
- **LiDAR vs V2X overlay** uses the newest V2X inbox whose frame is not newer than the point cloud (tested), and shows
  the gap `(F_lidar - F_v2x) * dt` in the HUD. Own-vehicle self-hits are filtered and counted. Colour modes a frame
  cannot offer (intensity on a semantic sensor) are disabled, not faked.
- **Channel statistics** are binned in 10 m, LOS and NLOS apart, with a Wilson 95 % interval and `n` per bin; two windows
  (last 60 s of simulation time, whole run). Awareness-filtered links are not counted in the PRR.
- **Configuration at runtime**: hot fields (`awareness_range_m`, `v2x_penetration_rate`, LiDAR limits, grace period,
  observer access, rates of published views) change via `PATCH /api/config`, all or nothing, each change an event with
  the frame it was applied at; everything else is marked `restart` and refused (409). RSUs can be disabled and their
  transmit power changed; sensors can be pinned (admin only).
- **Fake world** (`start_server.py --fake`): the REAL server runs on a deterministic synthetic CARLA (grid city, vehicles on
  rings, pedestrians, signal groups, ray-cast LiDAR with actor ids). It is a test double: no physics, no collisions,
  lights are not obeyed. The pages show a banner when it is active. All browser tests run against it.
- **Bugs found only by running a browser** (fixed, with tests): default timer functions called as methods ("Illegal
  invocation"); `[hidden]` defeated by `display:flex`; the map picker needing the map in view; the container's locale.
- **Not measured**: the handoff asks for >= 15 rendered fps with three LiDAR views of 50k points at 20 Hz on the lab
  machine. This container has no GPU (headless Chromium with software WebGL renders about 15 fps for one 13k-point view
  and is not representative), so the number has to be measured on the lab machine. The viewer renders only when the scene
  changed, uploads only the used part of its preallocated buffers and decodes in a worker, and counts dropped frames in
  its HUD, so the measurement is one line of the HUD. The observer allows two views at once; the limit is a constant
  (`MAX_LIDAR_VIEWS`) in `observer-core.js`.
- **Not done**: the Run card is a placeholder (recording is Phase 7); the manual live-CARLA script of the handoff is part of
  the documentation phase; the map's RSU coverage ring is not drawn yet.

## Phase 7 - recording, headless runs, reproducibility (2026-10-10, no change to the V2X/DSRC models)

New, outside the models:
- `RunRecorder` writes a run directory (`docs/ER_DATABASE_MODEL.md`): `stations`, `v2x_links`, `messages`,
  `lidar_meta`, optional raw LiDAR, `events`, and a `manifest.json` with git sha/dirty, seeds, the full
  configuration, hot-config changes and sha256 per file. A bounded queue feeds a writer thread; on overflow the
  run is failed and marked so rather than silently dropping rows.
- `python -m src.cli run scenarios/crossing.yaml --fake` runs a scripted scenario headless; the server also has
  `POST /api/runs`, `POST /api/runs/current/stop`, `GET /api/runs[/{id}/manifest]` and an admin Run card.

The one number that moved: the DSRC channel RNG is now seeded from `WorldConfig.seed` instead of a constant.
Default seed is 42, the old constant, so default outputs are unchanged (golden tests untouched).

Measured on the fake backend (`scenarios/crossing.yaml`, seed 7, 400 ticks at 20 Hz, 40 V2X updates,
LiDAR raw recording on): 3.0 s wall clock, 14,056 link rows (6.0 MB), 990 message lines (0.7 MB),
14.9 MB total including raw LiDAR. Two runs give identical sha256 for stations, links, messages and
lidar_meta; a different seed gives different files (`tests/e2e/test_scenarios.py`).

Not verified: reproducibility on real CARLA (traffic-manager and physics nondeterminism may break it). A live
test exists (`tests/integration/test_reproducibility_live.py`, marker `carla`) and has not been run.

## Phase 8 - documentation (2026-10-10, no change to any model)

- `AGENTS.md` replaces `.instructions.md`, `docs/agents/*` and `.github/agents/*` (they described removed code and
  referenced files that never existed); `CLAUDE.md` points to it. New `README.md`, `docs/running.md` (with the manual
  check for a live CARLA), run-layout document `docs/ER_DATABASE_MODEL.md`, cleaned `ACKNOWLEDGMENTS.md` and
  `DOCKER.md`. `docs/science/limitations.md` brought up to date (several "fix planned" items had been fixed in Phase 3).
- Small code fixes found while executing the documented commands: `python -m src.cli serve --fake ...` rejected its
  options (now forwarded untouched); `CARLA_HOST`/`CARLA_PORT`, which `docker-compose.yml` has always set, were read by
  nothing and are now the defaults of `--carla-host`/`--carla-port`; the Docker image now contains `scenarios/` and the
  compose file mounts `./runs`.
- Not executed: everything that needs a Docker daemon or a real CARLA server (marked in `docs/running.md`).
