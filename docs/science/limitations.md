# Known limitations and modelling assumptions

Things a reader of results from this platform must know. S/L/C numbers refer to the defect list in
`REFACTOR_HANDOFF.md`; what was changed for each is in `CHANGELOG.md`. Nothing here has been validated against measurement data.

## Radio / V2X

* **Parameters are assumptions.** Path-loss exponents, shadowing, sensitivity, the logistic steepness and the
  contention term are not calibrated against data and their cited sources are unverified
  (`docs/science/model.md`). In line of sight the radio is effectively lossless to ~150 m.
* **CSMA/contention term is experimental** (S6), not a MAC model.
* **Shadowing is i.i.d. per packet and tick** and applied through a second random draw (S5).
* **No noise floor, interference, hidden terminals or collisions**; "margin" is power above sensitivity.
* **Zero latency** (S21): every delivered message arrives in the tick it was sent.
* **NLOS = another registered vehicle near the tx-rx line** (S20). Buildings are ignored; vehicles that are
  not V2X stations are not blockers.
* **BSMs, RSU MAP/SPAT, pedestrian VRUAM and DENM all go through the channel model** (S4; CHANGELOG, Phase 3).
  The coverage radii (RSU 300 m, pedestrian 100 m) remain hard limits on top of the radio model. The pedestrian
  device power (23 dBm) is an assumption without a source. Antenna height is not modelled.
* **Which vehicles transmit** is decided by `v2x_penetration_rate` (default 1.0: all background vehicles and all
  clients) (S3, D2). Contender counts, and with them the experimental contention term, depend on that rate.
* **RSUs**: one per group of traffic lights as reported by CARLA's `get_group_traffic_lights()` (S7). That this
  matches physical intersections on your map is `TODO(verify)` on live CARLA.
* **Threat levels**: centre-to-centre distance (TTC overestimated); thresholds (10/5/2 s, 100 m) are
  project-defined.
* **Update rate**: 2 Hz by default, chosen by the project. SAE J2945/1 (BSM) and ETSI EN 302 637-2 (CAM) use
  other rates and triggering; the transmission-rate attribution in older docstrings was wrong (S18).
* **Standard attributions are unverified**: DENM cause codes (S15), RSU 33 dBm "ETSI TR 101 607" and the
  "Class A 23 dBm ERP" power class (S16), SPAT message structure.

## Scale

* A complete update with 100 vehicles, 20 RSUs and 40 pedestrians took 136 ms (mock vehicles); see the
  CHANGELOG table. With roughly 100 stations the loop falls behind real time for ~2 frames per update.
* The channel update runs on the simulation thread. One update takes about 7 ms for 50 stations, 34 ms for
  100 (34 ms without link records, 72 ms with them), 126 / 223 ms for 150 and 1.3 s for 300 (see CHANGELOG), against a 50 ms frame: plan penetration rate and
  update rate accordingly.

## Simulation

* **Reproducibility** of live runs depends on CARLA/Traffic Manager determinism, which has not been measured
  here; see "Reproducibility of runs" below. The channel RNG is seeded from `WorldConfig.seed`.
* **Only the autopilot drives**; clients cannot control their vehicle (D3).
* **Disconnecting a client WebSocket ends the session after a grace period** (`session_disconnect_grace_s`,
  default 30 s, D4); reconnecting in time keeps the session.
* Synchronous mode with fixed 0.05 s step and physics substepping (10 x 0.01 s); `use_hybrid_physics` must
  stay off (zero-velocity issue).

## LiDAR

* Sensors exist only on request, with a global budget (`lidar_max_sensors`, `lidar_max_points_per_second`);
  both limits are **unmeasured defaults** to be tuned on the real server (CHANGELOG, Phase 4).
* Not verified on a real CARLA: that measurement coordinates need no y flip, that `object_idx` of the
  semantic sensor is the hit actor id, the semantic tag table, and the callback timing relative to `tick()`
  (`TODO(verify)`; settled by `tests/integration/test_lidar_live.py`).
* Streams to viewers are subsampled (seeded, order preserved); recordings never are.
* The 15 fps target with three simultaneous LiDAR views in the browser needs a GPU machine and has not been measured.
* The LiDAR-vs-V2X overlay is a visual aid, not a metric (D13).

## Reproducibility of runs (Phase 7)

- `python -m src.cli run scenario.yaml` starts the world paused, sets it up and starts the recording on the simulation
  thread, then lets it tick without wall-clock pacing; each scripted command runs just before the tick with its index.
  On the **fake world** two runs of one scenario give byte-identical `stations.jsonl`, `v2x_links.jsonl`, `messages.jsonl`,
  `lidar_meta.jsonl` and raw LiDAR files (tested; `events.jsonl` carries wall-clock times and differs on purpose).
- On a **real CARLA** this is *not verified*. CARLA's physics and its Traffic Manager are not guaranteed to be
  deterministic, even with a fixed seed, synchronous mode and a fixed step. `tests/integration/test_reproducibility_live.py`
  runs a scenario twice and prints the first diverging line. Until it has been run on the lab machine, treat runs on CARLA
  as seeded but **not** reproducible, and write the first divergence frame and its cause here when it is known.
- What the recording does guarantee even without that: every number in the V2X files was computed from the stations
  recorded next to it (`stations.jsonl` is the ground truth of the same update), so an analysis never needs the simulator.
- Scripted commands count ticks from the release of the paused loop. A command whose target does not exist at that tick
  (for example "first vehicle" with none spawned) fails loudly (event `scripted_command_failed`, logged) and the run
  continues; the manifest keeps the scenario so this is visible afterwards.
- LiDAR sensors in scenarios are pinned by default: the wall-clock keep-alive release would make a run depend on wall time.

