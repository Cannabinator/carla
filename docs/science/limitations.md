# Known limitations and modelling assumptions

Things a reader of results from this platform must know. Items marked **fix planned** have a handoff id
(S/L/C numbers refer to `REFACTOR_HANDOFF.md`). Nothing here has been validated against measurement data.

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
* **Only V2V BSMs go through the channel model.** RSU (SPAT/MAP) and pedestrian (VRUAM) reception is a plain
  distance test, and `RSUConfig.tx_power_dbm` is unused (S4, **fix planned**).
* **Only registered stations transmit.** On the persistent server these are the joined client vehicles, so
  background traffic is silent (S3, **fix planned**: penetration rate), which makes contender counts and
  neighbour statistics depend on how many clients joined.
* **RSUs**: one RSU per *traffic light* actor, at most ten, in arbitrary order, so an intersection is not one
  RSU with several signal groups (S7, **fix planned**).
* **Threat levels**: centre-to-centre distance (TTC overestimated); thresholds (10/5/2 s, 100 m) are
  project-defined.
* **Update rate**: 2 Hz by default, chosen by the project. SAE J2945/1 (BSM) and ETSI EN 302 637-2 (CAM) use
  other rates and triggering; the transmission-rate attribution in older docstrings was wrong (S18).
* **Standard attributions are unverified**: DENM cause codes (S15), RSU 33 dBm "ETSI TR 101 607" and the
  "Class A 23 dBm ERP" power class (S16), SPAT message structure.

## Scale

* The channel update runs on the simulation thread. One update takes about 7 ms for 50 stations, 34 ms for
  100, 126 ms for 150 and 1.3 s for 300 (see CHANGELOG), against a 50 ms frame: plan penetration rate and
  update rate accordingly.

## Simulation

* **Reproducibility** depends on CARLA/Traffic Manager determinism, which has not been measured here.
  Spawn choices and the channel RNG are seeded; real-time pacing is on by default, so command timing relative
  to frames is not reproducible unless commands are applied by a script (Phase 7).
* **Only the autopilot drives**; clients cannot control their vehicle.
* **Disconnecting a client WebSocket ends the session at once** (C11, planned grace period).
* Synchronous mode with fixed 0.05 s step and physics substepping (10 x 0.01 s); `use_hybrid_physics` must
  stay off (zero-velocity issue).

## LiDAR

* The LiDAR path is the old one until Phase 4: sensor-to-world transform written by hand (L1, **may be wrong
  on slopes**), wall-clock timestamps, not frame-synchronised with V2X (L2/L3), enabled only at join, fixed
  configuration (L4), points decimated before they are stored (L5).
