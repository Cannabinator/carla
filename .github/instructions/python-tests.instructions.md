---
description: "Use when writing, updating, or reviewing Python tests in this repo. Covers unit vs integration boundaries, deterministic simulation testing, and progressive test complexity for CARLA, V2V, and LiDAR code."
name: "Python Testing Guidelines"
applyTo: "tests/**/*.py"
---
# Python Testing Guidelines

## Scope

- Apply these rules to files under tests/.
- Keep tests focused on behavior and regressions, not implementation details.

## Framework And Style

- Preserve existing style in a file: if a module already uses unittest, continue with unittest in that module.
- For new test modules, prefer pytest style unless there is a clear reason to align with unittest.
- Use descriptive test names that state the expected behavior.
- Keep each test short: arrange, act, assert.

## Complexity Ladder

Write tests in this order, increasing complexity only when lower levels are covered.

1. Level 1 - Pure unit tests
- No CARLA runtime dependency.
- Use mocks/fakes for world, actor, and sensor objects.
- Validate message fields, transforms, throttling gates, and helper utilities.

2. Level 2 - Component tests
- Exercise one subsystem end-to-end in-process (for example V2V network update loop with mocked world snapshots).
- Assert stable interfaces and output schema.

3. Level 3 - Integration/system tests
- Require external CARLA server and real timing assumptions.
- Guard with clear skip conditions when CARLA is unavailable.
- Keep runtime bounded and assertions high-value.

Do not jump to Level 3 when Level 1 or 2 can catch the bug.

## CARLA-Specific Constraints

- In synchronous simulation logic, do not use time.sleep() to advance simulation behavior; drive updates with tick/frame progression.
- Keep fixed-delta assumptions explicit in assertions when relevant (20 FPS, V2V effective 2 Hz update behavior).
- Keep Traffic Manager port separate from web server port in test setup.
- Avoid leaking actors and sensors; always clean up resources created by tests.

## Scientific Validity Requirements

- Treat data correctness as a test target: include assertions that protect metric validity, not only code execution.
- Prefer simulation time from CARLA snapshots over wall-clock time when asserting message timing, TTC, or temporal alignment.
- For threat and TTC tests, cover both converging and receding trajectories; receding cases should not produce finite collision TTC.
- For neighbor and range metrics, verify distances are refreshed across updates (no stale cached values).
- When tests inspect acceleration or cadence, assert tolerances explicitly and justify them in comments.

## V2V MQTT 2 Hz Requirements

- For V2V communication tests, enforce a target cadence of 2 Hz (0.5 s period) unless the test explicitly documents another mode.
- When MQTT transport is enabled, verify both sides of behavior: local publish cadence and received message cadence.
- Keep MQTT tests deterministic by mocking broker interactions for unit/component levels; reserve real broker checks for integration tests.
- Separate transport correctness from network-physics correctness: test serialization/transport independently from neighbor discovery/threat logic.
- For cadence assertions, prefer simulation-time or controlled fake-time clocks over uncontrolled wall-time sleeps.

## Determinism And Reliability

- Avoid flaky timing assertions; prefer tolerance windows and frame-based checks.
- Seed randomness when synthetic data is used.
- Do not rely on log text for correctness when direct state assertions are available.
- Keep tests isolated: no shared mutable global state across tests.

## Data And Contract Assertions

- For V2V messages, assert required fields and units are consistent.
- For LiDAR payloads, assert schema and basic invariants (point count, tag presence, bounds), not exact noisy values.
- For API tests, validate status code, response shape, and key semantics.

## Fast Feedback Commands

Run these first unless task explicitly requires system coverage:

```bash
python -m pytest tests/v2v/ -v
python -m pytest tests/test_lidar_api.py -v
```

If running specific V2V files, use repo-relative paths from workspace root:

```bash
python -m pytest tests/v2v/test_v2v_basic.py tests/v2v/test_scientific_integrity.py -v
```

Run integration/system tests only when needed and CARLA is available:

```bash
python -m pytest tests/test_v2v_lidar.py -v
python tests/test_frontend_visual.py --run
python tests/test_complete_system.py
```

## Review Checklist

- Is this the lowest complexity level that proves the behavior?
- Are skip conditions explicit for environment-dependent tests?
- Are cleanup paths guaranteed for actors/sensors/resources?
- Are assertions behavior-focused and deterministic?
