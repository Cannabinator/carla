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
