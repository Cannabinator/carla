# Conventions

> Match these patterns when writing or refactoring code so the agent's changes fit the codebase.

## Imports
- The package root is `src`. Imports are **absolute from `src`**, e.g.
  `from src.server.persistent_world import PersistentCARLAServer`
  `from src.utils.config import DEFAULT_SIM_CONFIG`.
- `start_server.py` inserts the repo root on `sys.path`, so **run from the repo root**.
  Don't rely on being inside `src/` — there is no `src/__init__.py` making it a package in
  the editable-install sense; the `sys.path` trick is what makes `src.` imports work.
- Relative imports are used *inside* subpackages (e.g. `from .messages import …`,
  `from ..v2v.network_enhanced import …`).

## Configuration
- **All tunables live in `src/config.py`** (not `src/utils/`) as `@dataclass`es
  (`SimulationConfig`, `VisualizationConfig`, `V2VConfig`, `VehicleConfig`, `DSRCConfig`).
  Exported as `DEFAULT_*`. Don't introduce magic numbers in logic modules.
- Simulation defaults: fixed dt **0.05 s** (20 FPS), Traffic Manager port **8001**,
  substepping on (`max_substep_delta_time=0.01`, `max_substeps=10`),
  `use_hybrid_physics=False`.

## Coding patterns
- **Thread safety:** `PersistentCARLAServer` mutates actor lists only under `self._lock`.
  The tick thread reads the vehicle dict; session expiry re-acquires the lock for despawns.
  Follow this if you touch actor lifecycle.
- **Logging:** use `logging.getLogger(__name__)` with the module's `%(name)s` banner set up
  in `start_server.py` (`%(asctime)s %(levelname)-8s %(name)s — %(message)s`).
- **REST/WebSocket:** FastAPI. Pydantic models for responses (see `v2v/api.py`:
  `BSMResponse`, `NeighborInfo`, `ThreatInfo`). CORS middleware is already configured.
- **Message model:** V2V messages are SAE J2735-style (BSM/DENM/CAM). V2X adds MAP/SPAT/VRUAM/CPM.
  Keep the `V2XNetworkEnhanced(V2VNetworkEnhanced)` inheritance so the V2V pipeline stays
  backward compatible.
- **Frontend:** plain static HTML and ES modules in `src/web/` (no build step); no
  build step.

## Git workflow
- Branch **`developer`** is the main dev line; it tracks **`gitlab/developer`**.
- Pull team changes from **`gitlab`**, not `origin` (origin is a GitHub fork).
- Feature work belongs in `feature/*` branches.

## Documentation
- Human-facing docs go in repo-root `DOCKER.md`, `README.md`, and `docs/V2V_*.md`.
- **Agent-oriented** orientation goes in `docs/agents/` (this folder). Don't duplicate the
  human docs; link to them from `AGENTS.md`.
