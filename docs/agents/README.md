# docs/agents/ — Agent Orientation Set

Maintained companion to the root [`AGENTS.md`](../AGENTS.md). These files help an AI agent
understand and navigate the CARLA V2V Research Platform quickly and safely.

| File | When to read it |
|------|-----------------|
| [architecture.md](architecture.md) | Before editing server / V2V / V2X logic — understand the data flow. |
| [running.md](running.md) | Before running, testing, or deploying the platform. |
| [conventions.md](conventions.md) | Before writing or refactoring code — match existing patterns. |
| [gotchas.md](gotchas.md) | Before debugging weird behavior — known footguns. |

## Guiding principles for agents working here
1. **Don't duplicate human docs.** `docs/V2V_*.md`, `DOCKER.md`, `LaurinNotes.md` already
   exist. Link to them; extend this folder only for *agent-oriented* orientation.
2. **Config lives in one place.** All tunables are dataclasses in `src/config.py`.
   Don't hardcode magic numbers elsewhere.
3. **Imports are absolute from `src`.** The repo root is added to `sys.path` by
   `start_server.py`. Run the server from the repo root.
4. **`developer` is the main branch and `gitlab` is the source of truth** for team work.
5. **Thread safety matters.** `PersistentCARLAServer` mutates actor lists under `_lock`.
