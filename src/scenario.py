"""Headless, seeded, reproducible runs: ``python -m src.cli run scenario.yaml``.

A scenario fixes everything that can change a result: the seed, the configuration, the population, the commands and
the frame at which each command executes. The world is started paused, set up and the recording started on the
simulation thread, then released; each command runs just before the tick with its index, so it happens at the same
frame in every run. ``realtime`` is off: the simulation ticks as fast as it can.

    name: crossing
    seed: 7
    fake: true                       # or --fake: the deterministic synthetic world instead of CARLA
    duration_frames: 400             # ticks after the release
    config: {awareness_range_m: 75}  # any WorldConfig field
    spawn: {vehicles: 12, pedestrians: 4}
    record: [links, stations, messages, lidar_meta]
    commands:
      - {frame: 100, action: inject_denm, x: 100, y: 100, cause_code: ACCIDENT, relevance_distance_m: 200, duration_s: 5}
      - {frame: 150, action: lidar_request, vehicle: first, record_raw: true}
      - {frame: 300, action: set_config, changes: {awareness_range_m: 50}}
"""

from __future__ import annotations

import logging
import threading
import time
from pathlib import Path
from typing import Any, Dict, List, Literal, Optional, Union

import yaml
from pydantic import BaseModel, ConfigDict, Field

from src.recording.recorder import DEFAULT_RECORD

logger = logging.getLogger(__name__)

ACTIONS = ("inject_denm", "cancel_denm", "lidar_request", "lidar_release", "set_config", "rsu", "spawn_traffic",
           "clear_traffic", "spawn_pedestrians", "clear_pedestrians")


class Spawn(BaseModel):
    vehicles: int = Field(default=0, ge=0, le=300)
    pedestrians: int = Field(default=0, ge=0, le=200)


class Command(BaseModel):
    model_config = ConfigDict(extra="allow")
    frame: int = Field(ge=0, description="Executes just before tick number ``frame`` (0 = before the first tick)")
    action: Literal[ACTIONS]  # type: ignore[valid-type]


class Scenario(BaseModel):
    model_config = ConfigDict(extra="forbid")
    name: str = Field(min_length=1, max_length=64)
    seed: int = 42
    fake: bool = False
    host: str = "localhost"
    port: int = 2000
    duration_frames: int = Field(ge=1)
    config: Dict[str, Any] = Field(default_factory=dict)
    spawn: Spawn = Field(default_factory=Spawn)
    record: List[str] = Field(default_factory=lambda: list(DEFAULT_RECORD))
    commands: List[Command] = Field(default_factory=list)


def load_scenario(path: Union[str, Path]) -> Scenario:
    data = yaml.safe_load(Path(path).read_text())
    if not isinstance(data, dict):
        raise ValueError(f"{path}: a scenario is a YAML mapping")
    return Scenario.model_validate(data)


def _pick_vehicle(world, which: Any) -> int:
    ids = sorted(a.id for a in world._background_vehicles)
    if isinstance(which, int) and which > 1000:
        return which
    if which in (None, "first"):
        if not ids:
            raise LookupError("no background vehicle to attach to")
        return ids[0]
    if isinstance(which, int) and 0 <= which < len(ids):
        return ids[which]
    raise LookupError(f"cannot pick vehicle {which!r}")


def _execute(world, cmd: Command) -> None:
    from src.lidar.config import LidarConfig
    from src.lidar.sensor import ADMIN
    from src.v2x.messages.denm import DENMCauseCode
    a: Dict[str, Any] = cmd.model_extra or {}
    act = cmd.action
    if act == "inject_denm":
        cause = a.get("cause_code", "ACCIDENT")
        cause = int(DENMCauseCode[cause.upper()] if isinstance(cause, str) else DENMCauseCode(cause))
        world.inject_denm(float(a["x"]), float(a["y"]), float(a.get("z", 0.0)), cause, int(a.get("subcause_code", 0)),
                          float(a.get("relevance_distance_m", 150.0)), int(a.get("information_quality", 7)),
                          a.get("station_id"), a.get("duration_s"))
    elif act == "cancel_denm":
        world.cancel_denm(int(a["station_id"]), int(a["action_id"]))
    elif act == "lidar_request":
        cfg = LidarConfig.model_validate(a.get("config", {}))
        # pinned by default: an unwatched sensor would otherwise be released by the wall-clock keep-alive and the run
        # would depend on wall time
        world.lidar_request(ADMIN, _pick_vehicle(world, a.get("vehicle")), cfg, record_raw=bool(a.get("record_raw", False)),
                            pinned=bool(a.get("pinned", True)))
    elif act == "lidar_release":
        world.lidar_release(ADMIN, int(a["sensor"]))
    elif act == "set_config":
        world.apply_config(dict(a["changes"]))
    elif act == "rsu":
        world.set_rsu(int(a["station_id"]), a.get("enabled"), a.get("tx_power_dbm"))
    elif act == "spawn_traffic":
        world.spawn_background_traffic(int(a["count"]))
    elif act == "clear_traffic":
        world.despawn_background_traffic()
    elif act == "spawn_pedestrians":
        world.spawn_pedestrians(int(a["count"]))
    elif act == "clear_pedestrians":
        world.despawn_pedestrians()


def run_scenario(sc: Scenario, runs_dir: Union[str, Path] = "runs", fake: Optional[bool] = None, timeout_s: float = 3600.0) -> Dict[str, Any]:
    """Run ``sc`` to completion and return its manifest. ``fake`` overrides the scenario's own flag."""
    use_fake = sc.fake if fake is None else fake
    if use_fake:
        from src.testing.fake_carla import install
        install(sc.seed)
    from src.server.persistent_world import PersistentCARLAServer
    from src.server.world_config import WorldConfig

    cfg = WorldConfig(host="fake" if use_fake else sc.host, port=sc.port, seed=sc.seed, realtime=False,
                      runs_dir=str(runs_dir), **sc.config)
    world = PersistentCARLAServer(cfg)
    world.start(paused=True)
    done = threading.Event()
    result: Dict[str, Any] = {}
    try:
        def setup() -> Dict[str, Any]:
            if sc.spawn.vehicles:
                world.spawn_background_traffic(sc.spawn.vehicles)
            if sc.spawn.pedestrians:
                world.spawn_pedestrians(sc.spawn.pedestrians)
            return world.start_run(sc.name, sc.record, scenario=sc.model_dump())
        info = world.submit(setup).result(timeout=600)
        base = world.tick_index                       # frame indices of commands count from the release
        for cmd in sc.commands:
            world.at_tick(base + cmd.frame, lambda c=cmd: _execute(world, c))

        def finish() -> None:
            result.update(world.stop_run("scenario finished"))
            done.set()
        world.at_tick(base + sc.duration_frames, finish)
        t0 = time.time()
        world.resume()
        if not done.wait(timeout_s):
            raise TimeoutError(f"scenario did not finish within {timeout_s:.0f} s")
        logger.info("scenario %s finished in %.1f s", sc.name, time.time() - t0)
        manifest = world.run_manifest(info["run_id"])
        manifest["_dir"] = str(Path(runs_dir) / info["run_id"])
        return manifest
    finally:
        world.stop()
