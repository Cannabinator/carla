"""The run manifest: everything needed to know what produced a run's files (REFACTOR_HANDOFF 5.6).

Written when the run starts (status "running") and rewritten when it ends with the final counts, the config changes
made during the run and the sha256 of every data file, so two runs can be compared by their manifests alone.
"""

from __future__ import annotations

import hashlib
import json
import platform
import re
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, List, Optional

SCHEMA_VERSION = 1


def git_info(cwd: Optional[Path] = None) -> Dict[str, Any]:
    """Commit and whether the working tree has uncommitted changes; ``unknown`` when this is not a git checkout."""
    root = cwd or Path(__file__).resolve().parents[2]
    try:
        sha = subprocess.run(["git", "rev-parse", "HEAD"], cwd=root, capture_output=True, text=True, timeout=10, check=True).stdout.strip()
        dirty = bool(subprocess.run(["git", "status", "--porcelain", "--untracked-files=no"], cwd=root, capture_output=True,
                                    text=True, timeout=10, check=True).stdout.strip())
        return {"sha": sha, "short_sha": sha[:7], "dirty": dirty}
    except (OSError, subprocess.SubprocessError):
        return {"sha": "unknown", "short_sha": "unknown", "dirty": None}


def library_versions() -> Dict[str, str]:
    out = {"python": sys.version.split()[0], "platform": platform.platform()}
    for name in ("numpy", "pydantic", "fastapi", "uvicorn", "websockets", "yaml"):
        try:
            mod = __import__(name)
            out[name] = str(getattr(mod, "__version__", "?"))
        except ImportError:
            out[name] = "not installed"
    return out


def slug(name: str) -> str:
    s = re.sub(r"[^A-Za-z0-9._-]+", "-", name.strip()).strip("-.")
    return s[:48] or "run"


def run_id(now: datetime, short_sha: str, name: str) -> str:
    return f"{now.strftime('%Y%m%dT%H%M%SZ')}_{short_sha}_{slug(name)}"


def utc_now() -> datetime:
    return datetime.now(timezone.utc)


def sha256_file(path: Path) -> Dict[str, Any]:
    h = hashlib.sha256()
    n = 0
    lines = 0
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
            n += len(chunk)
            lines += chunk.count(b"\n")
    return {"bytes": n, "lines": lines, "sha256": h.hexdigest()}


def write_manifest(run_dir: Path, manifest: Dict[str, Any]) -> None:
    tmp = run_dir / "manifest.json.tmp"
    tmp.write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n")
    tmp.replace(run_dir / "manifest.json")          # atomic: a reader never sees a half-written manifest


def read_manifest(run_dir: Path) -> Dict[str, Any]:
    return json.loads((run_dir / "manifest.json").read_text())


def list_runs(runs_dir: Path) -> List[Dict[str, Any]]:
    out = []
    if not runs_dir.exists():
        return out
    for d in sorted(runs_dir.iterdir(), reverse=True):
        mf = d / "manifest.json"
        if d.is_dir() and mf.exists():
            try:
                m = json.loads(mf.read_text())
            except ValueError:
                continue
            out.append({"run_id": m.get("run_id", d.name), "name": m.get("name"), "status": m.get("status"),
                        "started_utc": m.get("started_utc"), "ended_utc": m.get("ended_utc"),
                        "frames": m.get("frames_recorded"), "recorded": m.get("recorded")})
    return out
