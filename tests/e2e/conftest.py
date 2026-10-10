"""A real server process on the fake CARLA (``start_server.py --fake``) for end-to-end and browser tests.

No CARLA, no network access needed. The server runs in a subprocess on a free port with a known admin token.
"""

import json
import os
import socket
import subprocess
import sys
import time
import urllib.error
import urllib.request
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
TOKEN = "e2e-admin-token"


def _free_port() -> int:
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


class FakeServer:
    def __init__(self, base: str, proc: subprocess.Popen):
        self.base, self.proc, self.token = base, proc, TOKEN

    @property
    def ws_base(self) -> str:
        return self.base.replace("http://", "ws://")

    def request(self, method, path, body=None, headers=None, timeout=20):
        data = json.dumps(body).encode() if body is not None else None
        h = {"Content-Type": "application/json"} if data else {}
        h.update(headers or {})
        req = urllib.request.Request(self.base + path, data=data, method=method, headers=h)
        try:
            with urllib.request.urlopen(req, timeout=timeout) as r:
                return r.status, r.read()
        except urllib.error.HTTPError as e:
            return e.code, e.read()

    def json(self, method, path, body=None, admin=False, headers=None, **kw):
        h = dict(headers or {})
        if admin:
            h["X-Admin-Token"] = TOKEN
        status, raw = self.request(method, path, body, h, **kw)
        try:
            return status, json.loads(raw or b"null")
        except ValueError:
            return status, raw


@pytest.fixture(scope="session")
def fake_server(tmp_path_factory):
    port = _free_port()
    log = tmp_path_factory.mktemp("fake_server") / "server.log"
    env = {**os.environ, "PYTHONUNBUFFERED": "1"}
    with open(log, "w") as fh:
        proc = subprocess.Popen(
            [sys.executable, str(ROOT / "start_server.py"), "--fake", "--port", str(port), "--host", "127.0.0.1",
             "--admin-token", TOKEN, "--fake-vehicles", "12", "--fake-pedestrians", "6"],
            cwd=str(ROOT), env=env, stdout=fh, stderr=subprocess.STDOUT)
    srv = FakeServer(f"http://127.0.0.1:{port}", proc)
    deadline = time.time() + 60
    while time.time() < deadline:
        if proc.poll() is not None:
            pytest.fail(f"fake server exited with {proc.returncode}:\n{log.read_text()[-3000:]}")
        try:
            status, body = srv.json("GET", "/api/status", timeout=2)
            if status == 200 and body.get("connected"):
                break
        except (urllib.error.URLError, ConnectionError, OSError):
            pass
        time.sleep(0.3)
    else:
        proc.kill()
        pytest.fail(f"fake server did not become ready:\n{log.read_text()[-3000:]}")
    yield srv
    proc.terminate()
    try:
        proc.wait(timeout=15)
    except subprocess.TimeoutExpired:
        proc.kill()
