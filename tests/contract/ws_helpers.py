"""Helpers for TestClient WebSocket tests: reads with a timeout (a missing message fails the test instead of
hanging it) and a clean disconnect (see ``connect``)."""

import contextlib
import threading
import time


def _call_with_timeout(fn, timeout):
    box = {}

    def run():
        try:
            box["v"] = fn()
        except BaseException as exc:  # noqa: BLE001 - re-raised in the caller
            box["e"] = exc

    th = threading.Thread(target=run, daemon=True)
    th.start()
    th.join(timeout)
    if th.is_alive():
        raise AssertionError(f"no WebSocket message within {timeout:g} s")
    if "e" in box:
        raise box["e"]
    return box["v"]


def recv_json(ws, timeout=5.0):
    return _call_with_timeout(ws.receive_json, timeout)


def recv_raw(ws, timeout=5.0):
    """The raw ASGI message: {'text': ...} or {'bytes': ...}."""
    return _call_with_timeout(ws.receive, timeout)


@contextlib.contextmanager
def connect(client, path, done=lambda: True):
    """TestClient cancels the app right after the ``with`` body ends: a handler that still needs a few event-loop
    iterations to wind down then fails the test with CancelledError (a test-client artefact, not a server bug).
    So close from our side and wait until ``done()`` (the server finished with this connection)."""
    with client.websocket_connect(path) as ws:
        try:
            yield ws
        finally:
            ws.close()
            for _ in range(400):
                if done():
                    break
                time.sleep(0.005)
            time.sleep(0.01)
