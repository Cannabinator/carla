"""Fake CARLA sensor backend for LidarManager tests: no carla import, measurements produced on demand."""

from types import SimpleNamespace

import numpy as np

from src.lidar.config import SENSOR_SEMANTIC, LidarConfig


def make_measurement(frame: int, config: LidarConfig, n: int = 5, location=(1.0, 2.0, 3.0), rotation=(0.0, 90.0, 0.0)):
    if config.sensor_type == SENSOR_SEMANTIC:
        dtype = np.dtype([("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("c", "<f4"), ("o", "<u4"), ("t", "<u4")])
    else:
        dtype = np.dtype([("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("i", "<f4")])
    arr = np.zeros(n, dtype=dtype)
    arr["x"] = np.arange(n) + frame           # frame-dependent so mixing frames is detectable
    return SimpleNamespace(
        frame=frame, raw_data=arr.tobytes(),
        transform=SimpleNamespace(location=SimpleNamespace(x=location[0], y=location[1], z=location[2]),
                                  rotation=SimpleNamespace(pitch=rotation[0], yaw=rotation[1], roll=rotation[2])))


class FakeHandle:
    def __init__(self, backend, actor_id, config):
        self.backend, self.actor_id, self.config = backend, actor_id, config
        self.callback = None
        self.stopped = self.destroyed = False

    def listen(self, callback):
        self.callback = callback

    def stop(self):
        self.stopped = True
        self.callback = None

    def destroy(self):
        self.destroyed = True

    def emit(self, frame, n=5):
        if self.callback is not None:
            self.callback(make_measurement(frame, self.config, n))


class FakeBackend:
    def __init__(self):
        self.handles = []
        self.fail_next = None

    def spawn(self, actor_id, config):
        if self.fail_next:
            exc, self.fail_next = self.fail_next, None
            raise exc
        h = FakeHandle(self, actor_id, config)
        self.handles.append(h)
        return h

    @property
    def live(self):
        return [h for h in self.handles if not h.destroyed]

    def emit_all(self, frame, n=5):
        for h in self.live:
            h.emit(frame, n)
