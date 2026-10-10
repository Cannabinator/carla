"""CARLA implementation of the SensorBackend interface (simulation thread only)."""

from __future__ import annotations

import logging
from typing import Any, Callable

from .config import LidarConfig

logger = logging.getLogger(__name__)


class CarlaSensorHandle:
    def __init__(self, sensor: Any):
        self._sensor = sensor

    def listen(self, callback: Callable[[Any], None]) -> None:
        self._sensor.listen(callback)

    def stop(self) -> None:
        self._sensor.stop()

    def destroy(self) -> None:
        self._sensor.destroy()


class CarlaSensorBackend:
    def __init__(self, carla_module: Any, world: Any):
        self._carla = carla_module
        self._world = world

    def spawn(self, actor_id: int, config: LidarConfig) -> CarlaSensorHandle:
        carla = self._carla
        parent = self._world.get_actor(actor_id)
        if parent is None:
            raise RuntimeError(f"actor {actor_id} does not exist")
        bp = self._world.get_blueprint_library().find(config.blueprint_id)
        for name, value in config.carla_attributes().items():
            bp.set_attribute(name, value)
        m = config.mount
        transform = carla.Transform(carla.Location(x=m.x_m, y=m.y_m, z=m.z_m),
                                    carla.Rotation(pitch=m.pitch_deg, yaw=m.yaw_deg, roll=m.roll_deg))
        sensor = self._world.spawn_actor(bp, transform, attach_to=parent)
        return CarlaSensorHandle(sensor)
