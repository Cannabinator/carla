"""Stream profiles and encode-once fan-out for LiDAR viewers (REFACTOR_HANDOFF 5.5).

A viewer asks for ``{max_points, max_hz, compress}``. The request is *quantised* so that many viewers share a
handful of profiles; a frame is encoded once per profile and the same bytes go to every viewer on it.

Subsampling is display only (recordings are raw). It uses a fixed seeded random key per point index, not a
stride: CARLA orders points by channel, so a stride produces ring artefacts. The kept points stay in their
original order.
"""

from __future__ import annotations

import threading
from dataclasses import dataclass
from typing import Any, Dict, Optional, Tuple

import numpy as np

from .protocol import FrameHeader, encode_frame
from .sensor import LidarFrame

POINT_STEPS: Tuple[int, ...] = (10_000, 25_000, 50_000)      # "all" is None
DEFAULT_SEED = 20240611


@dataclass(frozen=True)
class StreamProfile:
    max_points: Optional[int] = None        # None = every point
    max_hz: float = 10.0                    # effective rate; a divisor of the simulation rate
    compress: bool = False


def quantize_points(requested: Optional[int]) -> Optional[int]:
    """Largest step not exceeding the request (never more than asked for; below the smallest step: the smallest).
    None / 0 / negative mean "all points"."""
    if requested is None or requested <= 0:
        return None
    best = POINT_STEPS[0]
    for step in POINT_STEPS:
        if step <= requested:
            best = step
    return best


def frame_divisor(max_hz: float, sim_rate_hz: float) -> int:
    """Smallest integer N with sim_rate / N <= max_hz."""
    if max_hz <= 0:
        raise ValueError("max_hz must be positive")
    n = 1
    while sim_rate_hz / n > max_hz * (1 + 1e-9):
        n += 1
    return n


def quantize_profile(max_points: Optional[int], max_hz: float, compress: bool, sim_rate_hz: float) -> StreamProfile:
    n = frame_divisor(max_hz, sim_rate_hz)
    return StreamProfile(quantize_points(max_points), sim_rate_hz / n, bool(compress))


class Subsampler:
    """Deterministic random subsampling for one sensor."""

    def __init__(self, seed: int):
        self._rng = np.random.default_rng(seed)
        self._keys = np.empty(0, dtype=np.float64)

    def _ensure(self, n: int) -> None:
        if len(self._keys) < n:
            grown = max(n, 2 * len(self._keys), 4096)
            extra = self._rng.random(grown - len(self._keys))
            self._keys = np.concatenate([self._keys, extra])

    def indices(self, n: int, k: int) -> np.ndarray:
        """Sorted indices of k of n points: the k with the smallest fixed key (same n, k, seed -> same set)."""
        if k >= n:
            return np.arange(n)
        self._ensure(n)
        chosen = np.argpartition(self._keys[:n], k - 1)[:k]
        chosen.sort()
        return chosen


@dataclass(frozen=True)
class EncodedFrame:
    data: bytes
    frame: int
    points_total: int
    points_sent: int
    profile: StreamProfile

    @property
    def subsampled(self) -> bool:
        return self.points_sent < self.points_total


class SensorStream:
    """Encode-once cache for one sensor. Safe to call from several threads."""

    def __init__(self, sensor_id: int, sim_rate_hz: float, seed: int = DEFAULT_SEED):
        self.sensor_id = sensor_id
        self.sim_rate_hz = sim_rate_hz
        self._sub = Subsampler(seed + sensor_id)
        self._lock = threading.Lock()
        self._frame: Optional[int] = None
        self._cache: Dict[StreamProfile, EncodedFrame] = {}
        self.encodes = 0                     # for tests / HUD: how many encodes were actually done

    def divisor(self, profile: StreamProfile) -> int:
        return frame_divisor(profile.max_hz, self.sim_rate_hz)

    def wants(self, frame: LidarFrame, profile: StreamProfile) -> bool:
        return frame.frame % self.divisor(profile) == 0

    def encode(self, frame: LidarFrame, profile: StreamProfile) -> Optional[EncodedFrame]:
        """Bytes for ``profile`` or None when ``max_hz`` skips this frame."""
        if not self.wants(frame, profile):
            return None
        with self._lock:
            if self._frame != frame.frame:
                self._frame, self._cache = frame.frame, {}
            hit = self._cache.get(profile)
            if hit is not None:
                return hit
            total = frame.num_points
            points: Dict[str, Any] = frame.points
            if profile.max_points is not None and total > profile.max_points:
                idx = self._sub.indices(total, profile.max_points)
                points = {k: v[idx] for k, v in frame.points.items()}
            header = FrameHeader(frame=frame.frame, sim_time=frame.sim_time, sensor_id=frame.sensor_id,
                                 ego_actor_id=frame.ego_actor_id, location=frame.location,
                                 rotation_pyr=frame.rotation_pyr, sensor_frame=True)
            data = encode_frame(header, points, compress=profile.compress)
            enc = EncodedFrame(data, frame.frame, total, len(points["x"]), profile)
            self._cache[profile] = enc
            self.encodes += 1
            return enc
