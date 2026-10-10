"""The immutable result of one V2X update (REFACTOR_HANDOFF section 5.3).

A ``V2XFrame`` is built once per update and never changed afterwards, so any thread can hold and
read it without locks. Ground truth (``stations``) and what each station actually received
(``inbox``) are separate on purpose: a vehicle's V2X view is its inbox, not the ground truth.

Station ids: vehicles and pedestrians use their CARLA actor id; an RSU uses ``-intersection_id``
(negative, so it cannot collide with actor ids).
"""

from __future__ import annotations

from dataclasses import dataclass
from types import MappingProxyType
from typing import Any, Mapping, NamedTuple, Optional, Tuple

from .channel.base import DROP_AWARENESS, DROP_CHANNEL, DROP_MAC, LinkResult

KIND_OBU, KIND_RSU, KIND_VRU = "OBU", "RSU", "VRU"


def rsu_station_id(intersection_id: int) -> int:
    return -int(intersection_id)


@dataclass(frozen=True)
class StationState:
    """Ground truth of one station at the frame's simulation time."""
    station_id: int
    kind: str                       # "OBU" | "RSU" | "VRU"
    x_m: float
    y_m: float
    z_m: float
    heading_deg: float = 0.0
    speed_mps: float = 0.0
    tx_power_dbm: Optional[float] = None


class ReceivedMessage(NamedTuple):
    """A message that reached a receiver's application, with the link outcome that delivered it."""
    tx_id: int
    msg_type: str
    message: Any
    link: LinkResult


class Threat(NamedTuple):
    """Collision threat of ``other_id`` as seen by a receiver, computed from the receiver's inbox."""
    other_id: int
    level: int
    ttc_s: float
    distance_m: float


class LinkCounts(NamedTuple):
    total: int
    delivered: int
    channel_drops: int
    mac_drops: int
    awareness_drops: int


@dataclass(frozen=True)
class ChannelStats:
    """Link outcomes of THIS frame (not cumulative), per link type."""
    by_link_type: Mapping[str, LinkCounts]
    skipped_not_in_snapshot: int = 0


@dataclass(frozen=True)
class V2XFrame:
    frame: int
    sim_time: float
    stations: Mapping[int, StationState]               # ground truth
    sent: Mapping[int, Tuple[Any, ...]]                # tx station -> messages it transmitted
    inbox: Mapping[int, Tuple[ReceivedMessage, ...]]   # rx station -> what it received
    links: Tuple[LinkResult, ...]                      # every attempt of this frame
    threats: Mapping[int, Tuple[Threat, ...]]          # rx station -> threats from its inbox
    stats: ChannelStats


def count_links(links) -> dict:
    """Group link outcomes by link type."""
    acc: dict = {}
    for l in links:
        c = acc.setdefault(l.link_type, [0, 0, 0, 0, 0])
        c[0] += 1
        if l.delivered:
            c[1] += 1
        elif l.drop_cause == DROP_CHANNEL:
            c[2] += 1
        elif l.drop_cause == DROP_MAC:
            c[3] += 1
        elif l.drop_cause == DROP_AWARENESS:
            c[4] += 1
    return {k: LinkCounts(*v) for k, v in acc.items()}


def frozen_map(d: dict) -> Mapping:
    return MappingProxyType(d)
