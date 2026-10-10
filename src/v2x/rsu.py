"""
Road Side Unit (RSU) — V2I Infrastructure Node

Implements a stationary ITS station that:
  - Generates MAP messages describing intersection geometry.
  - Reads CARLA TrafficLight state and produces live SPAT messages.

The RSU transmits at the same 2 Hz cadence as the vehicles' BSM beacons.
RSUConfig defaults to 33 dBm TX power (vs. 23 dBm for OBUs) and a 300 m radius; the attribution
to ETSI TR 101 607 is UNVERIFIED and the TX power is not used by any computation yet (S4).

IEEE / ETSI references:
  ETSI TS 103 301 v2.1.1  — MAP + SPAT (infrastructure services)
  SAE J2735-202007        — SPaTIntersectionState (section number unverified)
  ETSI TR 101 607 v1.1.1  — ITS-G5 RSU deployment guidelines
"""

from __future__ import annotations

import logging
from dataclasses import dataclass
from typing import Optional, List, Dict, Any

from .messages import MAPMessage, SPATMessage, SPATMovementState

logger = logging.getLogger(__name__)


# ── CARLA traffic-light state → SAE J2735 MovementPhaseState ────────────────

_CARLA_STATE_MAP: Dict[Any, str] = {}


def _build_carla_state_map() -> Dict[Any, str]:
    """
    Lazily build the CARLA TrafficLightState → SAE J2735 MovementPhaseState map.

    Imported lazily to avoid a hard CARLA dependency at module import time
    (important for unit tests that run without CARLA installed).
    """
    if _CARLA_STATE_MAP:
        return _CARLA_STATE_MAP
    try:
        import carla
        _CARLA_STATE_MAP.update({
            carla.TrafficLightState.Green:   "protectedMovementAllowed",
            carla.TrafficLightState.Yellow:  "protectedClearance",
            carla.TrafficLightState.Red:     "stop-And-Remain",
            carla.TrafficLightState.Off:     "caution-Conflicting-Traffic",
            carla.TrafficLightState.Unknown: "unavailable",
        })
    except (ImportError, AttributeError):
        pass
    return _CARLA_STATE_MAP


# ── RSU configuration ─────────────────────────────────────────────────────────

@dataclass
class RSUConfig:
    """
    Road Side Unit configuration.

    Defaults follow ETSI TR 101 607 v1.1.1 for ITS-G5 RSU deployments:
      TX power 33 dBm (higher than OBU 23 dBm to cover the full intersection).
      Broadcast range 300 m — typical urban intersection sight distance.
    """
    intersection_id: int
    position: tuple                       # (x, y, z) CARLA world coordinates
    tx_power_dbm: float = 33.0            # ETSI TR 101 607 RSU max
    broadcast_range_m: float = 300.0      # Intersection coverage radius (m)
    map_revision: int = 1


# ── Road Side Unit ─────────────────────────────────────────────────────────────

class RoadSideUnit:
    """
    Stationary V2I infrastructure node.

    Wraps one or more CARLA TrafficLight actors at an intersection and produces
    MAP / SPAT messages that are injected into the V2X network each 2 Hz tick
    via V2XNetwork.register_rsu().

    Usage::

        rsu = RoadSideUnit.from_traffic_light(tl_actor, intersection_id=1)
        v2x_network.register_rsu(rsu)
        # V2XNetwork.update() broadcasts MAP/SPAT automatically.
    """

    def __init__(
        self,
        config: RSUConfig,
        traffic_lights: Optional[List] = None,
    ) -> None:
        self.config = config
        self._traffic_lights: List = traffic_lights or []
        self._map_cache: Optional[MAPMessage] = None

        logger.debug(
            "RSU created: intersection_id=%d, pos=(%.1f, %.1f, %.1f), "
            "TX=%.0f dBm, range=%.0f m",
            config.intersection_id,
            *config.position,
            config.tx_power_dbm,
            config.broadcast_range_m,
        )

    # ── Factory ───────────────────────────────────────────────────────────────

    @classmethod
    def from_traffic_light(
        cls,
        traffic_light,
        intersection_id: int,
        broadcast_range_m: float = 300.0,
    ) -> "RoadSideUnit":
        """
        Build an RSU positioned at a single CARLA TrafficLight actor.

        Args:
            traffic_light:    CARLA TrafficLight actor.
            intersection_id:  Unique intersection identifier.
            broadcast_range_m: RSU coverage radius in metres.
        """
        loc = traffic_light.get_transform().location
        config = RSUConfig(
            intersection_id=intersection_id,
            position=(loc.x, loc.y, loc.z),
            broadcast_range_m=broadcast_range_m,
        )
        return cls(config, traffic_lights=[traffic_light])

    @classmethod
    def from_traffic_lights(
        cls,
        traffic_lights: List,
        intersection_id: int,
        broadcast_range_m: float = 300.0,
    ) -> "RoadSideUnit":
        """
        Build ONE RSU for an intersection from all of its CARLA TrafficLight actors (S7).

        The RSU sits at the mean position of the lights. Signal group ids are the lights'
        indices in ascending actor-id order, so they are stable between runs.
        """
        lights = sorted(traffic_lights, key=lambda tl: tl.id)
        locs = [tl.get_transform().location for tl in lights]
        n = len(locs)
        config = RSUConfig(
            intersection_id=intersection_id,
            position=(sum(l.x for l in locs) / n, sum(l.y for l in locs) / n, sum(l.z for l in locs) / n),
            broadcast_range_m=broadcast_range_m,
        )
        return cls(config, traffic_lights=lights)

    # ── MAP generation ─────────────────────────────────────────────────────────

    def get_map(self, timestamp: float) -> MAPMessage:
        """
        Return the static MAP message for this intersection.

        The MAP is generated once and cached; only the timestamp is refreshed
        each call.  Geometry changes would require a map_revision bump (not
        modelled dynamically in CARLA scenarios).
        """
        if self._map_cache is None:
            self._map_cache = MAPMessage(
                intersection_id=self.config.intersection_id,
                revision=self.config.map_revision,
                ref_point=self.config.position,
                timestamp=timestamp,
            )
        self._map_cache.timestamp = timestamp
        return self._map_cache

    # ── SPAT generation ────────────────────────────────────────────────────────

    def get_spat(self, timestamp: float) -> SPATMessage:
        """
        Build a live SPAT from current CARLA TrafficLight state(s).

        Each registered TrafficLight actor contributes one SPATMovementState.
        Phase timing is derived from CARLA's elapsed / phase-duration API; when it cannot be
        determined the three end-time fields are None.
        """
        state_map = _build_carla_state_map()
        states: List[SPATMovementState] = []

        for sg_id, tl in enumerate(self._traffic_lights):
            try:
                tl_state = tl.get_state()
                movement_event = state_map.get(tl_state, "unavailable")
                remaining = self._remaining_time(tl)

                # CARLA's signal program is exact, so a known remaining time has no uncertainty
                # window; an unknown one is reported as unknown (None), never as a guess (S17).
                end_time = None if remaining is None else timestamp + remaining
                states.append(SPATMovementState(
                    signal_group_id=sg_id,
                    movement_event=movement_event,
                    min_end_time_s=end_time,
                    max_end_time_s=end_time,
                    likely_time_s=end_time,
                ))
            except Exception as exc:
                logger.debug(
                    "RSU %d: failed to read TL state for signal group %d: %s",
                    self.config.intersection_id, sg_id, exc,
                )

        return SPATMessage(
            intersection_id=self.config.intersection_id,
            intersection_name=f"intersection_{self.config.intersection_id}",
            timestamp=timestamp,
            states=states,
        )

    # ── helpers ───────────────────────────────────────────────────────────────

    @staticmethod
    def _remaining_time(traffic_light) -> Optional[float]:
        """Remaining time of the current phase in seconds, or None if it cannot be determined."""
        try:
            import carla
            state = traffic_light.get_state()
            elapsed = traffic_light.get_elapsed_time()
            if state == carla.TrafficLightState.Green:
                return max(traffic_light.get_green_time() - elapsed, 0.0)
            elif state == carla.TrafficLightState.Yellow:
                return max(traffic_light.get_yellow_time() - elapsed, 0.0)
            elif state == carla.TrafficLightState.Red:
                return max(traffic_light.get_red_time() - elapsed, 0.0)
        except Exception as exc:
            logger.debug("Could not read traffic light timing: %s", exc)
        return None  # Off / Unknown state, or the query failed


def group_traffic_lights(traffic_lights: List) -> List[List]:
    """
    Partition traffic lights into intersections using CARLA's own light groups.

    ``TrafficLight.get_group_traffic_lights()`` lists the lights that switch together. Lights are
    merged transitively (union-find), so an asymmetric answer cannot split an intersection, and a
    light whose group query fails becomes its own group. The result is deterministic: lights inside
    a group are sorted by actor id and groups by their smallest actor id.
    TODO(verify): get_group_traffic_lights() semantics on 0.9.16 (does it include the light itself,
    does a group correspond to one physical intersection) - run on a live server.
    """
    parent: Dict[int, int] = {}
    actors: Dict[int, Any] = {}

    def find(x: int) -> int:
        while parent[x] != x:
            parent[x] = parent[parent[x]]
            x = parent[x]
        return x

    def add(tl) -> None:
        if tl.id not in parent:
            parent[tl.id] = tl.id
            actors[tl.id] = tl

    for tl in traffic_lights:
        add(tl)
    for tl in traffic_lights:
        try:
            members = list(tl.get_group_traffic_lights())
        except Exception as exc:
            logger.debug("traffic light %s: group query failed: %s", tl.id, exc)
            members = []
        for m in members:
            add(m)
            ra, rb = find(tl.id), find(m.id)
            if ra != rb:
                parent[max(ra, rb)] = min(ra, rb)

    groups: Dict[int, List] = {}
    for tid, tl in actors.items():
        groups.setdefault(find(tid), []).append(tl)
    return [sorted(groups[root], key=lambda t: t.id) for root in sorted(groups)]
