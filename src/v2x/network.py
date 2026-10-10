"""
V2X Network Manager — extends V2V with V2I (RSU) and V2P (VRU) support.

Adds to V2VNetworkEnhanced:
  - RSU registry: broadcast MAP + SPAT each 2 Hz tick (V2I).
  - VRU registry: broadcast VRUAM for pedestrians and cyclists (V2P).

Both extensions are in-process and deterministic; no external broker required.
The existing BSM / DSRC / threat-assessment pipeline is unchanged.
"""

from __future__ import annotations

import math
import logging
from typing import Dict, Optional, Any

from ..v2v.network_enhanced import V2VNetworkEnhanced
from ..v2v.dsrc_channel import DSRCConfig
from .messages import (
    SPATMessage, MAPMessage,
    VRUAwarenessMessage, VRUProfile,
)
from .rsu import RoadSideUnit

logger = logging.getLogger(__name__)


class V2XNetworkEnhanced(V2VNetworkEnhanced):
    """
    V2X Network Manager — Vehicle-to-Everything.

    Inherits the full SAE J2735 BSM + IEEE 802.11p DSRC channel model from
    V2VNetworkEnhanced and adds:

    V2I (Vehicle-to-Infrastructure):
        MAP messages describing intersection geometry (static, cached).
        SPAT messages with live signal phase / timing from CARLA TrafficLights.

    V2P (Vehicle-to-Pedestrian):
        VRUAM beacons for pedestrian and cyclist actors (ETSI TS 103 300-3).

    Backward compatible: any code that uses V2VNetworkEnhanced works unchanged
    when switched to V2XNetworkEnhanced.
    """

    def __init__(
        self,
        max_range: float = 150.0,
        update_rate_hz: float = 2.0,
        enable_cooperative_perception: bool = True,
        world=None,
        dsrc_config: Optional[DSRCConfig] = None,
    ) -> None:
        super().__init__(
            max_range=max_range,
            update_rate_hz=update_rate_hz,
            enable_cooperative_perception=enable_cooperative_perception,
            world=world,
            dsrc_config=dsrc_config,
        )

        # V2I: RSU registry {intersection_id → RoadSideUnit}
        self._rsus: Dict[int, RoadSideUnit] = {}

        # Latest MAP and SPAT per intersection
        self.map_messages:  Dict[int, MAPMessage]  = {}
        self.spat_messages: Dict[int, SPATMessage] = {}

        # V2P: VRU actor registry {vru_id → actor}
        self._vrus: Dict[int, Any] = {}
        self._vru_profiles: Dict[int, VRUProfile] = {}
        self._vru_msg_counters: Dict[int, int] = {}

        # Latest VRUAM per VRU station
        self.vru_messages: Dict[int, VRUAwarenessMessage] = {}

        logger.info(
            "V2XNetworkEnhanced initialized (V2I RSU + V2P VRU extensions active)"
        )

    # ── RSU registration ───────────────────────────────────────────────────────

    def register_rsu(self, rsu: RoadSideUnit) -> None:
        """Register a Road Side Unit with the V2X network."""
        iid = rsu.config.intersection_id
        self._rsus[iid] = rsu
        logger.debug(
            "RSU registered: intersection_id=%d at (%.1f, %.1f)",
            iid, rsu.config.position[0], rsu.config.position[1],
        )

    def unregister_rsu(self, intersection_id: int) -> None:
        """Remove an RSU from the network."""
        self._rsus.pop(intersection_id, None)
        self.map_messages.pop(intersection_id, None)
        self.spat_messages.pop(intersection_id, None)

    # ── VRU registration ───────────────────────────────────────────────────────

    def register_vru(
        self,
        vru_id: int,
        actor,
        profile: VRUProfile = VRUProfile.PEDESTRIAN,
    ) -> None:
        """
        Register a Vulnerable Road User actor (pedestrian / cyclist).

        Args:
            vru_id:  Unique station ID.  Must not collide with vehicle IDs —
                     use an offset such as ``num_vehicles + index``.
            actor:   CARLA walker or two-wheeled vehicle actor.
            profile: VRUProfile classification.
        """
        self._vrus[vru_id] = actor
        self._vru_profiles[vru_id] = profile
        self._vru_msg_counters[vru_id] = 0
        logger.debug("VRU registered: id=%d profile=%s", vru_id, profile.name)

    def unregister_vru(self, vru_id: int) -> None:
        """Remove a VRU from the network."""
        self._vrus.pop(vru_id, None)
        self._vru_profiles.pop(vru_id, None)
        self._vru_msg_counters.pop(vru_id, None)
        self.vru_messages.pop(vru_id, None)

    # ── Update ─────────────────────────────────────────────────────────────────

    def update(self, snapshot=None, force: bool = False) -> bool:
        """
        V2X update — V2V BSM tick + RSU MAP/SPAT + VRU VRUAM.

        Inherits the 2 Hz gate from V2VNetworkEnhanced.update().
        Returns True only when an update was performed.
        """
        updated = super().update(snapshot=snapshot, force=force)
        if not updated:
            return False

        sim_time = self._last_sim_time()
        self._update_rsu(sim_time)
        self._update_vru(snapshot, sim_time)
        return True

    # ── Internal helpers ───────────────────────────────────────────────────────

    def _last_sim_time(self) -> float:
        """Return last known simulation time or fall back to wall clock."""
        t = self.last_update_sim_time
        if t is not None:
            return t
        import time
        return time.time()

    def _update_rsu(self, sim_time: float) -> None:
        """Generate MAP and SPAT from all registered RSUs."""
        for iid, rsu in self._rsus.items():
            self.map_messages[iid]  = rsu.get_map(sim_time)
            self.spat_messages[iid] = rsu.get_spat(sim_time)

    def _update_vru(self, snapshot, sim_time: float) -> None:
        """Build VRUAM for all registered VRU actors."""
        for vru_id, actor in list(self._vrus.items()):
            try:
                vruam = self._build_vruam(vru_id, actor, snapshot, sim_time)
                if vruam is not None:
                    self.vru_messages[vru_id] = vruam
                    self._vru_msg_counters[vru_id] = (
                        self._vru_msg_counters[vru_id] + 1
                    ) % 128
            except Exception as exc:
                logger.debug("VRU %d update error: %s", vru_id, exc)

    def _build_vruam(
        self,
        vru_id: int,
        actor,
        snapshot,
        sim_time: float,
    ) -> Optional[VRUAwarenessMessage]:
        """Extract kinematics from a CARLA actor and build a VRUAM."""
        if snapshot is not None:
            actor_snap = snapshot.find(actor.id)
            if actor_snap:
                tf  = actor_snap.get_transform()
                vel = actor_snap.get_velocity()
            else:
                tf  = actor.get_transform()
                vel = actor.get_velocity()
        else:
            tf  = actor.get_transform()
            vel = actor.get_velocity()

        speed        = math.sqrt(vel.x ** 2 + vel.y ** 2 + vel.z ** 2)
        heading      = tf.rotation.yaw % 360.0
        profile      = self._vru_profiles.get(vru_id, VRUProfile.PEDESTRIAN)
        motion_state = "stationary" if speed < 0.3 else "moving"

        return VRUAwarenessMessage(
            station_id=vru_id,
            timestamp=sim_time,
            vru_profile=profile,
            position=(tf.location.x, tf.location.y, tf.location.z),
            speed=speed,
            heading=heading,
            motion_state=motion_state,
        )

    # ── Public accessors ───────────────────────────────────────────────────────

    def get_spat(self, intersection_id: int) -> Optional[SPATMessage]:
        """Return the latest SPAT message for an intersection."""
        return self.spat_messages.get(intersection_id)

    def get_map(self, intersection_id: int) -> Optional[MAPMessage]:
        """Return the MAP message for an intersection."""
        return self.map_messages.get(intersection_id)

    def get_vruam(self, vru_id: int) -> Optional[VRUAwarenessMessage]:
        """Return the latest VRUAM for a VRU station."""
        return self.vru_messages.get(vru_id)

    def get_all_vruam(self) -> Dict[int, VRUAwarenessMessage]:
        """Return all current VRUAM messages keyed by station ID."""
        return self.vru_messages.copy()

    def get_v2x_summary(self) -> dict:
        """V2X statistics summary (extends V2V network stats)."""
        summary = self.get_network_stats()
        summary.update({
            "rsu_count":    len(self._rsus),
            "vru_count":    len(self._vrus),
            "active_spat":  len(self.spat_messages),
            "active_vruam": len(self.vru_messages),
        })
        return summary
