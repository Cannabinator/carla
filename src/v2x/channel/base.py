"""Channel-model interface and the per-link result record (REFACTOR_HANDOFF section 5.3)."""

from __future__ import annotations

from typing import NamedTuple, Optional, Protocol

# Why a message did not reach the application of the receiver
DROP_CHANNEL = "channel"                 # lost although the MAC factor was ignored (radio)
DROP_MAC = "mac"                         # radio would have delivered it; contention cost it
DROP_AWARENESS = "out_of_awareness_range"  # delivered by the radio, removed by the application filter

LINK_V2V, LINK_V2I, LINK_I2V, LINK_V2P, LINK_P2V = "V2V", "V2I", "I2V", "V2P", "P2V"


class LinkResult(NamedTuple):
    """Outcome of one transmission attempt tx -> rx of one message. All quantities are inputs or
    intermediate results of the computation that decided ``delivered`` (nothing is rounded).
    A NamedTuple: immutable and cheap to create (tens of thousands per update); use ``_replace``
    to derive a modified copy.

    ``delivered`` is the final outcome seen by the receiver's application. ``p_total`` is
    ``p_channel * p_mac``; one uniform draw ``u`` decides: delivered iff ``u < p_total``. A loss is
    attributed to the channel if ``u >= p_channel`` and to the MAC term if ``p_total <= u < p_channel``.
    """
    frame: int
    sim_time: float
    tx_id: int
    rx_id: int
    msg_type: str               # "BSM" | "SPAT" | "MAP" | "VRUAM" | "DENM" | "CPM"
    link_type: str              # "V2V" | "V2I" | "I2V" | "V2P" | "P2V"
    distance_m: float
    los: bool
    tx_power_dbm: float
    path_loss_db: float         # mean path loss (without the shadowing sample)
    shadowing_db: float         # the sample drawn for this packet
    rx_power_dbm: float
    rx_margin_db: float         # rx_power - sensitivity (not an SNR)
    p_channel: float
    p_mac: float
    p_total: float
    delivered: bool
    drop_cause: Optional[str]
    msg_id: Optional[int] = None   # distinguishes several messages of one type from one sender (DENM: action_id)


class ChannelModel(Protocol):
    """A radio model that decides, per link, whether a message is received."""

    def evaluate_link(
        self,
        tx_id: int,
        rx_id: int,
        distance_m: float,
        nlos: bool,
        n_contenders: int,
        *,
        msg_type: str = ...,
        link_type: str = ...,
        tx_power_dbm: Optional[float] = ...,
        frame: int = ...,
        sim_time: float = ...,
        msg_id: Optional[int] = ...,
    ) -> LinkResult: ...
