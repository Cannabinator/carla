import enum
import math
from dataclasses import dataclass
from typing import List

from src.server.published import build_v2x_summary, link_to_dict, message_to_dict, to_jsonable
from src.v2x.channel.base import LinkResult
from src.v2x.frame import ChannelStats, LinkCounts, ReceivedMessage, StationState, Threat, V2XFrame, frozen_map
from src.v2x.messages.bsm import BSMCore
from src.v2x.messages.denm import DENM, DENMCauseCode
from src.v2x.messages.spat import SPATMessage, SPATMovementState


class Color(enum.IntEnum):
    RED = 1


@dataclass
class Inner:
    c: Color
    xs: List[float]


def test_to_jsonable_plain_types():
    assert to_jsonable(Inner(Color.RED, [1.0, math.inf, math.nan])) == {"c": "RED", "xs": [1.0, None, None]}
    assert to_jsonable({1: (1, 2), "a": None}) == {"1": [1, 2], "a": None}
    np = __import__("numpy")
    assert to_jsonable(np.float32(1.5)) == 1.5 and to_jsonable(np.arange(3)) == [0, 1, 2]


def test_message_to_dict_names_the_type_and_is_json_serialisable():
    import json
    d = message_to_dict(BSMCore(timestamp=1.0, msg_count=3, vehicle_id=7, x_m=1.5))
    assert d["type"] == "BSMCore" and d["vehicle_id"] == 7 and d["vehicle_type"] == "PASSENGER_CAR"
    json.dumps(d)


def link(tx, rx, delivered=True, msg="BSM", lt="V2V", cause=None, mid=None):
    return LinkResult(1, 0.5, tx, rx, msg, lt, 10.0, True, 20.0, 80.0, 0.0, -60.0, 20.0, 1, 1, 1, delivered, cause, mid)


def test_link_to_dict_has_every_field():
    d = link_to_dict(link(1, 2))
    assert set(d) == set(LinkResult._fields) and d["delivered"] is True and d["msg_type"] == "BSM"


def test_v2x_summary():
    bsm = BSMCore(timestamp=0.5, msg_count=1, vehicle_id=1)
    denm = DENM(station_id=1, action_id=5, detection_time=0.0, reference_time=0.0, event_position=(1, 2, 0),
                cause_code=DENMCauseCode.ACCIDENT)
    spat = SPATMessage(intersection_id=3, intersection_name="x", timestamp=0.5,
                       states=[SPATMovementState(1, "stop-And-Remain")])
    l1 = link(1, 2)
    frame = V2XFrame(
        frame=10, sim_time=0.5,
        stations=frozen_map({1: StationState(1, "OBU", 0, 0, 0, 90, 5, 20), 2: StationState(2, "OBU", 10, 0, 0),
                             -3: StationState(-3, "RSU", 5, 5, 0)}),
        sent=frozen_map({1: (bsm, denm), 2: (), -3: (spat,)}),
        inbox=frozen_map({2: (ReceivedMessage(1, "BSM", bsm, l1),)}),
        links=(l1, link(1, 2, False, cause="channel")),
        threats=frozen_map({2: (Threat(1, 3, 1.2, 10.0), Threat(9, 1, 5.0, 40.0))}),
        stats=ChannelStats(frozen_map({"V2V": LinkCounts(2, 1, 1, 0, 0)}), 0))
    s = build_v2x_summary(frame)
    by = {x["station_id"]: x for x in s["stations"]}
    assert by[1]["sent"] == 2 and by[1]["received"] == 0 and by[2]["received"] == 1
    assert by[2]["threats"] == 2 and by[2]["max_threat_level"] == 3
    assert s["channel"]["V2V"]["delivered"] == 1 and s["links"] == 2
    assert [d["action_id"] for d in s["denms"]] == [5] and s["denms"][0]["cause_code"] == "ACCIDENT"
    assert s["rsu_states"]["-3"]["spat"]["states"][0]["movement_event"] == "stop-And-Remain"
