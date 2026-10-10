"""A small hand-built V2XFrame for hub tests: vehicles 101 and 102, RSU -1, one pedestrian 300."""

from src.v2x.channel.base import LinkResult
from src.v2x.frame import ChannelStats, ReceivedMessage, StationState, Threat, V2XFrame, count_links, frozen_map
from src.v2x.messages.bsm import BSMCore
from src.v2x.messages.denm import DENM, DENMCauseCode


def link(tx, rx, msg="BSM", lt="V2V", delivered=True, cause=None, dist=20.0, mid=None, frame=10):
    return LinkResult(frame, frame * 0.05, tx, rx, msg, lt, dist, True, 20.0, 80.0, 0.0, -60.0, 20.0, 1.0, 1.0, 1.0,
                      delivered, cause, mid)


def sample_frame(frame_no=10, extra_links=0):
    bsm1 = BSMCore(timestamp=0.5, msg_count=1, vehicle_id=101, x_m=0, y_m=0)
    bsm2 = BSMCore(timestamp=0.5, msg_count=1, vehicle_id=102, x_m=20, y_m=0)
    denm = DENM(station_id=-1, action_id=7, detection_time=0.0, reference_time=0.0, event_position=(5, 5, 0),
                cause_code=DENMCauseCode.ACCIDENT)
    l_12 = link(101, 102, frame=frame_no)
    l_21 = link(102, 101, delivered=False, cause="channel", frame=frame_no)
    l_r1 = link(-1, 101, msg="DENM", lt="I2V", mid=7, frame=frame_no)
    l_p1 = link(300, 101, msg="VRUAM", lt="P2V", frame=frame_no)
    links = [l_12, l_21, l_r1, l_p1] + [link(101, 5000 + i, frame=frame_no) for i in range(extra_links)]
    return V2XFrame(
        frame=frame_no, sim_time=frame_no * 0.05,
        stations=frozen_map({101: StationState(101, "OBU", 0, 0, 0, 0, 5, 20), 102: StationState(102, "OBU", 20, 0, 0, 0, 5, 20),
                             -1: StationState(-1, "RSU", 5, 5, 0), 300: StationState(300, "VRU", 8, 8, 0)}),
        sent=frozen_map({101: (bsm1,), 102: (bsm2,), -1: (denm,), 300: ()}),
        inbox=frozen_map({102: (ReceivedMessage(101, "BSM", bsm1, l_12),), 101: (ReceivedMessage(-1, "DENM", denm, l_r1),)}),
        links=tuple(links),
        threats=frozen_map({102: (Threat(101, 2, 3.0, 20.0),)}),
        stats=ChannelStats(frozen_map(count_links(links)), 0))
