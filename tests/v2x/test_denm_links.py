"""S4: DENM hazard warnings are transmitted through the radio model."""

import sys
from pathlib import Path


sys.path.insert(0, str(Path(__file__).parent.parent.parent))

from src.v2x.channel.dsrc_logdistance import DSRCConfig
from src.v2x.messages import DENM, DENMCauseCode
from tests.v2x.test_infrastructure_links import build, of_type, step


def denm(station, action=1, event=(100.0, 0.0, 0.0), relevance=150.0):
    return DENM(station_id=station, action_id=action, detection_time=1.0, reference_time=1.0,
                event_position=event, cause_code=DENMCauseCode.ACCIDENT, relevance_distance=relevance)



def test_denm_from_a_vehicle_reaches_vehicles_in_the_relevance_area_as_a_v2v_link():
    net, vs = build([0, 90, 400])            # vehicle 1 detects a hazard at x=100
    net.publish_denm(denm(1, event=(100.0, 0.0, 0.0), relevance=150.0))
    f = step(net, vs)
    links = [l for l in f.links if l.msg_type == "DENM"]
    assert sorted((l.tx_id, l.rx_id, l.link_type, l.msg_id) for l in links) == [(1, 2, "V2V", 1)]   # 400 m is outside 150 m of the event
    got = net.get_received(2, "DENM")
    assert len(got) == 1 and got[0].message.cause_code == DENMCauseCode.ACCIDENT
    assert net.get_received(3, "DENM") == [] and net.get_received(1, "DENM") == []   # originator excluded


def test_denm_from_an_rsu_uses_its_tx_power_and_is_an_i2v_link():
    net, vs = build([30, 60], rsus=[(2, (0.0, 0.0), 33.0)])
    net.publish_denm(denm(-2, event=(50.0, 0.0, 0.0), relevance=100.0))
    links = [l for l in step(net, vs).links if l.msg_type == "DENM"]
    assert {l.rx_id for l in links} == {1, 2} and all(l.tx_power_dbm == 33.0 and l.link_type == "I2V" for l in links)


def test_denm_from_an_unregistered_station_is_sent_from_the_event_position():
    net, vs = build([20, 400])
    net.publish_denm(denm(9001, event=(0.0, 0.0, 0.0), relevance=100.0))
    f = step(net, vs)
    links = [l for l in f.links if l.msg_type == "DENM"]
    assert [(l.tx_id, l.rx_id, l.distance_m) for l in links] == [(9001, 1, 20.0)]
    assert f.stations[9001].x_m == 0.0 and f.sent[9001][0].action_id == 1


def test_several_denms_from_one_station_are_told_apart_by_msg_id():
    net, vs = build([0, 50])
    net.publish_denm(denm(1, action=1, event=(25.0, 0.0, 0.0)))
    net.publish_denm(denm(1, action=2, event=(25.0, 0.0, 0.0)))
    got = net.get_received(2, "DENM")
    assert got == []                                         # nothing transmitted before an update
    step(net, vs)
    got = net.get_received(2, "DENM")
    assert sorted(m.message.action_id for m in got) == [1, 2]
    assert sorted(m.link.msg_id for m in got) == [1, 2]


def test_cancelled_denms_stop_being_transmitted():
    net, vs = build([0, 50])
    net.publish_denm(denm(1, event=(25.0, 0.0, 0.0)))
    step(net, vs, frame=10, t=1.0)
    assert net.cancel_denm(1, 1)
    f = step(net, vs, frame=20, t=2.0)
    assert [l for l in f.links if l.msg_type == "DENM"] == [] and net.get_received(2, "DENM") == []


def test_denm_traffic_does_not_change_any_vehicle_to_vehicle_outcome():
    xs = [0, 40, 85, 130]
    plain, v1 = build(xs, dsrc=DSRCConfig(random_seed=3))
    warned, v2 = build(xs, dsrc=DSRCConfig(random_seed=3))
    warned.publish_denm(denm(1, event=(60.0, 0.0, 0.0), relevance=200.0))
    for i in range(5):
        a, b = step(plain, v1, frame=10 * i, t=0.5 * i), step(warned, v2, frame=10 * i, t=0.5 * i)
        assert [(l.tx_id, l.rx_id, l.delivered, l.shadowing_db) for l in of_type(a, "V2V") if l.msg_type == "BSM"] == \
               [(l.tx_id, l.rx_id, l.delivered, l.shadowing_db) for l in of_type(b, "V2V") if l.msg_type == "BSM"]
    assert any(l.msg_type == "DENM" for l in b.links)


def test_a_denm_loss_has_a_cause_on_a_marginal_link():
    net, vs = build([0, 260], dsrc=DSRCConfig(random_seed=1, tx_power_dbm=12.0))
    net.publish_denm(denm(1, event=(130.0, 0.0, 0.0), relevance=200.0))
    lost = []
    for i in range(80):
        lost += [l for l in step(net, vs, frame=10 + i, t=1.0 + i).links if l.msg_type == "DENM" and not l.delivered]
    assert lost and all(l.drop_cause in ("channel", "mac") for l in lost)
