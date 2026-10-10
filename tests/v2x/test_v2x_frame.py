"""V2XFrame: the immutable record of one V2X update (section 5.3)."""

import dataclasses
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).parent.parent.parent))

from src.v2x.channel.base import DROP_AWARENESS
from src.v2x.frame import KIND_OBU, KIND_RSU, KIND_VRU, rsu_station_id
from src.v2x.network import V2XNetwork
from src.v2x.rsu import RoadSideUnit, RSUConfig
from tests.v2x.test_network_basic import MockVector, MockVehicle, MockWorld, MockWorldSnapshot, _transparent_dsrc


def _net(positions, awareness=None, **kw):
    world = MockWorld()
    vehicles = [MockVehicle(i + 1, x=x, y=y) for i, (x, y) in enumerate(positions)]
    for v in vehicles:
        world.add_vehicle(v)
    net = V2XNetwork(awareness_range_m=awareness, dsrc_config=_transparent_dsrc(), world=world, **kw)
    for v in vehicles:
        net.register(v.id, v)
    return net, vehicles


def _update(net, vehicles, frame=100, t=5.0):
    return net.update(snapshot=MockWorldSnapshot(vehicles, elapsed_seconds=t, frame=frame), force=True)


def test_no_frame_before_the_first_update():
    net, _ = _net([(0, 0), (10, 0)])
    assert net.get_frame() is None


def test_frame_carries_frame_number_time_stations_and_links():
    net, vs = _net([(0, 0), (30, 0), (60, 0)])
    assert _update(net, vs, frame=321, t=16.05)
    f = net.get_frame()
    assert (f.frame, f.sim_time) == (321, 16.05)
    assert set(f.stations) == {1, 2, 3} and all(s.kind == KIND_OBU for s in f.stations.values())
    assert f.stations[2].x_m == 30.0
    assert len(f.links) == 6 and all(l.frame == 321 and l.sim_time == 16.05 for l in f.links)
    assert set(f.sent) == {1, 2, 3}


def test_inbox_equals_the_neighbour_tables():
    net, vs = _net([(0, 0), (50, 0), (200, 0), (210, 0)], awareness=75.0)
    _update(net, vs)
    f = net.get_frame()
    for v in vs:
        from_inbox = sorted(m.tx_id for m in f.inbox.get(v.id, ()) if m.msg_type == "BSM")
        assert from_inbox == sorted(n.vehicle_id for n in net.get_neighbors(v.id)), v.id


def test_out_of_awareness_range_is_recorded_as_the_drop_cause_but_ground_truth_still_lists_the_station():
    net, vs = _net([(0, 0), (50, 0), (200, 0)], awareness=75.0)
    _update(net, vs)
    f = net.get_frame()
    far = [l for l in f.links if {l.tx_id, l.rx_id} == {1, 3}]
    assert len(far) == 2 and all(not l.delivered and l.drop_cause == DROP_AWARENESS for l in far)
    assert 3 in f.stations                                    # ground truth knows it
    assert all(m.tx_id != 3 for m in f.inbox.get(1, ()))     # but vehicle 1 did not receive it
    counts = f.stats.by_link_type["V2V"]
    assert counts.total == 6 and counts.awareness_drops == 4 and counts.delivered == 2


def test_no_awareness_limit_means_the_radio_alone_decides():
    net, vs = _net([(0, 0), (500, 0)], awareness=None)
    _update(net, vs)
    f = net.get_frame()
    assert all(l.delivered for l in f.links)   # transparent channel, no filter


def test_threats_in_the_frame_are_those_of_the_network():
    world = MockWorld()
    v1 = MockVehicle(1, x=0.0, y=0.0)      # head-on, 50 m apart, closing at 20 m/s
    v2 = MockVehicle(2, x=50.0, y=0.0)
    v1._velocity, v2._velocity = MockVector(10, 0, 0), MockVector(-10, 0, 0)
    for v in (v1, v2):
        world.add_vehicle(v)
    net = V2XNetwork(awareness_range_m=100.0, dsrc_config=_transparent_dsrc(), world=world)
    net.register(1, v1)
    net.register(2, v2)
    _update(net, [v1, v2])
    f = net.get_frame()
    assert net.threats, "scenario must produce threats, otherwise this test proves nothing"
    assert set(f.threats) == {1, 2}
    for (ego, other), t in net.threats.items():
        match = [x for x in f.threats[ego] if x.other_id == other]
        assert len(match) == 1 and match[0].level == t["level"] and match[0].distance_m == t["distance"]
        assert match[0].ttc_s == pytest.approx(2.5) and match[0].level == 3   # 50 m / 20 m/s


def test_frame_is_immutable_and_later_updates_do_not_change_it():
    net, vs = _net([(0, 0), (30, 0)])
    _update(net, vs, frame=1, t=0.5)
    first = net.get_frame()
    links_before = first.links
    with pytest.raises(dataclasses.FrozenInstanceError):
        first.frame = 5
    with pytest.raises(TypeError):
        first.stations[99] = None
    with pytest.raises(TypeError):
        first.inbox[1] = ()
    _update(net, vs, frame=11, t=1.0)
    second = net.get_frame()
    assert second is not first and second.frame == 11 and first.frame == 1 and first.links is links_before


def test_a_station_missing_from_the_snapshot_is_not_in_the_frame_and_is_counted():
    net, vs = _net([(0, 0), (30, 0), (60, 0)])
    net.update(snapshot=MockWorldSnapshot(vs[:2], elapsed_seconds=1.0, frame=5), force=True)
    f = net.get_frame()
    assert set(f.stations) == {1, 2} and f.stats.skipped_not_in_snapshot == 1
    assert len(f.links) == 2


def test_a_gated_update_leaves_the_previous_frame_in_place():
    net, vs = _net([(0, 0), (30, 0)], update_every_n_frames=10)
    net.update(snapshot=MockWorldSnapshot(vs, elapsed_seconds=0.5, frame=10), force=False)
    first = net.get_frame()
    assert net.update(snapshot=MockWorldSnapshot(vs, elapsed_seconds=0.55, frame=11), force=False) is False
    assert net.get_frame() is first


def test_rsu_and_vru_are_stations_with_their_own_kind_and_negative_rsu_ids():
    net, vs = _net([(0, 0)])
    rsu = RoadSideUnit(RSUConfig(intersection_id=4, position=(10.0, 5.0, 0.0)), traffic_lights=[])
    net.register_rsu(rsu)
    _update(net, vs)
    f = net.get_frame()
    sid = rsu_station_id(4)
    assert sid == -4 and f.stations[sid].kind == KIND_RSU and f.stations[sid].tx_power_dbm == 33.0
    assert f.stations[sid].x_m == 10.0
    assert KIND_VRU == "VRU"
