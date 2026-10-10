"""S4: RSU (MAP, SPAT) and pedestrian (VRUAM) messages go through the radio model."""

import sys
from pathlib import Path
from types import SimpleNamespace as NS

import pytest

sys.path.insert(0, str(Path(__file__).parent.parent.parent))

from src.v2x.channel.dsrc_logdistance import DSRCConfig
from src.v2x.messages import SPATMessage, VRUAwarenessMessage, VRUProfile
from src.v2x.network import V2XNetwork
from src.v2x.rsu import RoadSideUnit, RSUConfig
from tests.v2x.test_network_basic import MockVehicle, MockWorld, MockWorldSnapshot, _transparent_dsrc


class Walker:
    def __init__(self, wid, x, y, speed=1.0):
        self.id, self._x, self._y, self._v = wid, x, y, speed

    def get_transform(self):
        return NS(location=NS(x=self._x, y=self._y, z=0.0), rotation=NS(pitch=0.0, yaw=90.0, roll=0.0))

    def get_velocity(self):
        return NS(x=0.0, y=self._v, z=0.0)


def build(vehicle_xs, rsus=(), walkers=(), dsrc=None, **kw):
    world = MockWorld()
    vs = [MockVehicle(i + 1, x=x, y=0) for i, x in enumerate(vehicle_xs)]
    for v in vs:
        world.add_vehicle(v)
    net = V2XNetwork(awareness_range_m=None, dsrc_config=dsrc or _transparent_dsrc(), world=world, **kw)
    for v in vs:
        net.register(v.id, v)
    for iid, (x, y), power in rsus:
        net.register_rsu(RoadSideUnit(RSUConfig(intersection_id=iid, position=(x, y, 0.0), tx_power_dbm=power),
                                      traffic_lights=[]))
    for w in walkers:
        net.register_vru(w.id, w, VRUProfile.PEDESTRIAN)
    return net, vs


def step(net, vs, frame=10, t=5.0):
    assert net.update(snapshot=MockWorldSnapshot(vs, elapsed_seconds=t, frame=frame), force=True)
    return net.get_frame()


def of_type(f, link_type):
    return [l for l in f.links if l.link_type == link_type]


# ── RSU -> vehicles ────────────────────────────────────────────────────────────

def test_rsu_sends_map_and_spat_to_vehicles_inside_its_coverage_only():
    net, vs = build([50, 400], rsus=[(1, (0.0, 0.0), 33.0)])
    f = step(net, vs)
    links = of_type(f, "I2V")
    assert sorted((l.rx_id, l.msg_type) for l in links) == [(1, "MAP"), (1, "SPAT")]
    assert all(l.tx_id == -1 and l.tx_power_dbm == 33.0 and l.delivered for l in links)
    got = net.get_received(1)
    assert sorted(m.msg_type for m in got) == ["BSM", "MAP", "SPAT"]   # + the other vehicle's BSM
    assert isinstance(net.get_received(1, "SPAT")[0].message, SPATMessage)
    assert net.get_received(2, "SPAT") == []                   # 400 m: outside the 300 m coverage


def test_coverage_boundary_is_inclusive():
    net, vs = build([300.0], rsus=[(1, (0.0, 0.0), 33.0)])
    assert len(of_type(step(net, vs), "I2V")) == 2
    net2, vs2 = build([300.01], rsus=[(1, (0.0, 0.0), 33.0)])
    assert of_type(step(net2, vs2), "I2V") == []


def test_a_vehicle_between_the_rsu_and_the_receiver_makes_the_link_nlos():
    net, vs = build([50, 100], rsus=[(1, (0.0, 0.0), 33.0)], dsrc=DSRCConfig(random_seed=1))  # NLOS model on
    by_rx = {l.rx_id: l for l in of_type(step(net, vs), "I2V") if l.msg_type == "SPAT"}
    assert by_rx[1].los is True                 # nothing between the RSU and vehicle 1
    assert by_rx[2].los is False                # vehicle 1 (x=50) sits on the segment to vehicle 2


def test_rsu_tx_power_is_used_and_shifts_rx_power_one_to_one():
    cfg = DSRCConfig(random_seed=5)
    a, vs_a = build([80], rsus=[(1, (0.0, 0.0), 33.0)], dsrc=cfg)
    b, vs_b = build([80], rsus=[(1, (0.0, 0.0), 23.0)], dsrc=DSRCConfig(random_seed=5))
    la = [l for l in of_type(step(a, vs_a), "I2V") if l.msg_type == "SPAT"][0]
    lb = [l for l in of_type(step(b, vs_b), "I2V") if l.msg_type == "SPAT"][0]
    assert la.rx_power_dbm - lb.rx_power_dbm == pytest.approx(10.0)
    assert la.shadowing_db == lb.shadowing_db and la.path_loss_db == lb.path_loss_db


def test_channel_counters_by_link_type_and_frame_stats():
    net, vs = build([50, 100], rsus=[(1, (0.0, 0.0), 33.0)])
    f = step(net, vs)
    assert net._dsrc_channel.stats_by_link_type["I2V"]["total"] == 4
    assert f.stats.by_link_type["I2V"].total == 4 and f.stats.by_link_type["V2V"].total == 2
    assert f.stats.by_link_type["I2V"].delivered == 4


def test_a_lossy_link_can_drop_infrastructure_messages_with_a_cause():
    # 280 m, NLOS-free but weak: some of many attempts must fail, each with a recorded cause
    net, vs = build([280], rsus=[(1, (0.0, 0.0), 10.0)], dsrc=DSRCConfig(random_seed=2))
    drops = []
    for i in range(80):
        drops += [l for l in of_type(step(net, vs, frame=10 + i, t=5.0 + i), "I2V") if not l.delivered]
    assert drops and all(l.drop_cause in ("channel", "mac") for l in drops)


# ── independence of the random streams ─────────────────────────────────────────

def test_rsus_and_pedestrians_do_not_change_any_vehicle_to_vehicle_outcome():
    xs = [0, 40, 85, 130, 170]
    plain, vs1 = build(xs, dsrc=DSRCConfig(random_seed=9))
    busy, vs2 = build(xs, rsus=[(1, (20.0, 5.0), 33.0), (2, (150.0, 5.0), 33.0)],
                      walkers=[Walker(900, 60.0, 10.0), Walker(901, 100.0, -10.0)], dsrc=DSRCConfig(random_seed=9))
    for i in range(6):
        fa, fb = step(plain, vs1, frame=10 * i, t=0.5 * i), step(busy, vs2, frame=10 * i, t=0.5 * i)
        va, vb = of_type(fa, "V2V"), of_type(fb, "V2V")
        assert [(l.tx_id, l.rx_id, l.delivered, l.shadowing_db) for l in va] == \
               [(l.tx_id, l.rx_id, l.delivered, l.shadowing_db) for l in vb]
    assert of_type(fb, "I2V") and of_type(fb, "P2V")        # and the extra traffic did exist


def test_infrastructure_links_are_reproducible():
    def run():
        net, vs = build([60, 120], rsus=[(1, (0.0, 0.0), 33.0)], walkers=[Walker(900, 90.0, 5.0)],
                        dsrc=DSRCConfig(random_seed=4))
        return [step(net, vs, frame=10 * i, t=0.5 * i).links for i in range(4)]
    assert run() == run()


# ── pedestrians -> vehicles ────────────────────────────────────────────────────

def test_pedestrian_vruam_reaches_vehicles_within_100_m_with_the_vru_tx_power():
    net, vs = build([40, 160], walkers=[Walker(900, 20.0, 0.0)])
    f = step(net, vs)
    p2v = of_type(f, "P2V")
    assert [(l.tx_id, l.rx_id, l.msg_type, l.tx_power_dbm) for l in p2v] == [(900, 1, "VRUAM", 23.0)]
    assert isinstance(net.get_received(1, "VRUAM")[0].message, VRUAwarenessMessage)
    assert net.get_received(2, "VRUAM") == []                  # 140 m away


def test_vru_tx_power_and_coverage_are_configurable():
    net, vs = build([40, 160], walkers=[Walker(900, 20.0, 0.0)], vru_tx_power_dbm=10.0, vru_awareness_range_m=200.0)
    p2v = of_type(step(net, vs), "P2V")
    assert sorted(l.rx_id for l in p2v) == [1, 2] and all(l.tx_power_dbm == 10.0 for l in p2v)


def test_ground_truth_lists_rsu_and_pedestrian_stations_apart_from_what_vehicles_received():
    net, vs = build([40, 500], rsus=[(3, (0.0, 0.0), 33.0)], walkers=[Walker(900, 20.0, 0.0)])
    f = step(net, vs)
    assert {-3, 900} <= set(f.stations)
    assert f.stations[-3].kind == "RSU" and f.stations[900].kind == "VRU"
    assert all(m.tx_id not in (-3, 900) for m in f.inbox.get(2, ()))   # the far vehicle heard neither
