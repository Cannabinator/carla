"""S7: one RSU per intersection (CARLA light group), deterministic, signal groups per light."""

import random
import sys
from pathlib import Path


sys.path.insert(0, str(Path(__file__).parent.parent.parent))

from src.v2x.rsu import RoadSideUnit, group_traffic_lights
from tests.unit.carla_stub import TrafficLight


def _junction(base_id, x, y, n=4, symmetric=True):
    lights = [TrafficLight(base_id + i, x + i, y) for i in range(n)]
    for tl in lights:
        tl.group = list(lights) if symmetric else [l for l in lights if l.id >= tl.id]
    return lights


def _ids(groups):
    return [[tl.id for tl in g] for g in groups]


def test_lights_of_one_junction_form_one_group_and_singletons_stay_alone():
    a, b = _junction(10, 0, 0, 4), _junction(50, 100, 0, 2)
    lone = TrafficLight(90, 300, 0, group=[])
    assert _ids(group_traffic_lights(a + b + [lone])) == [[10, 11, 12, 13], [50, 51], [90]]


def test_result_is_independent_of_input_order():
    lights = _junction(10, 0, 0, 4) + _junction(50, 100, 0, 3) + [TrafficLight(7, 9, 9, group=[])]
    want = _ids(group_traffic_lights(lights))
    rng = random.Random(0)
    for _ in range(20):
        shuffled = lights[:]
        rng.shuffle(shuffled)
        assert _ids(group_traffic_lights(shuffled)) == want
    assert want[0] == [7]  # groups are ordered by their smallest actor id


def test_asymmetric_group_answers_do_not_split_an_intersection():
    assert _ids(group_traffic_lights(_junction(10, 0, 0, 4, symmetric=False))) == [[10, 11, 12, 13]]


def test_a_failed_group_query_makes_that_light_its_own_group():
    broken = TrafficLight(1, 0, 0, group=None)
    ok = _junction(20, 50, 0, 2)
    assert _ids(group_traffic_lights([broken] + ok)) == [[1], [20, 21]]


def test_rsu_for_an_intersection_sits_at_the_mean_position_and_has_one_signal_group_per_light(monkeypatch):
    lights = _junction(10, 0, 0, 4)  # x = 0,1,2,3 -> mean 1.5
    rsu = RoadSideUnit.from_traffic_lights(list(reversed(lights)), intersection_id=7)
    assert rsu.config.intersection_id == 7 and rsu.config.position == (1.5, 0.0, 0.0)
    assert [tl.id for tl in rsu._traffic_lights] == [10, 11, 12, 13]  # id order whatever the input order


def test_spat_has_one_movement_state_per_light_with_stable_signal_group_ids():
    import types

    import src.v2x.rsu as rsu_mod
    lights = _junction(10, 0, 0, 3)
    rsu = RoadSideUnit.from_traffic_lights(lights, intersection_id=1)
    mod = types.ModuleType("carla")
    mod.TrafficLightState = types.SimpleNamespace(Red="Red", Yellow="Y", Green="G", Off="O", Unknown="U")
    sys.modules["carla"] = mod
    try:
        rsu_mod._CARLA_STATE_MAP.clear()
        spat = rsu.get_spat(timestamp=5.0)
    finally:
        del sys.modules["carla"]
        rsu_mod._CARLA_STATE_MAP.clear()
    assert [st.signal_group_id for st in spat.states] == [0, 1, 2]
