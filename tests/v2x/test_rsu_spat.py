"""RSU SPAT timing: known timing is exact, unknown timing is None (S17)."""

import sys
import types
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).parent.parent.parent))

import src.v2x.rsu as rsu_mod
from src.v2x.rsu import RoadSideUnit, RSUConfig


class _State:
    """Stand-in for carla.TrafficLightState members."""
    Red, Yellow, Green, Off, Unknown = "Red", "Yellow", "Green", "Off", "Unknown"


@pytest.fixture(autouse=True)
def fake_carla(monkeypatch):
    mod = types.ModuleType("carla")
    mod.TrafficLightState = _State
    monkeypatch.setitem(sys.modules, "carla", mod)
    # the RSU maps CARLA states to SAE strings through a lazily built, cached table
    monkeypatch.setattr(rsu_mod, "_CARLA_STATE_MAP", {
        _State.Red: "stop-And-Remain", _State.Yellow: "protectedClearance",
        _State.Green: "protectedMovementAllowed"}, raising=True)
    monkeypatch.setattr(rsu_mod, "_build_carla_state_map", lambda: rsu_mod._CARLA_STATE_MAP)


class _TL:
    def __init__(self, state, elapsed=2.0, green=10.0, yellow=3.0, red=8.0, fail=False):
        self._s, self._e, self._g, self._y, self._r, self._fail = state, elapsed, green, yellow, red, fail

    def get_state(self):
        return self._s

    def get_elapsed_time(self):
        if self._fail:
            raise RuntimeError("rpc failed")
        return self._e

    def get_green_time(self): return self._g
    def get_yellow_time(self): return self._y
    def get_red_time(self): return self._r


def _rsu(*tls):
    cfg = RSUConfig(intersection_id=1, position=(0.0, 0.0, 0.0))
    return RoadSideUnit(cfg, traffic_lights=list(tls))


@pytest.mark.parametrize("state,expected_remaining", [
    (_State.Green, 10.0 - 2.0), (_State.Yellow, 3.0 - 2.0), (_State.Red, 8.0 - 2.0)])
def test_known_timing_is_exact_with_no_made_up_window(state, expected_remaining):
    spat = _rsu(_TL(state)).get_spat(timestamp=100.0)
    st = spat.states[0]
    assert st.likely_time_s == pytest.approx(100.0 + expected_remaining)
    assert st.min_end_time_s == st.max_end_time_s == st.likely_time_s   # was +-1 s before S17


def test_failed_query_gives_none_not_five_seconds():
    st = _rsu(_TL(_State.Green, fail=True)).get_spat(timestamp=100.0).states[0]
    assert st.likely_time_s is None and st.min_end_time_s is None and st.max_end_time_s is None


@pytest.mark.parametrize("state", [_State.Off, _State.Unknown])
def test_state_without_a_duration_gives_none(state):
    st = _rsu(_TL(state)).get_spat(timestamp=50.0).states[0]
    assert st.likely_time_s is None


def test_remaining_time_never_negative_when_phase_overran():
    st = _rsu(_TL(_State.Green, elapsed=99.0)).get_spat(timestamp=10.0).states[0]
    assert st.likely_time_s == pytest.approx(10.0)
