"""V2XStats bins hand-made links; expected numbers are worked out by hand."""

import pytest

from src.v2x.channel.base import DROP_AWARENESS, DROP_CHANNEL, DROP_MAC, LinkResult
from src.v2x.frame import ChannelStats, V2XFrame, frozen_map
from src.v2x.stats import MARGIN_EDGES_DB, V2XStats, wilson_interval


def link(dist, delivered, los=True, link_type="V2V", cause=None, margin=10.0, rx=2):
    if cause is None and not delivered:
        cause = DROP_CHANNEL
    return LinkResult(frame=1, sim_time=0.05, tx_id=1, rx_id=rx, msg_type="BSM", link_type=link_type,
                      distance_m=dist, los=los, tx_power_dbm=20.0, path_loss_db=80.0, shadowing_db=0.0,
                      rx_power_dbm=-70.0, rx_margin_db=margin, p_channel=1.0, p_mac=1.0, p_total=1.0,
                      delivered=delivered, drop_cause=cause)


def frame(links, n=1):
    return V2XFrame(frame=n, sim_time=n * 0.5, stations=frozen_map({}), sent=frozen_map({}), inbox=frozen_map({}),
                    links=tuple(links), threats=frozen_map({}), stats=ChannelStats(frozen_map({})))


def test_empty_window():
    s = V2XStats()
    snap = s.snapshot()
    for part in (snap["window"], snap["run"]):
        assert part["updates"] == 0 and part["prr_by_distance"] == {} and part["sim_time_span"] is None


def test_prr_by_distance_bin_los_nlos_and_link_type():
    s = V2XStats(bin_m=20.0, max_distance_m=100.0)       # bins 0,20,40,60,80 and overflow (>=100)
    s.add(frame([
        link(5, True), link(10, True), link(15, False),                 # bin 0 LOS: 2 of 3
        link(25, True), link(39.9, False),                              # bin 1 LOS: 1 of 2
        link(25, False, los=False),                                     # bin 1 NLOS: 0 of 1
        link(500, True),                                                # overflow bin
        link(10, True, link_type="I2V"),
    ]))
    p = s.snapshot()["window"]["prr_by_distance"]
    assert set(p) == {"V2V/LOS", "V2V/NLOS", "I2V/LOS"}
    assert p["V2V/LOS"]["counted"][:2] == [3, 2] and p["V2V/LOS"]["delivered"][:2] == [2, 1]
    assert p["V2V/LOS"]["prr"][0] == pytest.approx(2 / 3) and p["V2V/LOS"]["prr"][1] == 0.5
    assert p["V2V/LOS"]["prr"][2] is None                                  # nothing counted: no number
    assert p["V2V/LOS"]["counted"][-1] == 1 and p["V2V/LOS"]["delivered"][-1] == 1      # overflow
    assert p["V2V/NLOS"]["counted"][1] == 1 and p["V2V/NLOS"]["prr"][1] == 0.0
    assert p["I2V/LOS"]["counted"][0] == 1
    assert len(s.snapshot()["distance_bin_starts_m"]) == 6


def test_awareness_drops_are_not_counted_in_prr_but_are_in_drops():
    s = V2XStats()
    s.add(frame([link(5, True), link(5, False, cause=DROP_AWARENESS), link(5, False, cause=DROP_MAC),
                 link(5, False, cause=DROP_CHANNEL)]))
    snap = s.snapshot()
    assert snap["window"]["prr_by_distance"]["V2V/LOS"]["counted"][0] == 3             # delivered, mac, channel
    assert snap["window"]["prr_by_distance"]["V2V/LOS"]["prr"][0] == pytest.approx(1 / 3)
    assert snap["window"]["drops"]["V2V"] == {"delivered": 1, DROP_CHANNEL: 1, DROP_MAC: 1, DROP_AWARENESS: 1}


def test_window_slides_in_simulated_seconds_while_the_run_keeps_everything():
    s = V2XStats(window_s=1.0)
    s.add(frame([link(5, True)], n=1))          # sim time 0.5 (frame(n) uses n * 0.5)
    s.add(frame([link(5, False)], n=2))         # 1.0
    s.add(frame([link(5, False)], n=3))         # 1.5
    snap = s.snapshot()
    w, r = snap["window"], snap["run"]
    assert w["updates"] == 3 and w["first_frame"] == 1                 # 1.5 - 0.5 = 1.0 is still inside the window
    s.add(frame([link(5, False)], n=4))         # 2.0: the first update (0.5) is now older than 1.0 s
    w, r = s.snapshot()["window"], s.snapshot()["run"]
    assert w["updates"] == 3 and w["first_frame"] == 2 and w["last_frame"] == 4
    assert w["prr_by_distance"]["V2V/LOS"]["prr"][0] == 0.0            # the delivered link left the window
    assert r["updates"] == 4 and r["first_frame"] == 1
    assert r["prr_by_distance"]["V2V/LOS"]["prr"][0] == pytest.approx(0.25)
    assert w["sim_time_span"] == [1.0, 2.0] and r["sim_time_span"] == [0.5, 2.0]


def test_a_restarted_simulation_resets_window_and_run():
    s = V2XStats(window_s=60.0)
    s.add(frame([link(5, True)], n=10))
    s.add(frame([link(5, False)], n=2))         # sim time went backwards
    snap = s.snapshot()
    assert snap["window"]["updates"] == 1 and snap["run"]["updates"] == 1
    assert snap["run"]["prr_by_distance"]["V2V/LOS"]["prr"][0] == 0.0


def test_margin_histogram_clips_and_separates_delivered():
    s = V2XStats()
    s.add(frame([link(10, True, margin=12.0), link(10, True, margin=100.0), link(10, False, margin=-80.0),
                 link(10, False, margin=-3.0, cause=DROP_AWARENESS)]))
    h = s.snapshot()["window"]["rx_margin_hist"]
    assert h["edges_db"] == MARGIN_EDGES_DB and sum(h["delivered"]) == 2 and sum(h["dropped"]) == 1
    assert h["delivered"][-1] == 1                                           # 100 dB clipped into the last bin
    assert h["delivered"][MARGIN_EDGES_DB.index(10)] == 1                    # 12 dB in [10, 15)
    assert h["dropped"][0] == 1                                              # -80 dB clipped into the first bin


def test_scales_to_a_real_update_size():
    import time
    links = [link(d % 300, d % 3 != 0, los=d % 5 != 0, link_type=("V2V", "I2V", "P2V")[d % 3]) for d in range(12000)]
    f = frame(links)
    s = V2XStats()
    t0 = time.perf_counter()
    s.add(f)
    s.snapshot()
    assert time.perf_counter() - t0 < 0.5


@pytest.mark.parametrize("k,n,lo,hi", [(0, 10, 0.0, 0.2775), (5, 10, 0.2366, 0.7634), (10, 10, 0.7225, 1.0),
                                       (1, 1, 0.2065, 1.0), (50, 100, 0.4038, 0.5962)])
def test_wilson_interval_matches_published_values(k, n, lo, hi):
    """Reference values of the Wilson score interval at 95 % (Wilson 1927; tabulated e.g. in Brown, Cai & DasGupta
    2001, and reproducible with statsmodels.proportion_confint(method='wilson'))."""
    a, b = wilson_interval(k, n)
    assert a == pytest.approx(lo, abs=1e-3) and b == pytest.approx(hi, abs=1e-3)


def test_wilson_interval_empty_and_bounds():
    assert wilson_interval(0, 0) == (None, None)
    for n in (1, 2, 7, 100, 10_000):
        for k in range(0, min(n, 20) + 1):
            lo, hi = wilson_interval(k, n)
            assert 0.0 <= lo <= k / n <= hi <= 1.0


def test_stats_carry_the_interval_per_bin():
    s = V2XStats(bin_m=10.0, max_distance_m=50.0)
    s.add(frame([link(5, True)] * 8 + [link(5, False)] * 2 + [link(25, True)]))
    p = s.snapshot()["window"]["prr_by_distance"]["V2V/LOS"]
    assert p["prr"][0] == pytest.approx(0.8)
    assert p["ci95_low"][0] == pytest.approx(0.4902, abs=1e-3) and p["ci95_high"][0] == pytest.approx(0.9433, abs=1e-3)
    assert p["ci95_low"][2] == pytest.approx(0.2065, abs=1e-3)                       # 1 of 1, in the 20-30 m bin
    assert p["ci95_low"][4] is None and p["ci95_high"][4] is None                     # empty bin: no number
