"""LinkResult records of the channel (REFACTOR_HANDOFF section 5.3)."""

import math
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).parent.parent.parent))

from src.v2x.channel.base import DROP_CHANNEL, DROP_MAC, LINK_I2V, LinkResult
from src.v2x.channel.dsrc_logdistance import DSRCChannel, DSRCConfig
from tests.golden.test_dsrc_channel_golden import _bsm


def _grid(n, spacing=40.0):
    return {i: ((i % 5) * spacing, (i // 5) * spacing) for i in range(n)}


def _run(cfg=None, n=10, frame=7, sim_time=3.5):
    pos = _grid(n)
    msgs = {i: _bsm(i, *p) for i, p in pos.items()}
    ch = DSRCChannel(cfg or DSRCConfig(random_seed=11))
    return ch, ch.broadcast_all(msgs, pos, frame=frame, sim_time=sim_time)


def test_one_record_per_ordered_pair_stamped_with_frame_and_time():
    ch, links = _run(n=10)
    assert len(links) == 10 * 9 and ch.last_links == links
    assert {(l.tx_id, l.rx_id) for l in links} == {(a, b) for a in range(10) for b in range(10) if a != b}
    assert all(l.frame == 7 and l.sim_time == 3.5 and l.msg_type == "BSM" and l.link_type == "V2V" for l in links)


def test_fields_are_internally_consistent():
    cfg = DSRCConfig(random_seed=11)
    _, links = _run(cfg)
    for l in links:
        assert l.rx_power_dbm == pytest.approx(l.tx_power_dbm + 2 * cfg.antenna_gain_dbi - l.path_loss_db - l.shadowing_db)
        assert l.rx_margin_db == pytest.approx(l.rx_power_dbm - cfg.rx_sensitivity_dbm)
        assert l.p_total == pytest.approx(l.p_channel * l.p_mac)
        assert l.p_channel == pytest.approx(1 / (1 + math.exp(-cfg.prr_sigmoid_k * l.rx_margin_db)))
        assert l.delivered == (l.drop_cause is None)
        assert l.drop_cause in (None, DROP_CHANNEL, DROP_MAC)


def test_received_inboxes_are_exactly_the_delivered_links():
    ch, links = _run(n=10)
    delivered = {(l.tx_id, l.rx_id) for l in links if l.delivered}
    inbox = {(tx, rx) for rx in range(10) for tx in ch.get_received(rx)}
    assert inbox == delivered


def test_same_seed_gives_identical_records():
    assert _run()[1] == _run()[1]
    assert _run(DSRCConfig(random_seed=12))[1] != _run()[1]


def test_legacy_stats_equal_the_v2v_link_counts():
    ch, links = _run(n=10)
    assert ch.stats["total_broadcasts"] == len(links) == ch.stats_by_link_type["V2V"]["total"]
    assert ch.stats["total_deliveries"] == sum(l.delivered for l in links)
    assert ch.stats["total_drops"] == sum(not l.delivered for l in links)


# ── property tests: a parameter change changes the logged numbers as the model says ──────────

def test_tx_power_shifts_rx_power_by_exactly_the_same_db_and_nothing_else():
    _, base = _run(DSRCConfig(random_seed=11, tx_power_dbm=23.0))
    _, loud = _run(DSRCConfig(random_seed=11, tx_power_dbm=33.0))
    for a, b in zip(base, loud):
        assert b.rx_power_dbm - a.rx_power_dbm == pytest.approx(10.0)
        assert b.path_loss_db == a.path_loss_db and b.shadowing_db == a.shadowing_db
        assert b.p_channel >= a.p_channel


def test_path_loss_exponent_changes_path_loss_only_by_the_log_term():
    _, base = _run(DSRCConfig(random_seed=11, path_loss_exponent_los=2.0, path_loss_exponent_nlos=2.0))
    _, steep = _run(DSRCConfig(random_seed=11, path_loss_exponent_los=3.0, path_loss_exponent_nlos=3.0))
    for a, b in zip(base, steep):
        assert b.path_loss_db - a.path_loss_db == pytest.approx(10.0 * math.log10(a.distance_m / 10.0))


def test_shadowing_sample_scales_with_sigma():
    """Same seed, same underlying normal draws: doubling sigma doubles the logged sample."""
    _, one = _run(DSRCConfig(random_seed=11, shadowing_std_los_db=4.0, shadowing_std_nlos_db=4.0))
    _, two = _run(DSRCConfig(random_seed=11, shadowing_std_los_db=8.0, shadowing_std_nlos_db=8.0))
    for a, b in zip(one, two):
        assert b.shadowing_db == pytest.approx(2.0 * a.shadowing_db)


def test_sensitivity_moves_the_margin_one_to_one():
    _, base = _run(DSRCConfig(random_seed=11, rx_sensitivity_dbm=-85.0))
    _, worse = _run(DSRCConfig(random_seed=11, rx_sensitivity_dbm=-70.0))
    for a, b in zip(base, worse):
        assert a.rx_margin_db - b.rx_margin_db == pytest.approx(15.0)


# ── single-link behaviour ─────────────────────────────────────────────────────────────────────

def test_co_located_stations_are_delivered_without_a_random_draw():
    ch = DSRCChannel(DSRCConfig(random_seed=1))
    state = ch._rng.getstate()
    link = ch.evaluate_link(1, 2, 0.2, False, 5)
    assert link.delivered and link.p_total == 1.0 and link.drop_cause is None
    assert ch._rng.getstate() == state


def test_losses_are_attributed_to_the_channel_when_the_radio_is_the_cause():
    ch = DSRCChannel(DSRCConfig(random_seed=3))
    causes = [ch.evaluate_link(1, 2, 900.0, True, 1).drop_cause for _ in range(300)]
    assert set(causes) <= {None, DROP_CHANNEL} and causes.count(DROP_CHANNEL) > 250


def test_losses_are_attributed_to_the_mac_when_contention_is_the_cause():
    ch = DSRCChannel(DSRCConfig(random_seed=3))
    links = [ch.evaluate_link(1, 2, 20.0, False, 400) for _ in range(600)]   # p_channel ~ 1, p_mac = 0.35
    drops = [l.drop_cause for l in links if not l.delivered]
    assert drops and drops.count(DROP_MAC) > 0.95 * len(drops)
    assert 0.25 < sum(l.delivered for l in links) / len(links) < 0.45


def test_per_station_tx_power_and_link_type_are_recorded():
    ch = DSRCChannel(DSRCConfig(random_seed=3))
    a = ch.evaluate_link(-1, 5, 100.0, False, 1, msg_type="SPAT", link_type=LINK_I2V, tx_power_dbm=33.0)
    assert (a.tx_power_dbm, a.msg_type, a.link_type) == (33.0, "SPAT", "I2V")
    assert a.rx_power_dbm == pytest.approx(33.0 + 4.0 - a.path_loss_db - a.shadowing_db)
    assert ch.stats_by_link_type["I2V"]["total"] == 1
    assert ch.stats["total_broadcasts"] == 0   # legacy stats cover V2V only


def test_link_records_are_immutable():
    link = _run(n=3)[1][0]
    assert isinstance(link, LinkResult)
    with pytest.raises(Exception):
        link.delivered = not link.delivered
