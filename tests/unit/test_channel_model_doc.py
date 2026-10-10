"""docs/science/model.md must match what the channel code computes (S1 acceptance test).

The tables in the document are parsed and recomputed from the code: P_rx within 0.01 dB,
E[p_channel] within 0.001, p_csma within 0.0001. If someone changes the channel parameters or
code without updating the document (or the other way round) this fails.
"""

import math
import re
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).parent.parent.parent))

from src.v2x.channel.dsrc_logdistance import DSRCChannel, DSRCConfig

DOC = (Path(__file__).parent.parent.parent / "docs" / "science" / "model.md").read_text()


def _rows(begin, end):
    block = DOC.split(begin)[1].split(end)[0]
    return [[c.strip() for c in line.strip("|").split("|")]
            for line in block.splitlines() if re.match(r"\|\s*\d", line)]


def _mean_prx(cfg, d, nlos):
    quiet = DSRCConfig(**{**cfg.__dict__, "shadowing_std_los_db": 0.0, "shadowing_std_nlos_db": 0.0})
    return DSRCChannel(quiet)._received_power_dbm(d, nlos)


def _expected_p_channel(cfg, d, nlos):
    sigma = cfg.shadowing_std_nlos_db if nlos else cfg.shadowing_std_los_db
    margin = _mean_prx(cfg, d, nlos) - cfg.rx_sensitivity_dbm
    x = np.linspace(-12 * sigma, 12 * sigma, 400_001)
    pdf = np.exp(-x**2 / (2 * sigma**2)) / (sigma * math.sqrt(2 * math.pi))
    return float(np.trapezoid(pdf / (1 + np.exp(-cfg.prr_sigmoid_k * (margin - x))), x))


CHANNEL_ROWS = _rows("<!-- BEGIN CHANNEL TABLE -->", "<!-- END CHANNEL TABLE -->")
CSMA_ROWS = _rows("<!-- BEGIN CSMA TABLE -->", "<!-- END CSMA TABLE -->")


def test_tables_were_found():
    assert len(CHANNEL_ROWS) >= 8 and len(CSMA_ROWS) >= 6


@pytest.mark.parametrize("row", CHANNEL_ROWS, ids=lambda r: f"{r[0]}m-{r[1]}")
def test_channel_table_matches_code(row):
    d, link, prx, margin, e = int(row[0]), row[1], float(row[2]), float(row[3]), float(row[4])
    cfg, nlos = DSRCConfig(), link == "NLOS"
    assert _mean_prx(cfg, d, nlos) == pytest.approx(prx, abs=0.01)
    assert _mean_prx(cfg, d, nlos) - cfg.rx_sensitivity_dbm == pytest.approx(margin, abs=0.01)
    assert _expected_p_channel(cfg, d, nlos) == pytest.approx(e, abs=0.001)


@pytest.mark.parametrize("row", CSMA_ROWS, ids=lambda r: f"N={r[0]}")
def test_csma_table_matches_code(row):
    assert DSRCChannel()._csma_success_prob(int(row[0])) == pytest.approx(float(row[1]), abs=1e-4)


def test_expected_p_channel_agrees_with_the_real_code_path():
    """Independent check of the integral: Monte Carlo through DSRCChannel._compute_prr."""
    ch = DSRCChannel(DSRCConfig(random_seed=3))
    n = 60_000
    mc = sum(ch._compute_prr(150.0, True, 1) for _ in range(n)) / n
    assert mc == pytest.approx(_expected_p_channel(DSRCConfig(), 150.0, True), abs=0.01)


def test_changing_a_parameter_changes_the_numbers():
    """The table is not a constant: a 3 dB lower TX power must move it."""
    base = _expected_p_channel(DSRCConfig(), 150.0, True)
    weaker = _expected_p_channel(DSRCConfig(tx_power_dbm=20.0), 150.0, True)
    assert weaker < base - 0.05
