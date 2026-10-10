# V2X radio model (DSRC / IEEE 802.11p), as implemented

Status: describes the code in `src/v2x/channel/dsrc_logdistance.py` at the commit that added this file.
Everything in the tables below is recomputed from the code by `tests/unit/test_channel_model_doc.py`; if the
code changes, that test fails until this file is updated.

**Source status.** The authors of this repository could not read any of the papers or standards cited in the
code. Each source below is marked *unverified*: the claim "parameter X comes from source Y" has not been
checked. Treat every parameter as a **modelling assumption of this project** until a reference is verified
against the primary source (page/table) and this file is updated.

## 1. What is computed for one transmitter -> receiver pair

Per V2X update, for every ordered pair (tx, rx) of equipped vehicles:

1. Distance `d` (m) between the two BSM positions (centre to centre, 2-D, CARLA world x/y).
2. If `d < 0.5 m`: delivered (no draw).
3. LOS or NLOS (section 3).
4. Received power, one **shadowing sample per packet** (section 2):
   `P_rx = P_tx + 2*G - [ L0 + 10*n*log10(d / d0) + X ]`, with `X ~ N(0, sigma^2)`.
5. `margin = P_rx - S` (dB above the receiver sensitivity `S`). *This is not an SNR*: there is no noise floor
   and no interference term. (Called `snr_margin` in older code and docs.)
6. `p_channel = 1 / (1 + exp(-k * margin))` (logistic function applied to the *sampled* margin).
7. `p_csma = max(0.35, exp(-0.005 * (N - 1)))`, `N` = vehicles within 300 m of the transmitter incl. itself.
8. The packet is delivered with probability `p_channel * p_csma` (one Bernoulli draw).
9. Afterwards the network applies an application-layer **awareness range** filter (default 150 m): neighbours
   beyond it are dropped regardless of the radio result.

All draws use one seeded `random.Random(seed)` per channel; results are reproducible for a fixed order of calls.

## 2. Parameters (defaults of `DSRCConfig`)

| Symbol | Default | Meaning | Source status |
|---|---|---|---|
| `P_tx` | 23 dBm | transmit power | "ETSI EN 302 663 Class A, 23 dBm ERP" - *unverified* |
| `G` | 2 dBi | antenna gain, applied at TX **and** RX (`2*G` total) | project assumption |
| `S` | -85 dBm | receiver sensitivity (50 % point of the logistic) | "typical 802.11p" - *unverified* |
| `f` | 5.9 GHz | carrier | ETSI/DSRC band - *unverified* |
| `d0` | 10 m | reference distance | project choice |
| `L0` | 67.859 dB | free-space (Friis) loss at `d0` | computed: `20*log10(4*pi*d0*f/c)` |
| `n_LOS` / `n_NLOS` | 2.3 / 3.2 | path-loss exponents | "Paier et al. 2008" - *unverified* |
| `sigma_LOS` / `sigma_NLOS` | 5.5 / 8.0 dB | log-normal shadowing std | "Paier 2008, Sommer 2011" - *unverified* |
| `k` | 2.0 | logistic steepness (1/dB) | "Sommer et al. 2011" - *unverified* |
| carrier-sense range | 300 m | contender counting radius | hard-coded, uncited |
| NLOS blocker radius | 2.5 m | another vehicle within this distance of the tx-rx segment makes NLOS | project assumption |
| `channel_busy_ratio` | 0.15 | **not used by any computation** (kept for compatibility) | - |

## 3. LOS / NLOS

A link is NLOS if any *other registered vehicle* lies within 2.5 m of the straight tx-rx segment. Buildings
and other obstacles are **not** considered, and vehicles that are not V2X stations are not blockers.

## 4. Numbers (mean P_rx and E[p_channel]) - machine-checked

`P_rx` is the mean (X = 0). `E[p_channel]` is the logistic function averaged over the shadowing distribution
(`N` = 1 contender, so `p_csma` = 1). Computed by numerical integration; cross-checked by Monte Carlo through
`DSRCChannel._compute_prr` (150 m NLOS: integral 0.7905, 5 x 200 000 draws 0.7904).

<!-- BEGIN CHANNEL TABLE -->
| d (m) | link | P_rx (dBm) | margin (dB) | E[p_channel] |
|---|---|---|---|---|
| 10 | LOS | -40.859 | 44.141 | 1.0000 |
| 50 | LOS | -56.935 | 28.065 | 1.0000 |
| 100 | LOS | -63.859 | 21.141 | 0.9999 |
| 150 | LOS | -67.909 | 17.091 | 0.9989 |
| 150 | NLOS | -78.494 | 6.506 | 0.7905 |
| 300 | LOS | -74.833 | 10.167 | 0.9659 |
| 500 | LOS | -79.935 | 5.065 | 0.8182 |
| 500 | NLOS | -95.226 | -10.226 | 0.1020 |
| 1000 | LOS | -86.859 | -1.859 | 0.3694 |
<!-- END CHANNEL TABLE -->

`p_csma(N)` (section 1, step 7):

<!-- BEGIN CSMA TABLE -->
| N | p_csma |
|---|---|
| 1 | 1.0000 |
| 10 | 0.9560 |
| 20 | 0.9094 |
| 50 | 0.7827 |
| 100 | 0.6096 |
| 200 | 0.3697 |
| 500 | 0.3500 |
<!-- END CSMA TABLE -->

## 5. What this means (and what earlier documents got wrong)

* In line of sight the radio is **effectively lossless out to about 150 m** (E[p_channel] >= 0.998) and still
  0.82 at 500 m. With the default 150 m awareness range (75 m on the server), the radio model rarely decides
  anything in line of sight; the awareness-range cutoff does. Only NLOS links (0.79 at 150 m) and high
  contender counts make the radio matter inside the awareness range.
* The calibration printed in earlier code comments and in the removed `docs/agents/audit.md` (in git history) ("P_rx ~ -71 dBm at 50 m LOS,
  -77 dBm at 100 m LOS, -87 dBm at 150 m NLOS; meaningful PRR variation within 10-150 m") is **numerically
  false**. The code gives -56.9, -63.9 and -78.5 dBm (table above). Those statements were removed.

## 6. Known weaknesses (limitations, not bugs fixed here)

* **S5 double randomisation.** Shadowing is drawn independently for every packet and tick (no spatial or
  temporal correlation), and that *random* margin then goes through a logistic probability, followed by a
  second Bernoulli draw. Link behaviour is therefore more volatile than a correlated-shadowing model and the
  logistic is applied to a sample, not to the mean. Whether this matches any measurement is *unverified*.
* **S6 CSMA term is EXPERIMENTAL and uncalibrated.** `lambda = 0.005` and the 0.35 floor are not derived from
  data or from an analytical model; the older docstring table (0.95 / 0.85 / 0.70 / 0.50) contradicted the
  code (0.91 / 0.78 / 0.61 / 0.37 for N = 20 / 50 / 100 / 200 - the table above is what the code does). The
  cited "Campolo et al. (2011)", "ETSI TR 102 863" and "Bianchi (2000)" are *unverified*; the model is **not**
  Bianchi's analytical model. Do not report results that depend on `p_csma` as a validated MAC model.
* **No interference, no noise floor, no latency.** All messages that are delivered arrive in the same tick.
* **Transmit side is uniform.** RSUs and pedestrians do not use this channel at all yet (their reception is a
  distance test); `RSUConfig.tx_power_dbm` (33 dBm) is not used. See `docs/science/limitations.md`.
