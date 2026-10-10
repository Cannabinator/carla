"""Pre-binned channel statistics over a sliding window of V2X updates (REFACTOR_HANDOFF 5.7.2, topic v2x.stats).

The browser gets bins, never raw links: with 100 equipped vehicles one update has ~10,000 links. Everything here
is computed from the same ``LinkResult`` rows the recorder writes, so a plot can be reproduced from a run.

Definitions (also shown in the UI):
  * a link is *counted* in a PRR bin unless the application-layer awareness filter dropped it
    (``drop_cause == "out_of_awareness_range"``): that decision is not the radio's;
  * PRR of a bin = delivered / counted (None when nothing was counted);
  * distance bins are ``bin_m`` wide, the last one collects everything beyond ``max_distance_m``.
"""

from __future__ import annotations

from collections import deque
from typing import Any, Deque, Dict, List, Optional, Tuple

import numpy as np

from .channel.base import DROP_AWARENESS, DROP_CHANNEL, DROP_MAC
from .frame import V2XFrame

MARGIN_EDGES_DB = list(range(-30, 65, 5))      # rx margin above sensitivity (not an SNR)


class V2XStats:
    def __init__(self, window_updates: int = 20, bin_m: float = 20.0, max_distance_m: float = 400.0):
        self.window = window_updates
        self.bin_m = float(bin_m)
        self.n_bins = int(round(max_distance_m / bin_m)) + 1       # +1: overflow bin
        self._updates: Deque[Dict[str, Any]] = deque(maxlen=window_updates)

    def add(self, frame: V2XFrame) -> None:
        links = frame.links
        n = len(links)
        dist = np.fromiter((l.distance_m for l in links), dtype=np.float64, count=n)
        deliv = np.fromiter((l.delivered for l in links), dtype=bool, count=n)
        los = np.fromiter((l.los for l in links), dtype=bool, count=n)
        margin = np.fromiter((l.rx_margin_db for l in links), dtype=np.float64, count=n)
        cause = np.array([l.drop_cause or "" for l in links], dtype="<U32")
        ltype = np.array([l.link_type for l in links]) if n else np.array([], dtype="<U3")
        counted = cause != DROP_AWARENESS
        bins = np.minimum((dist // self.bin_m).astype(np.int64), self.n_bins - 1)

        series: Dict[str, Tuple[np.ndarray, np.ndarray]] = {}
        drops: Dict[str, Dict[str, int]] = {}
        for lt in sorted(set(ltype.tolist())):
            m_lt = ltype == lt
            for los_flag, name in ((True, "LOS"), (False, "NLOS")):
                m = m_lt & counted & (los == los_flag)
                if not m.any():
                    continue
                series[f"{lt}/{name}"] = (np.bincount(bins[m], minlength=self.n_bins),
                                          np.bincount(bins[m & deliv], minlength=self.n_bins))
            drops[lt] = {
                "delivered": int((m_lt & deliv).sum()),
                DROP_CHANNEL: int((m_lt & (cause == DROP_CHANNEL)).sum()),
                DROP_MAC: int((m_lt & (cause == DROP_MAC)).sum()),
                DROP_AWARENESS: int((m_lt & (cause == DROP_AWARENESS)).sum()),
            }
        lo, hi = MARGIN_EDGES_DB[0], MARGIN_EDGES_DB[-1]
        hist_deliv = np.histogram(np.clip(margin[counted & deliv], lo, hi), bins=MARGIN_EDGES_DB)[0]
        hist_drop = np.histogram(np.clip(margin[counted & ~deliv], lo, hi), bins=MARGIN_EDGES_DB)[0]
        self._updates.append({"frame": frame.frame, "sim_time": frame.sim_time, "series": series, "drops": drops,
                              "hist_deliv": hist_deliv, "hist_drop": hist_drop})

    def snapshot(self) -> Dict[str, Any]:
        ups = list(self._updates)
        edges = [i * self.bin_m for i in range(self.n_bins)]
        out: Dict[str, Any] = {
            "window_updates": len(ups), "window_max": self.window, "bin_m": self.bin_m,
            "distance_bin_starts_m": edges, "last_bin_is_overflow": True,
            "first_frame": ups[0]["frame"] if ups else None, "last_frame": ups[-1]["frame"] if ups else None,
            "sim_time_span": [ups[0]["sim_time"], ups[-1]["sim_time"]] if ups else None,
            "definition": "PRR = delivered / counted links; links dropped by the awareness-range filter are not counted",
            "prr_by_distance": {}, "drops": {}, "rx_margin_hist": None,
        }
        if not ups:
            return out
        keys = sorted({k for u in ups for k in u["series"]})
        for k in keys:
            n = sum((u["series"][k][0] for u in ups if k in u["series"]), np.zeros(self.n_bins, dtype=np.int64))
            d = sum((u["series"][k][1] for u in ups if k in u["series"]), np.zeros(self.n_bins, dtype=np.int64))
            with np.errstate(invalid="ignore", divide="ignore"):
                prr: List[Optional[float]] = [None if c == 0 else float(x) / float(c) for x, c in zip(d, n)]
            out["prr_by_distance"][k] = {"counted": n.tolist(), "delivered": d.tolist(), "prr": prr}
        lts = sorted({lt for u in ups for lt in u["drops"]})
        for lt in lts:
            acc = {"delivered": 0, DROP_CHANNEL: 0, DROP_MAC: 0, DROP_AWARENESS: 0}
            for u in ups:
                for kk, vv in u["drops"].get(lt, {}).items():
                    acc[kk] += vv
            out["drops"][lt] = acc
        out["rx_margin_hist"] = {
            "edges_db": MARGIN_EDGES_DB,
            "note": "values are clipped into the first/last bin",
            "delivered": sum((u["hist_deliv"] for u in ups), np.zeros(len(MARGIN_EDGES_DB) - 1, dtype=np.int64)).tolist(),
            "dropped": sum((u["hist_drop"] for u in ups), np.zeros(len(MARGIN_EDGES_DB) - 1, dtype=np.int64)).tolist(),
        }
        return out
