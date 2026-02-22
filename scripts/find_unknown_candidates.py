#!/usr/bin/env python3
"""
Find and rank unknown CAN IDs/bytes, and optionally plot them.

Examples:
  python scripts/find_unknown_candidates.py canlog.csv
  python scripts/find_unknown_candidates.py canlog.csv --session 2 --top-ids 6
  python scripts/find_unknown_candidates.py canlog.csv --plot-top-bytes 6 --output unknown.png
  python scripts/find_unknown_candidates.py canlog.csv --plot 0x176.B0,0x177.B2 --subplots
"""

from __future__ import annotations

import argparse
import csv
import importlib.util
import math
import pathlib
import sys
from collections import Counter, defaultdict
from dataclasses import dataclass, field
from typing import Dict, Iterable, List, Optional, Set, Tuple

try:
    import matplotlib.pyplot as plt

    HAS_MATPLOTLIB = True
except ImportError:
    HAS_MATPLOTLIB = False


@dataclass
class CorrAgg:
    n: int = 0
    sx: float = 0.0
    sy: float = 0.0
    sxx: float = 0.0
    syy: float = 0.0
    sxy: float = 0.0

    def add(self, x: float, y: float) -> None:
        self.n += 1
        self.sx += x
        self.sy += y
        self.sxx += x * x
        self.syy += y * y
        self.sxy += x * y

    def corr(self) -> Optional[float]:
        if self.n < 50:
            return None
        num = self.n * self.sxy - self.sx * self.sy
        den_x = self.n * self.sxx - self.sx * self.sx
        den_y = self.n * self.syy - self.sy * self.sy
        den = den_x * den_y
        if den <= 0:
            return None
        return num / math.sqrt(den)


@dataclass
class ByteStats:
    count: int = 0
    min_v: int = 255
    max_v: int = 0
    last_v: Optional[int] = None
    changes: int = 0
    uniq: Set[int] = field(default_factory=set)
    delta_hist: Counter = field(default_factory=Counter)
    corr_speed: CorrAgg = field(default_factory=CorrAgg)
    corr_rpm: CorrAgg = field(default_factory=CorrAgg)
    corr_brake: CorrAgg = field(default_factory=CorrAgg)
    corr_gear: CorrAgg = field(default_factory=CorrAgg)

    def add(self, v: int, refs: Dict[str, Optional[float]]) -> None:
        self.count += 1
        self.min_v = min(self.min_v, v)
        self.max_v = max(self.max_v, v)
        self.uniq.add(v)

        if self.last_v is not None:
            if v != self.last_v:
                self.changes += 1
                self.delta_hist[(v - self.last_v) % 256] += 1
            else:
                self.delta_hist[0] += 1
        self.last_v = v

        if refs["speed"] is not None:
            self.corr_speed.add(float(v), float(refs["speed"]))
        if refs["rpm"] is not None:
            self.corr_rpm.add(float(v), float(refs["rpm"]))
        if refs["brake"] is not None:
            self.corr_brake.add(float(v), float(refs["brake"]))
        if refs["gear"] is not None:
            self.corr_gear.add(float(v), float(refs["gear"]))


def parse_data_bytes(data_str: str) -> bytes:
    if data_str.startswith("0x") or data_str.startswith("0X"):
        data_str = data_str[2:]
    return bytes.fromhex(data_str)


def safe_int(s: str, base: int = 10) -> Optional[int]:
    try:
        return int(s, base)
    except Exception:
        return None


def load_known_ids(default_decoder_path: str) -> Set[int]:
    path = pathlib.Path(default_decoder_path)
    if not path.exists():
        return set()
    try:
        spec = importlib.util.spec_from_file_location("decode_log_mod", path)
        if spec is None or spec.loader is None:
            return set()
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
        messages = getattr(mod, "MESSAGES", {})
        if isinstance(messages, dict):
            return {int(k) for k in messages.keys()}
    except Exception:
        return set()
    return set()


def detect_sessions(input_file: str) -> Dict[int, Dict[str, int]]:
    sessions: Dict[int, Dict[str, int]] = {}
    prev_seq: Optional[int] = None
    sid = 0
    with open(input_file, "r", newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for row in reader:
            seq = safe_int(row.get("seq", ""), 10)
            ts = safe_int(row.get("timestamp_ms", ""), 10)
            if seq is None or ts is None:
                continue
            if prev_seq is not None and seq <= prev_seq:
                sid += 1
            prev_seq = seq

            s = sessions.setdefault(
                sid,
                {"rows": 0, "seq_start": seq, "seq_end": seq, "ts_start": ts, "ts_end": ts},
            )
            s["rows"] += 1
            s["seq_end"] = seq
            s["ts_end"] = ts
    return sessions


def list_sessions(input_file: str) -> None:
    sessions = detect_sessions(input_file)
    print("Detected sessions (0-based):")
    print("-" * 72)
    for sid in sorted(sessions.keys()):
        s = sessions[sid]
        dur_s = (s["ts_end"] - s["ts_start"]) / 1000.0
        print(
            f"  session={sid:2d} rows={s['rows']:8d} "
            f"seq={s['seq_start']}..{s['seq_end']} "
            f"time_ms={s['ts_start']}..{s['ts_end']} ({dur_s:.1f}s)"
        )


def first_pass_id_counts(input_file: str, session_filter: Optional[int]) -> Tuple[Counter, Dict[int, int], Dict[int, int]]:
    id_counts: Counter = Counter()
    first_ts: Dict[int, int] = {}
    last_ts: Dict[int, int] = {}

    prev_seq: Optional[int] = None
    sid = 0
    with open(input_file, "r", newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for row in reader:
            seq = safe_int(row.get("seq", ""), 10)
            ts = safe_int(row.get("timestamp_ms", ""), 10)
            cid = safe_int(row.get("id", ""), 16)
            if seq is None or ts is None or cid is None:
                continue
            if prev_seq is not None and seq <= prev_seq:
                sid += 1
            prev_seq = seq

            if session_filter is not None and sid != session_filter:
                continue

            id_counts[cid] += 1
            first_ts.setdefault(cid, ts)
            last_ts[cid] = ts

    return id_counts, first_ts, last_ts


def ref_update(cid: int, data: bytes, refs: Dict[str, Optional[float]]) -> None:
    if cid == 0x354 and len(data) >= 2:
        refs["speed"] = ((data[0] << 8) | data[1]) * 0.00621
    elif cid == 0x180 and len(data) >= 2:
        refs["rpm"] = ((data[0] << 8) | data[1]) * 0.125
    elif cid == 0x35D and len(data) >= 5:
        refs["brake"] = float((data[4] >> 4) & 0x01)
    elif cid == 0x174 and len(data) >= 4:
        refs["gear"] = float(data[3] & 0x0F)


def counter_likeness(hist: Counter) -> Tuple[float, float]:
    nonzero = sum(v for k, v in hist.items() if k != 0)
    if nonzero <= 0:
        return 0.0, 0.0
    plus1 = hist.get(1, 0)
    small = hist.get(1, 0) + hist.get(255, 0) + hist.get(2, 0) + hist.get(254, 0)
    return plus1 / nonzero, small / nonzero


def fmt_corr(v: Optional[float]) -> str:
    return "n/a" if v is None else f"{v:.3f}"


def second_pass_analyze(
    input_file: str,
    target_ids: Set[int],
    session_filter: Optional[int],
) -> Dict[int, Dict[int, ByteStats]]:
    stats: Dict[int, Dict[int, ByteStats]] = {cid: defaultdict(ByteStats) for cid in target_ids}
    refs = {"speed": None, "rpm": None, "brake": None, "gear": None}

    prev_seq: Optional[int] = None
    sid = 0
    with open(input_file, "r", newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for row in reader:
            seq = safe_int(row.get("seq", ""), 10)
            cid = safe_int(row.get("id", ""), 16)
            data_str = row.get("data", "")
            if seq is None or cid is None or not data_str:
                continue
            if prev_seq is not None and seq <= prev_seq:
                sid += 1
            prev_seq = seq

            if session_filter is not None and sid != session_filter:
                continue

            try:
                data = parse_data_bytes(data_str)
            except Exception:
                continue

            ref_update(cid, data, refs)
            if cid not in target_ids:
                continue

            for bi, b in enumerate(data):
                stats[cid][bi].add(b, refs)

    return stats


def classify_byte(bs: ByteStats) -> str:
    c_speed = bs.corr_speed.corr()
    c_rpm = bs.corr_rpm.corr()
    c_brake = bs.corr_brake.corr()
    p1, small = counter_likeness(bs.delta_hist)

    tags: List[str] = []
    if c_speed is not None and abs(c_speed) >= 0.85:
        tags.append("speed-like")
    if c_rpm is not None and abs(c_rpm) >= 0.85:
        tags.append("rpm-like")
    if c_brake is not None and abs(c_brake) >= 0.4:
        tags.append("brake-linked")
    if small >= 0.9 and len(bs.uniq) >= 8:
        tags.append("counter-like")
    if not tags and len(bs.uniq) <= 4:
        tags.append("state/flag")
    return ", ".join(tags) if tags else "-"


def score_byte(bs: ByteStats) -> float:
    change_rate = bs.changes / max(1, bs.count - 1)
    c_speed = abs(bs.corr_speed.corr() or 0.0)
    c_rpm = abs(bs.corr_rpm.corr() or 0.0)
    c_brake = abs(bs.corr_brake.corr() or 0.0)
    p1, small = counter_likeness(bs.delta_hist)
    uniq_score = min(1.0, len(bs.uniq) / 64.0)
    return (2.0 * max(c_speed, c_rpm, c_brake)) + (1.5 * change_rate) + (1.0 * small) + (0.5 * uniq_score)


def parse_plot_channels(spec: str) -> List[Tuple[int, int]]:
    out: List[Tuple[int, int]] = []
    for tok in [t.strip() for t in spec.split(",") if t.strip()]:
        tok_u = tok.upper()
        if ".B" not in tok_u:
            raise ValueError(f"Invalid plot channel '{tok}'. Use 0x123.B2 format.")
        id_part, b_part = tok_u.split(".B", 1)
        cid = int(id_part, 16)
        bi = int(b_part, 10)
        out.append((cid, bi))
    return out


def collect_plot_series(
    input_file: str,
    channels: List[Tuple[int, int]],
    session_filter: Optional[int],
    include_refs: bool = True,
) -> Dict[str, Tuple[List[float], List[float]]]:
    want = set(channels)
    out: Dict[str, Tuple[List[float], List[float]]] = {}
    for cid, bi in channels:
        out[f"0x{cid:03X}.B{bi}"] = ([], [])
    if include_refs:
        out["REF.Speed"] = ([], [])
        out["REF.RPM"] = ([], [])
        out["REF.Brake"] = ([], [])
        out["REF.Gear"] = ([], [])
    ref_label = {"speed": "REF.Speed", "rpm": "REF.RPM", "brake": "REF.Brake", "gear": "REF.Gear"}

    refs = {"speed": None, "rpm": None, "brake": None, "gear": None}
    t0: Optional[float] = None

    prev_seq: Optional[int] = None
    sid = 0
    with open(input_file, "r", newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for row in reader:
            seq = safe_int(row.get("seq", ""), 10)
            ts_ms = safe_int(row.get("timestamp_ms", ""), 10)
            cid = safe_int(row.get("id", ""), 16)
            data_str = row.get("data", "")
            if seq is None or ts_ms is None or cid is None or not data_str:
                continue
            if prev_seq is not None and seq <= prev_seq:
                sid += 1
            prev_seq = seq

            if session_filter is not None and sid != session_filter:
                continue

            try:
                data = parse_data_bytes(data_str)
            except Exception:
                continue

            ref_update(cid, data, refs)
            t = ts_ms / 1000.0
            if t0 is None:
                t0 = t
            t_rel = t - t0

            if include_refs:
                for k in ("speed", "rpm", "brake", "gear"):
                    if refs[k] is not None:
                        out[ref_label[k]][0].append(t_rel)
                        out[ref_label[k]][1].append(float(refs[k]))

            for ch_cid, ch_bi in want:
                if cid == ch_cid and ch_bi < len(data):
                    name = f"0x{ch_cid:03X}.B{ch_bi}"
                    out[name][0].append(t_rel)
                    out[name][1].append(float(data[ch_bi]))

    return out


def plot_series(
    series: Dict[str, Tuple[List[float], List[float]]],
    output_file: Optional[str],
    subplots: bool,
) -> None:
    if not HAS_MATPLOTLIB:
        print("Error: matplotlib is required for plotting. Install with: pip install matplotlib", file=sys.stderr)
        return

    names = [k for k, (t, v) in series.items() if t and v]
    if not names:
        print("No data to plot.")
        return

    if subplots:
        fig, axes = plt.subplots(len(names), 1, figsize=(12, max(3, 2.6 * len(names))), sharex=True)
        if len(names) == 1:
            axes = [axes]
        for ax, name in zip(axes, names):
            t, v = series[name]
            ax.plot(t, v, linewidth=0.6)
            ax.set_ylabel(name)
            ax.grid(True, alpha=0.3)
        axes[-1].set_xlabel("Time (s)")
        fig.suptitle("Unknown Signal/Byte Plot")
        plt.tight_layout()
    else:
        plt.figure(figsize=(12, 6))
        for name in names:
            t, v = series[name]
            plt.plot(t, v, linewidth=0.6, label=name)
        plt.xlabel("Time (s)")
        plt.ylabel("Raw Value / Ref")
        plt.title("Unknown Signal/Byte Plot")
        plt.grid(True, alpha=0.3)
        plt.legend(loc="upper right", ncol=2, fontsize=8)
        plt.tight_layout()

    if output_file:
        plt.savefig(output_file, dpi=150, bbox_inches="tight")
        print(f"Saved plot to {output_file}")
    else:
        plt.show()


def main() -> int:
    parser = argparse.ArgumentParser(description="Rank unknown CAN IDs/bytes and infer likely meaning.")
    parser.add_argument("input", help="Input CAN log CSV (canlog.csv)")
    parser.add_argument("--decoder", default="scripts/decode_log.py", help="Path to decoder with MESSAGES map")
    parser.add_argument("--session", type=int, help="Analyze one session index (0-based, by seq reset)")
    parser.add_argument("--list-sessions", action="store_true", help="List detected sessions and exit")
    parser.add_argument("--top-ids", type=int, default=8, help="How many unknown IDs to analyze by frame count")
    parser.add_argument("--min-frames", type=int, default=1000, help="Minimum frames for unknown ID analysis")
    parser.add_argument(
        "--plot",
        type=str,
        help="Comma-separated channels to plot, e.g. 0x176.B0,0x177.B2",
    )
    parser.add_argument(
        "--plot-top-bytes",
        type=int,
        default=0,
        help="Plot top-N ranked unknown bytes automatically (if --plot omitted)",
    )
    parser.add_argument("--output", type=str, help="Plot output image path (png/pdf)")
    parser.add_argument("--subplots", action="store_true", help="Use one subplot per series")
    args = parser.parse_args()

    if args.list_sessions:
        list_sessions(args.input)
        return 0

    known_ids = load_known_ids(args.decoder)
    if not known_ids:
        print("Warning: could not load known IDs from decoder; treating all IDs as unknown.", file=sys.stderr)

    id_counts, first_ts, last_ts = first_pass_id_counts(args.input, args.session)
    unknown = [(cid, c) for cid, c in id_counts.items() if cid not in known_ids and c >= args.min_frames]
    unknown.sort(key=lambda kv: kv[1], reverse=True)
    unknown = unknown[: args.top_ids]

    if not unknown:
        print("No unknown IDs matched current filters.")
        return 0

    print("Unknown ID candidates:")
    print("-" * 86)
    for cid, c in unknown:
        dur_ms = max(1, last_ts[cid] - first_ts[cid])
        hz = c / (dur_ms / 1000.0)
        print(f"0x{cid:03X}  frames={c:8d}  est_hz={hz:7.2f}  span_ms={dur_ms}")

    target_ids = {cid for cid, _ in unknown}
    stats = second_pass_analyze(args.input, target_ids, args.session)

    ranked_bytes: List[Tuple[float, int, int, ByteStats]] = []
    print("\nTop unknown bytes by inference score:")
    print("-" * 140)
    print(
        "score  channel    uniq  min  max   changes%   corr(speed)  corr(rpm)  corr(brake)  corr(gear)  +1/nonzero  smallstep   class"
    )
    for cid in sorted(stats.keys()):
        for bi in sorted(stats[cid].keys()):
            bs = stats[cid][bi]
            if bs.count < 100:
                continue
            score = score_byte(bs)
            ranked_bytes.append((score, cid, bi, bs))

    ranked_bytes.sort(key=lambda x: x[0], reverse=True)
    for score, cid, bi, bs in ranked_bytes[:60]:
        p1, small = counter_likeness(bs.delta_hist)
        change_pct = 100.0 * bs.changes / max(1, bs.count - 1)
        print(
            f"{score:5.2f}  0x{cid:03X}.B{bi:<2d}   {len(bs.uniq):4d} {bs.min_v:4d} {bs.max_v:4d}  "
            f"{change_pct:8.2f}%   {fmt_corr(bs.corr_speed.corr()):>11}  {fmt_corr(bs.corr_rpm.corr()):>9}  "
            f"{fmt_corr(bs.corr_brake.corr()):>11}  {fmt_corr(bs.corr_gear.corr()):>10}  {p1:10.3f}  {small:9.3f}   "
            f"{classify_byte(bs)}"
        )

    plot_channels: List[Tuple[int, int]] = []
    if args.plot:
        plot_channels = parse_plot_channels(args.plot)
    elif args.plot_top_bytes > 0:
        plot_channels = [(cid, bi) for _, cid, bi, _ in ranked_bytes[: args.plot_top_bytes]]

    if plot_channels:
        series = collect_plot_series(args.input, plot_channels, args.session, include_refs=True)
        plot_series(series, args.output, args.subplots)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
