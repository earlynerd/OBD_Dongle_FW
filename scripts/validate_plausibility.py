#!/usr/bin/env python3
"""
Validate decoded CAN signal plausibility from decoded.csv.

Usage:
  python scripts/validate_plausibility.py decoded.csv
"""

from __future__ import annotations

import csv
import math
import sys
from dataclasses import dataclass
from typing import Dict, Iterable, List, Optional, Tuple


@dataclass
class Stats:
    n: int = 0
    min_v: Optional[float] = None
    max_v: Optional[float] = None
    sum_v: float = 0.0
    sum2_v: float = 0.0

    def add(self, v: float) -> None:
        self.n += 1
        self.min_v = v if self.min_v is None else min(self.min_v, v)
        self.max_v = v if self.max_v is None else max(self.max_v, v)
        self.sum_v += v
        self.sum2_v += v * v

    def mean(self) -> float:
        return self.sum_v / self.n if self.n else 0.0

    def sd(self) -> float:
        if not self.n:
            return 0.0
        var = (self.sum2_v / self.n) - (self.mean() ** 2)
        return math.sqrt(max(0.0, var))


def as_float(value: str) -> Optional[float]:
    if value is None:
        return None
    v = value.strip()
    if not v:
        return None
    try:
        return float(v)
    except ValueError:
        return None


def main(argv: List[str]) -> int:
    if len(argv) != 2:
        print("Usage: python scripts/validate_plausibility.py <decoded.csv>")
        return 2

    decoded_path = argv[1]
    with open(decoded_path, newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        if reader.fieldnames is None:
            print("Input CSV is missing a header row.")
            return 2

        numeric_fields = [
            n for n in reader.fieldnames if n not in ("seq", "timestamp_ms", "id", "decoded")
        ]
        stats: Dict[str, Stats] = {name: Stats() for name in numeric_fields}

        suspect_rows: List[Tuple[str, str, str]] = []
        counters = {
            "speed2_gt_200": 0,
            "speed2_max_sentinel_406p972": 0,
            "imu_sentinel_all_2047_or_255": 0,
            "wheel_speed_mismatch_gt10mph": 0,
            "stopped_but_speed_gt2": 0,
        }

        total_rows = 0
        for row in reader:
            total_rows += 1

            vals: Dict[str, Optional[float]] = {}
            for field in numeric_fields:
                x = as_float(row.get(field, ""))
                vals[field] = x
                if x is not None:
                    stats[field].add(x)

            speed = vals.get("Speed")
            speed2 = vals.get("Speed2")
            stopped = vals.get("Stopped")
            lat = vals.get("LatAccel")
            yaw = vals.get("Yaw")
            brake_pres = vals.get("BrakePres")
            wfl = vals.get("WheelFL")
            wfr = vals.get("WheelFR")
            wrl = vals.get("WheelRL")
            wrr = vals.get("WheelRR")

            if speed2 is not None and speed2 > 200:
                counters["speed2_gt_200"] += 1
                if abs(speed2 - 406.97235) < 1e-3:
                    counters["speed2_max_sentinel_406p972"] += 1
                if len(suspect_rows) < 20:
                    suspect_rows.append((row["seq"], row["id"], row["decoded"]))

            if (
                lat is not None
                and yaw is not None
                and brake_pres is not None
                and (lat in (2047.0, -2048.0) or yaw in (2047.0, -2048.0) or brake_pres == 255.0)
            ):
                counters["imu_sentinel_all_2047_or_255"] += 1
                if len(suspect_rows) < 20:
                    suspect_rows.append((row["seq"], row["id"], row["decoded"]))

            wheels = [x for x in (wfl, wfr, wrl, wrr) if x is not None]
            if speed is not None and len(wheels) >= 2:
                wheel_mean = sum(wheels) / len(wheels)
                if abs(wheel_mean - speed) > 10.0 and max(wheel_mean, speed) > 15.0:
                    counters["wheel_speed_mismatch_gt10mph"] += 1

            if speed is not None and stopped is not None and stopped > 0.5 and speed > 2.0:
                counters["stopped_but_speed_gt2"] += 1

    print(f"Rows scanned: {total_rows}")
    print("\nMost populated numeric signals:")
    top = sorted(stats.items(), key=lambda kv: kv[1].n, reverse=True)[:20]
    for name, s in top:
        if s.n == 0:
            continue
        print(
            f"{name:14} n={s.n:7d} min={s.min_v:9.3f} max={s.max_v:9.3f} "
            f"mean={s.mean():9.3f} sd={s.sd():9.3f}"
        )

    print("\nRange checks:")
    limits = {
        "Speed": (0, 200),
        "Speed1": (0, 200),
        "Speed2": (0, 200),
        "RPM": (0, 10000),
        "Throttle": (0, 100),
        "Pedal": (0, 100),
        "Pedal1": (0, 100),
        "Coolant": (-40, 150),
        "OilTemp": (-40, 180),
        "LatAccel": (-5, 5),
        "Yaw": (-200, 200),
        "WheelFL": (0, 200),
        "WheelFR": (0, 200),
        "WheelRL": (0, 200),
        "WheelRR": (0, 200),
    }
    for field, (lo, hi) in limits.items():
        s = stats.get(field)
        if not s or s.n == 0:
            continue
        if s.min_v is not None and s.max_v is not None and (s.min_v < lo or s.max_v > hi):
            print(f"{field:12} min={s.min_v:.3f} max={s.max_v:.3f} expected=[{lo},{hi}]")

    print("\nConsistency counters:")
    for k, v in counters.items():
        print(f"{k}: {v}")

    if suspect_rows:
        print("\nSample suspect decoded rows:")
        for seq, can_id, decoded in suspect_rows[:10]:
            print(f"seq={seq} id={can_id} decoded={decoded}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
