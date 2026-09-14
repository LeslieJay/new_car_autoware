#!/usr/bin/env python3
"""Create the compact CSV/JSON summary for one reusable Autoware sweep."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path


FIELDS = [
    "longitudinal_m",
    "result",
    "clear_confirmed",
    "object_count",
    "has_simple_avoidance",
    "has_obstacle_stop",
    "has_dynamic_obstacle_stop",
    "min_collision_clearance_m",
    "vehicle_passed",
    "early_stop_reason",
    "timeout",
    "bag_path",
    "log_path",
]


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("log_root", type=Path)
    parser.add_argument("--run-id", required=True)
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()

    rows = []
    for result_path in sorted(
        args.log_root.glob(f"[0-9]*m/reuse_{args.run_id}/result.json")
    ):
        item = json.loads(result_path.read_text(encoding="utf-8"))
        log_paths = item.get("log_paths") or [str(result_path.parent / "case_window.log")]
        item["log_path"] = log_paths[-1]
        rows.append({key: item.get(key) for key in FIELDS})

    args.out.mkdir(parents=True, exist_ok=True)
    csv_path = args.out / "summary.csv"
    with csv_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=FIELDS)
        writer.writeheader()
        writer.writerows(rows)
    (args.out / "summary.json").write_text(
        json.dumps(rows, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    print(csv_path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
