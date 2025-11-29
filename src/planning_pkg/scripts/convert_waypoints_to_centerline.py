#!/usr/bin/env python3

import argparse
import json
from pathlib import Path
from typing import Iterable, List


def load_waypoints(json_path: Path, section: str, entry_key: str) -> List[dict]:
    with json_path.open() as handle:
        data = json.load(handle)

    if section not in data:
        raise KeyError(f"Section '{section}' not found in {json_path}")

    section_data = data[section]
    if entry_key not in section_data:
        raise KeyError(
            f"Entry key '{entry_key}' not present under '{section}' in {json_path}"
        )

    waypoints = section_data[entry_key]
    if not isinstance(waypoints, list):
        raise TypeError(f"Expected list for {section}.{entry_key}")

    # Sort by waypoint id to ensure deterministic ordering
    return sorted(waypoints, key=lambda item: item.get("id", 0))


def write_centerline_csv(out_path: Path, waypoints: Iterable[dict]) -> None:
    out_path.parent.mkdir(parents=True, exist_ok=True)
    with out_path.open("w") as csv_file:
        csv_file.write("x,y\n")
        for wp in waypoints:
            x = wp.get("x_m")
            y = wp.get("y_m")
            if x is None or y is None:
                raise ValueError("Waypoint missing 'x_m' or 'y_m'")
            csv_file.write(f"{x:.9f},{y:.9f}\n")


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Convert stack_master global_waypoints.json into a simple centerline CSV."
    )
    parser.add_argument(
        "--json",
        required=True,
        type=Path,
        help="Path to global_waypoints.json",
    )
    parser.add_argument(
        "--section",
        default="centerline_waypoints",
        help="Top-level section containing waypoint list (default: centerline_waypoints)",
    )
    parser.add_argument(
        "--entry-key",
        default="wpnts",
        help="Key under the section that stores the waypoint array (default: wpnts)",
    )
    parser.add_argument(
        "--out",
        required=True,
        type=Path,
        help="Destination CSV path (x,y columns)",
    )
    args = parser.parse_args()

    waypoints = load_waypoints(args.json, args.section, args.entry_key)
    write_centerline_csv(args.out, waypoints)


if __name__ == "__main__":
    main()

