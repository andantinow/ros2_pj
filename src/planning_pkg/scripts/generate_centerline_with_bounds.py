#!/usr/bin/env python3
"""
Generate a centerline CSV with boundary information from global_waypoints.json.

This script extracts centerline points along with track width info (d_left, d_right)
from the stack_master global_waypoints.json file, creating a CSV that can be used
by the generate_raceline tool.

Usage:
    python3 generate_centerline_with_bounds.py \
        --json /path/to/global_waypoints.json \
        --out /path/to/centerline_with_bounds.csv
"""

import argparse
import json
from pathlib import Path


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Generate centerline CSV with track boundaries from global_waypoints.json"
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
        help="Destination CSV path (x,y,d_left,d_right,psi columns)",
    )
    parser.add_argument(
        "--default-width",
        type=float,
        default=0.5,
        help="Default track half-width if d_left/d_right not in data (default: 0.5m)",
    )
    args = parser.parse_args()

    # Load waypoints
    with args.json.open() as f:
        data = json.load(f)

    if args.section not in data:
        raise KeyError(f"Section '{args.section}' not found in {args.json}")

    section_data = data[args.section]
    if args.entry_key not in section_data:
        raise KeyError(f"Entry key '{args.entry_key}' not in '{args.section}'")

    waypoints = section_data[args.entry_key]
    if not isinstance(waypoints, list):
        raise TypeError(f"Expected list for {args.section}.{args.entry_key}")

    # Sort by waypoint id for deterministic ordering
    waypoints = sorted(waypoints, key=lambda w: w.get("id", 0))

    # Write CSV
    args.out.parent.mkdir(parents=True, exist_ok=True)
    with args.out.open("w") as f:
        f.write("x,y,d_left,d_right,psi\n")
        for wp in waypoints:
            x = wp.get("x_m")
            y = wp.get("y_m")
            d_left = wp.get("d_left", args.default_width)
            d_right = wp.get("d_right", args.default_width)
            psi = wp.get("psi_rad", 0.0)
            
            if x is None or y is None:
                raise ValueError("Waypoint missing 'x_m' or 'y_m'")
            
            f.write(f"{x},{y},{d_left},{d_right},{psi}\n")

    print(f"Wrote {args.out} ({len(waypoints)} points)")
    print(f"  X range: {min(w['x_m'] for w in waypoints):.3f} to {max(w['x_m'] for w in waypoints):.3f}")
    print(f"  Y range: {min(w['y_m'] for w in waypoints):.3f} to {max(w['y_m'] for w in waypoints):.3f}")


if __name__ == "__main__":
    main()
