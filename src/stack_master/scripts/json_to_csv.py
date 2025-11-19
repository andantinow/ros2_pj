#!/usr/bin/env python3
"""
Robust JSON -> centerline CSV converter.
Searches JSON for a list of waypoint entries (dicts with x/y keys or [x,y] lists).
Writes CSV with header: x,y
Usage:
  python3 src/stack_master/scripts/json_to_csv.py --in <global_waypoints.json> --out <centerline.csv>
"""
import json
import csv
import argparse
import os
import sys
from collections.abc import Mapping, Sequence

def get_xy_from_entry(e):
    # Try common key names
    if isinstance(e, Mapping):
        # direct keys
        for kx, ky in (('x_m','y_m'), ('x','y'), ('px','py'), ('lon','lat'), ('X','Y')):
            if kx in e and ky in e:
                try:
                    return float(e[kx]), float(e[ky])
                except Exception:
                    pass
        # nested 'position' or similar
        for key in ('position','pos','pose','point'):
            if key in e and isinstance(e[key], Mapping):
                for kx, ky in (('x_m','y_m'), ('x','y'), ('px','py'), ('lon','lat')):
                    if kx in e[key] and ky in e[key]:
                        try:
                            return float(e[key][kx]), float(e[key][ky])
                        except Exception:
                            pass
        # try any numeric pair in dict (best-effort)
        nums = {k: v for k, v in e.items() if isinstance(v, (int, float))}
        if len(nums) >= 2:
            items = list(nums.items())
            return float(items[0][1]), float(items[1][1])
    elif isinstance(e, Sequence) and not isinstance(e, (str, bytes)):
        if len(e) >= 2 and isinstance(e[0], (int, float)) and isinstance(e[1], (int, float)):
            return float(e[0]), float(e[1])
    return None

def is_waypoint_list(obj):
    # true if obj is a list-like and many elements yield xy
    if not isinstance(obj, Sequence) or isinstance(obj, (str, bytes)):
        return False
    cnt = 0
    total = 0
    for it in obj:
        total += 1
        if get_xy_from_entry(it) is not None:
            cnt += 1
        if total >= 200:  # sample limit for large lists
            break
    if total == 0:
        return False
    # require >50% or at least 5 points matching
    return (cnt >= max(5, int(total*0.5)))

def find_waypoint_list(obj, path="root"):
    # Depth-first search for a candidate list
    if is_waypoint_list(obj):
        return obj, path
    if isinstance(obj, Mapping):
        for k, v in obj.items():
            res = find_waypoint_list(v, path + "/" + str(k))
            if res is not None:
                return res
    elif isinstance(obj, Sequence) and not isinstance(obj, (str, bytes)):
        for idx, v in enumerate(obj):
            res = find_waypoint_list(v, path + f"[{idx}]")
            if res is not None:
                return res
    return None

def main():
    p = argparse.ArgumentParser()
    p.add_argument('--in', dest='infile', required=True)
    p.add_argument('--out', dest='outfile', required=True)
    args = p.parse_args()

    if not os.path.exists(args.infile):
        print("Input not found:", args.infile, file=sys.stderr)
        return 2

    try:
        with open(args.infile, 'r') as f:
            data = json.load(f)
    except Exception as e:
        print("Failed to parse JSON:", e, file=sys.stderr)
        return 3

    # quick cases
    if is_waypoint_list(data):
        wp_list = data
        path = "root"
    else:
        found = find_waypoint_list(data)
        if found is None:
            print("Unexpected JSON structure: cannot find waypoint list", file=sys.stderr)
            return 4
        wp_list, path = found

    # extract points
    points = []
    for entry in wp_list:
        xy = get_xy_from_entry(entry)
        if xy is None:
            # skip non-matching entries
            continue
        points.append(xy)

    if len(points) < 2:
        print("Too few extracted points:", len(points), file=sys.stderr)
        return 5

    os.makedirs(os.path.dirname(args.outfile) or '.', exist_ok=True)
    with open(args.outfile, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['x','y'])  # IMPORTANT: header for generate_raceline.cpp
        for x,y in points:
            w.writerow([f"{x:.6f}", f"{y:.6f}"])

    print(f"Wrote centerline CSV: {args.outfile} points:{len(points)} (from {path})")
    return 0

if __name__ == "__main__":
    raise SystemExit(main())
