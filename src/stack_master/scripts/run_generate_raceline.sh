#!/usr/bin/env bash
# Usage: ./run_generate_raceline.sh teras
set -e
MAP="$1"
if [ -z "$MAP" ]; then
  echo "Usage: $0 <mapname> (e.g. teras)"
  exit 1
fi

ROOT="$(cd "$(dirname "$0")/../.." && pwd)"  # points to src
MAP_DIR="$ROOT/stack_master/maps/$MAP"
CENTER="$MAP_DIR/centerline.csv"
RACELINE="$MAP_DIR/raceline.csv"

# 1) JSON -> CSV (if centerline missing)
if [ ! -f "$CENTER" ]; then
  echo "[1/3] Converting JSON -> CSV for $MAP"
  python3 "$ROOT/stack_master/scripts/json_to_csv.py" --in "$MAP_DIR/global_waypoints.json" --out "$CENTER"
fi

# 2) Generate raceline via installed C++ binary (ros2 run) or built binary
echo "[2/3] Generating raceline (C++ generator)"
if ros2 pkg prefix planning_pkg >/dev/null 2>&1; then
  ros2 run planning_pkg generate_raceline --centerline_csv "$CENTER" --out_csv "$RACELINE" --ds 0.25 --mu 1.0 --v_max 6.0
else
  BIN="$(pwd)/install/lib/planning_pkg/generate_raceline"
  if [ -x "$BIN" ]; then
    "$BIN" --centerline_csv "$CENTER" --out_csv "$RACELINE" --ds 0.25 --mu 1.0 --v_max 6.0
  else
    echo "generate_raceline not found. Build planning_pkg or install it."
    exit 2
  fi
fi

# 3) Show head of raceline
echo "[3/3] raceline head:"
head -n 10 "$RACELINE" || true
echo "Done. Raceline: $RACELINE"
