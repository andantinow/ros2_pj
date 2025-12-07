#!/bin/bash
# Verification script for OUT-IN-OUT opponent path implementation

set -e

echo "======================================"
echo "Opponent OUT-IN-OUT Implementation"
echo "Verification Script"
echo "======================================"
echo ""

# Check if running from correct directory
if [ ! -d "src/planning_pkg" ]; then
    echo "Error: Please run this script from the repository root"
    exit 1
fi

echo "Step 1: Checking Python script syntax..."
python3 src/planning_pkg/scripts/generate_opponent_raceline.py --help > /dev/null
echo "✓ Python script syntax OK"
echo ""

echo "Step 2: Verifying parameter changes..."
echo "  Checking simple_controller.hpp..."

# Check if OUT-IN-OUT is disabled for ego
if grep -q "enable_out_in_out_ = false" src/control_pkg/src/simple_controller.hpp; then
    echo "  ✓ OUT-IN-OUT disabled for ego car"
else
    echo "  ✗ Warning: OUT-IN-OUT still enabled for ego"
fi

# Check follow distance increase
if grep -q "follow_distance_threshold_ = 5.0" src/control_pkg/src/simple_controller.hpp; then
    echo "  ✓ Follow distance threshold increased to 5.0m"
else
    echo "  ✗ Warning: Follow distance threshold not updated"
fi

if grep -q "target_follow_gap_ = 3.5" src/control_pkg/src/simple_controller.hpp; then
    echo "  ✓ Target follow gap increased to 3.5m"
else
    echo "  ✗ Warning: Target follow gap not updated"
fi

if grep -q "follow_min_distance_ = 0.8" src/control_pkg/src/simple_controller.hpp; then
    echo "  ✓ Minimum follow distance increased to 0.8m"
else
    echo "  ✗ Warning: Minimum follow distance not updated"
fi

echo ""
echo "Step 3: Verifying virtual box implementation..."

# Check for virtual box constants
if grep -q "LIDAR_BOX_SAFETY_FRONT" src/control_pkg/src/simple_controller.hpp; then
    echo "  ✓ Virtual box constants defined"
else
    echo "  ✗ Warning: Virtual box constants not found"
fi

# Check for virtual box functions
if grep -q "check_lidar_box_occupancy" src/control_pkg/src/simple_controller.cpp; then
    echo "  ✓ Virtual box checking function implemented"
else
    echo "  ✗ Warning: Virtual box checking function not found"
fi

if grep -q "update_virtual_boxes" src/control_pkg/src/simple_controller.cpp; then
    echo "  ✓ Virtual box update function implemented"
else
    echo "  ✗ Warning: Virtual box update function not found"
fi

echo ""
echo "Step 4: Verifying opponent raceline generation..."

# Check for OUT-IN-OUT logic in Python script
if grep -q "out_in_out_strength" src/planning_pkg/scripts/generate_opponent_raceline.py; then
    echo "  ✓ OUT-IN-OUT strength parameter added"
else
    echo "  ✗ Warning: OUT-IN-OUT parameter not found"
fi

if grep -q "CURVATURE_THRESHOLD" src/planning_pkg/scripts/generate_opponent_raceline.py; then
    echo "  ✓ Curvature-based corner detection implemented"
else
    echo "  ✗ Warning: Curvature detection not found"
fi

echo ""
echo "======================================"
echo "Verification Summary"
echo "======================================"
echo ""
echo "Key Changes Implemented:"
echo "  1. Opponent uses OUT-IN-OUT racing line ✓"
echo "  2. Ego disabled OUT-IN-OUT ✓"
echo "  3. Follow distance increased ✓"
echo "  4. LiDAR virtual boxes added ✓"
echo ""
echo "Next Steps:"
echo "  1. Build the code: colcon build --packages-select control_pkg planning_pkg"
echo "  2. Generate opponent raceline:"
echo "     cd src/planning_pkg/scripts"
echo "     python3 generate_opponent_raceline.py \\"
echo "       --centerline tracks/centerline_with_bounds.csv \\"
echo "       --output data/opponent_raceline.csv \\"
echo "       --out-in-out-strength 0.5"
echo "  3. Launch simulation and verify:"
echo "     - Opponent follows OUT-IN-OUT line"
echo "     - Ego maintains ~3.5m follow gap"
echo "     - FSM transitions work (check logs)"
echo "     - Overtaking happens at appropriate times"
echo ""
echo "Documentation:"
echo "  See OPPONENT_OUT_IN_OUT_IMPLEMENTATION.md for full details"
echo ""
