#!/bin/bash
# Verification script for widened raceline implementation

echo "================================================"
echo "Widened Raceline Implementation Verification"
echo "================================================"
echo ""

# Color codes
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

SUCCESS=0
WARNINGS=0
FAILURES=0

check_file() {
    local file=$1
    local desc=$2
    if [ -f "$file" ]; then
        echo -e "${GREEN}✓${NC} $desc: $file"
        ((SUCCESS++))
    else
        echo -e "${RED}✗${NC} $desc: $file (NOT FOUND)"
        ((FAILURES++))
    fi
}

check_dir() {
    local dir=$1
    local desc=$2
    if [ -d "$dir" ]; then
        echo -e "${GREEN}✓${NC} $desc: $dir"
        ((SUCCESS++))
    else
        echo -e "${RED}✗${NC} $desc: $dir (NOT FOUND)"
        ((FAILURES++))
    fi
}

echo "=== Checking Generated Racelines ==="
check_file "src/planning_pkg/data/raceline.csv" "Main raceline (widened)"
check_file "src/planning_pkg/data/opponent_raceline.csv" "Opponent raceline (outside bias)"
check_file "src/planning_pkg/data/raceline_original.csv" "Original raceline (backup)"
check_file "src/planning_pkg/data/raceline_widened.csv" "Widened raceline (source)"
echo ""

echo "=== Checking Source Files ==="
check_file "src/planning_pkg/scripts/generate_opponent_raceline.py" "Raceline generator script"
check_file "src/planning_pkg/tracks/centerline_with_bounds.csv" "Centerline with boundaries"
echo ""

echo "=== Checking Modified Files ==="
check_file "src/project_launch/launch/main.launch.py" "Main launch file"
check_file "src/utilities/nodes/opponent_publisher_cpp/launch/opponent_publisher_launch.xml" "Opponent launch file"
echo ""

echo "=== Checking Documentation ==="
check_file "WIDENED_RACELINE_IMPLEMENTATION.md" "Implementation documentation"
check_file "QUICK_REFERENCE_WIDENED.md" "Quick reference guide"
check_file "src/planning_pkg/README.md" "Planning package README"
echo ""

echo "=== Checking Launch Configuration ==="
if grep -q "opponent_raceline_node" src/project_launch/launch/main.launch.py; then
    echo -e "${GREEN}✓${NC} Opponent raceline server configured in launch file"
    ((SUCCESS++))
else
    echo -e "${RED}✗${NC} Opponent raceline server NOT configured in launch file"
    ((FAILURES++))
fi

if grep -q "path_topic.*opponent_raceline" src/project_launch/launch/main.launch.py; then
    echo -e "${GREEN}✓${NC} Opponent configured to use opponent raceline topic"
    ((SUCCESS++))
else
    echo -e "${RED}✗${NC} Opponent NOT configured to use opponent raceline topic"
    ((FAILURES++))
fi

if grep -q "path_topic" src/utilities/nodes/opponent_publisher_cpp/launch/opponent_publisher_launch.xml; then
    echo -e "${GREEN}✓${NC} Opponent launch file accepts path_topic parameter"
    ((SUCCESS++))
else
    echo -e "${RED}✗${NC} Opponent launch file does NOT accept path_topic parameter"
    ((FAILURES++))
fi
echo ""

echo "=== Verifying Raceline Contents ==="
# Check that opponent raceline has data
OPP_LINES=$(wc -l < src/planning_pkg/data/opponent_raceline.csv)
if [ "$OPP_LINES" -gt 10 ]; then
    echo -e "${GREEN}✓${NC} Opponent raceline has $OPP_LINES lines"
    ((SUCCESS++))
else
    echo -e "${RED}✗${NC} Opponent raceline has only $OPP_LINES lines (expected >10)"
    ((FAILURES++))
fi

# Check that main raceline has data
MAIN_LINES=$(wc -l < src/planning_pkg/data/raceline.csv)
if [ "$MAIN_LINES" -gt 10 ]; then
    echo -e "${GREEN}✓${NC} Main raceline has $MAIN_LINES lines"
    ((SUCCESS++))
else
    echo -e "${RED}✗${NC} Main raceline has only $MAIN_LINES lines (expected >10)"
    ((FAILURES++))
fi
echo ""

echo "=== Script Permissions ==="
if [ -x "src/planning_pkg/scripts/generate_opponent_raceline.py" ]; then
    echo -e "${GREEN}✓${NC} Generator script is executable"
    ((SUCCESS++))
else
    echo -e "${YELLOW}⚠${NC} Generator script is not executable (can still run with python3)"
    ((WARNINGS++))
fi
echo ""

echo "================================================"
echo "Summary:"
echo "  ${GREEN}Success: $SUCCESS${NC}"
if [ $WARNINGS -gt 0 ]; then
    echo "  ${YELLOW}Warnings: $WARNINGS${NC}"
fi
if [ $FAILURES -gt 0 ]; then
    echo "  ${RED}Failures: $FAILURES${NC}"
fi
echo "================================================"
echo ""

if [ $FAILURES -eq 0 ]; then
    echo -e "${GREEN}✓ All critical checks passed!${NC}"
    echo ""
    echo "Next steps:"
    echo "  1. Build the workspace: colcon build --packages-select planning_pkg"
    echo "  2. Launch the system: ros2 launch project_launch main.launch.py"
    echo "  3. Visualize in RViz:"
    echo "     - Add /global_raceline (GREEN)"
    echo "     - Add /opponent_raceline (ORANGE)"
    echo "     - Add /racing_agent/inside_overtake_lane (RED)"
    echo ""
    echo "For more info, see QUICK_REFERENCE_WIDENED.md"
    exit 0
else
    echo -e "${RED}✗ Some checks failed. Please review the errors above.${NC}"
    exit 1
fi
