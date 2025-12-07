# V6.0 Quick Start Guide

## What Changed?

This update fixes three critical issues:

1. **Opponent Path** - Now uses a more aggressive OUT-IN-OUT racing line (closer to walls)
2. **FOLLOW Distance** - Ego maintains a larger, safer gap (2.5m-6.0m) behind opponent
3. **Speed Logic** - Verified correct (straights: fast, corners: slow based on curvature)

## Critical: Regenerate Opponent Raceline

**IMPORTANT**: After pulling these changes, you MUST regenerate the opponent raceline file with the new parameters.

### Step 1: Navigate to Planning Package
```bash
cd src/planning_pkg
```

### Step 2: Generate New Opponent Raceline
```bash
python3 scripts/generate_opponent_raceline.py
```

This will create a new `data/opponent_raceline.csv` with:
- OUT-IN-OUT strength: 0.75 (was 0.5)
- Lane position: -0.3 (was -0.2)
- Wall margin: 0.25m (was 0.3m)
- Speed: 2.5 m/s

### Step 3: Build the Workspace
```bash
cd ../..  # Return to workspace root
colcon build --symlink-install
```

### Step 4: Source and Launch
```bash
source install/setup.bash
# Launch your stack (e.g., ros2 launch project_launch main.launch.py)
```

## Expected Behavior

### Opponent
- Follows clear OUT-IN-OUT racing line through corners
- Approaches walls more closely (but safely) in corners
- Creates obvious overtaking lanes on opposite side

### Ego (FOLLOW Mode)
- Maintains **2.5m-6.0m gap** behind opponent (much larger than before)
- Less aggressive, more stable following
- Speed stays below opponent by 0.15 m/s margin

### Ego (CRUISE Mode - Not Following)
- **Straights**: Fast (up to 6.5 m/s)
- **Corners**: Slower (down to 2.5 m/s based on curvature)
- Clear speed difference between straights and corners

## Verification Checklist

After launching, verify:

- [ ] Opponent clearly uses OUT-IN-OUT line in corners
- [ ] Opponent appears closer to walls than before
- [ ] Ego maintains larger gap in FOLLOW mode (not too close)
- [ ] Ego is faster on straights when not following
- [ ] Ego is slower in corners
- [ ] Clear overtaking opportunities on opposite side from opponent

## Parameters Changed

### nmpc_params.yaml
- `opponent_following_distance`: 1.0m → 2.5m
- `opponent_detection_range`: 2.0m → 3.0m
- `follow_margin`: 0.05 m/s → 0.15 m/s

### racing_agent_params.yaml
- `safe_follow_distance`: 4.5m → 6.0m
- `min_stop_distance`: 0.5m → 0.8m

### generate_opponent_raceline.py (defaults)
- `out_in_out_strength`: 0.5 → 0.75
- `lane_position`: -0.2 → -0.3
- `wall_margin`: 0.3m → 0.25m
- `DEFAULT_OPPONENT_SPEED`: 3.0 m/s → 2.5 m/s

## Troubleshooting

### Issue: Opponent still too centered
**Solution**: Regenerate opponent raceline (Step 2 above) - the script parameters changed

### Issue: Ego still too close to opponent
**Solution**: Check that nmpc_params.yaml has `opponent_following_distance: 2.5`

### Issue: Ego slow on straights
**Solution**: This should be fixed - speed logic already correct. If still slow, check:
- Ego might be in FOLLOW mode (check distance to opponent)
- Check v_max_straight parameter (should be 6.5 m/s)

### Issue: Build errors
**Solution**: Try clean build:
```bash
./clean_build.sh
colcon build --symlink-install
```

## Files Modified

- `src/planning_pkg/scripts/generate_opponent_raceline.py` - Opponent path parameters
- `src/planning_pkg/config/racing_agent_params.yaml` - High-level following
- `src/project_launch/config/nmpc_params.yaml` - NMPC following parameters
- `V6_FINAL_TUNING_SUMMARY.md` - Detailed documentation

## Files NOT Modified

✓ `src/control_pkg/src/simple_controller.cpp` - Unchanged as required
✓ All other control logic files

## Support

For detailed technical information, see:
- `V6_FINAL_TUNING_SUMMARY.md` - Complete change documentation
- Parameter comments in the YAML files

## Version

- **Previous**: V5.1
- **Current**: V6.0 Final Tuning
- **Date**: 2025-12-07
