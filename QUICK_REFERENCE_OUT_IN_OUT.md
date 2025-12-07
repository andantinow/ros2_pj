# Quick Reference: OUT-IN-OUT Opponent Path Implementation

## What Changed?

### Opponent Behavior
- **Before**: Simple outside-biased path
- **After**: Classic OUT-IN-OUT racing line through corners
  - Wide entry (OUTSIDE)
  - Tight apex (INSIDE)
  - Wide exit (OUTSIDE)

### Ego Behavior
- **Before**: Used OUT-IN-OUT like opponent
- **After**: Follows global raceline + overtaking lanes only
  - OUT-IN-OUT disabled (`enable_out_in_out_ = false`)
  - Uses strategic paths from racing agent

### Follow Distance
- Detection: 4.0m → **5.0m** (detect opponent sooner)
- Target gap: 3.0m → **3.5m** (maintain larger gap)
- Minimum: 0.5m → **0.8m** (safer buffer)

### LiDAR Usage
- **Before**: Used for wall repulsion and generic slowdown
- **After**: Virtual box FSM for opponent detection only
  - 3 boxes: Safety, Follow, Overtake
  - Clear state transitions with logging
  - No wall repulsion (uses track data instead)

## Quick Commands

### Generate Opponent Path
```bash
cd src/planning_pkg/scripts
python3 generate_opponent_raceline.py \
  --centerline tracks/centerline_with_bounds.csv \
  --output data/opponent_raceline.csv \
  --out-in-out-strength 0.5 \
  --wall-margin 0.3
```

### Verify Changes
```bash
./verify_out_in_out_changes.sh
```

### Build
```bash
colcon build --packages-select control_pkg planning_pkg
```

## Tuning Parameters

### Opponent Path (`generate_opponent_raceline.py`)
- `--out-in-out-strength`: How aggressive the racing line is (0.0-1.0)
  - 0.3 = Conservative (small inside cut)
  - 0.5 = Balanced (default)
  - 0.7 = Aggressive (large inside cut)
- `--wall-margin`: Safety margin from walls (0.2-0.5m)
- `--lane-position`: Base position on straights (-0.3 to 0.0)

### Follow Distance (`simple_controller.hpp`)
```cpp
follow_distance_threshold_ = 5.0;  // When to start following
target_follow_gap_ = 3.5;          // Desired gap
follow_min_distance_ = 0.8;        // Emergency minimum
```

### Virtual Boxes (`simple_controller.hpp`)
```cpp
LIDAR_BOX_SAFETY_FRONT = 0.18;     // Emergency box
LIDAR_BOX_FOLLOW_FRONT = 5.0;      // Follow box
LIDAR_BOX_OVERTAKE_FRONT = 4.0;    // Overtake assessment
```

## Monitoring FSM States

Watch console for these logs:
```
FSM: Safety box OCCUPIED (STOP/REVERSE)
FSM: Follow box OCCUPIED (FOLLOW)
FSM: Overtake path CLEAR (ASSESS_OVERTAKE)
```

## Common Issues & Solutions

### Issue: Opponent crashes into walls
**Solution**: Increase `--wall-margin` to 0.4 or 0.5

### Issue: Opponent path too conservative
**Solution**: Increase `--out-in-out-strength` to 0.6 or 0.7

### Issue: Ego too close to opponent
**Solution**: Increase `target_follow_gap_` to 4.0m

### Issue: FSM transitions too sensitive
**Solution**: Adjust virtual box sizes in `simple_controller.hpp`

### Issue: No overtaking happening
**Solution**: 
1. Check `overtake_path_clear_` in logs
2. Verify wall data is loaded
3. Check `min_overtake_clearance_` parameter

## Visualization

### In RViz
- Global raceline (GREEN) - Ego follows this
- Opponent raceline (should show OUT-IN-OUT curves)
- Inside/outside overtaking lanes (RED/ORANGE)
- Safety zones (when following)

### Expected Behavior
1. Opponent takes wide line into corner (OUTSIDE)
2. Opponent cuts to apex (INSIDE)
3. Opponent exits wide (OUTSIDE)
4. Ego follows at 3.5m gap or overtakes on alternative line

## Performance Metrics

Track these in your simulation:
- Average follow gap (should be ~3.5m ± 0.5m)
- Overtake success rate
- Collision rate (should be near zero)
- Lap time difference (ego should be competitive)

## Documentation

- Full details: `OPPONENT_OUT_IN_OUT_IMPLEMENTATION.md`
- Code review history: Git commit messages
- Verification: `verify_out_in_out_changes.sh`

## Support

If something doesn't work as expected:
1. Run verification script
2. Check console logs for FSM states
3. Verify opponent path was regenerated
4. Check parameter values in launch files
5. Review `OPPONENT_OUT_IN_OUT_IMPLEMENTATION.md`
