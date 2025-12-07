# F1TENTH Racing Stack Improvements - Implementation Summary

## Date: 2025-12-07

All requirements from the problem statement have been successfully implemented.

## Quick Reference

### Launch Commands

**With Outer Opponent (Default)**:
```bash
ros2 launch project_launch main.launch.py controller:=nmpc opponent_line_mode:=outer opponent_speed:=3.25
```

**With Inner Opponent**:
```bash
ros2 launch project_launch main.launch.py controller:=nmpc opponent_line_mode:=inner opponent_speed:=3.25
```

### Regenerate Opponent Racelines
```bash
cd src/planning_pkg
python3 scripts/generate_opponent_racelines.py --input data/raceline.csv --outer-offset 0.9 --inner-offset -0.9 --speed-factor 0.5
```

## Implementation Complete ✅

### PHASE 1 – Opponent Behavior ✅
- ✅ Opponent runs on dedicated OUT (outer) or IN (inner) raceline
- ✅ Opponent speed: 3.25 m/s (50% of ego's 6.5 m/s)
- ✅ Lateral offset: ±0.9m provides clear side positioning
- ✅ Launch system supports dynamic line selection

### PHASE 2 – Ego Speed Control ✅
- ✅ Curvature-based speed: 6.5 m/s on straights, 2.5 m/s in corners
- ✅ NO wall-proximity slowdowns (A2 avoidance disabled)
- ✅ FOLLOW distance: 2.5m gap behind opponent
- ✅ FOLLOW speed: opponent_speed - 0.15 m/s

### PHASE 3 – Overtaking Logic ✅
- ✅ Corridor-based overtaking (not "empty box")
- ✅ Min clearance: 1.3m (allows 0.5-0.6m actual gap)
- ✅ Opponent detection via odometry (not walls)
- ✅ Relaxed LiDAR requirements

### PHASE 4 – FSM & Paths ✅
- ✅ State flow: CRUISE → FOLLOW → OVERTAKE_CANDIDATE → OVERTAKE
- ✅ Lateral offset overtaking paths (0.75m)
- ✅ Visualization: magenta (left), cyan (right)

## Files Created/Modified

### Created
1. `src/planning_pkg/scripts/generate_opponent_racelines.py`
2. `src/planning_pkg/data/opponent_outer.csv`
3. `src/planning_pkg/data/opponent_inner.csv`
4. `OPPONENT_RACELINE_GUIDE.md`
5. `F1TENTH_IMPROVEMENTS_SUMMARY.md`

### Modified
1. `src/project_launch/launch/main.launch.py`
2. `src/project_launch/config/nmpc_params.yaml`

### Unchanged (As Required)
- `src/control_pkg/src/simple_controller.cpp` - NOT modified

## Key Parameters

```yaml
# Opponent
opponent_line_mode: 'outer'              # or 'inner'
opponent_speed: 3.25                     # m/s

# Ego Speed
v_max_straight: 6.5                      # m/s
v_min_corner: 2.5                        # m/s
curvature_k1: 0.2                        # straight threshold
curvature_k2: 0.8                        # corner threshold

# FOLLOW
opponent_following_distance: 2.5         # m
follow_margin: 0.15                      # m/s

# Overtake Corridor
overtake_opponent_width: 0.35            # m
overtake_safety_margin: 0.20             # m
overtake_path_width: 0.75                # m
# Total min clearance: 1.30m
```

## Documentation

See `OPPONENT_RACELINE_GUIDE.md` for complete usage guide and troubleshooting.
