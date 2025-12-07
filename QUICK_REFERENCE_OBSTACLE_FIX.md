# Quick Reference - Obstacle Geometry & Overtaking Implementation

## ✅ All Requirements Met

This implementation fixes obstacle/opponent geometry understanding and implements proper NMPC-based avoidance and overtaking paths.

---

## What Was Fixed

### 1. Vehicle Geometry ✓
- **Ego car**: 0.50m × 0.35m (length × width)
- **Opponent**: 0.50m × 0.35m  
- **Safety margin**: 0.10m buffer
- **Visualization**: Semi-transparent boxes (GREEN=ego, RED=opponent)

### 2. Obstacle Detection ✓
- Opponent detected as moving obstacle
- Walls detected from LiDAR
- Track boundaries represented
- Used for path shaping, NOT speed limiting

### 3. NMPC Obstacle Awareness ✓
- Obstacle avoidance in cost function (weight=50.0)
- Wall clearance minimum: 0.20m
- Collision avoidance through path modification
- No generic "slow near walls" behavior

### 4. Overtake Paths ✓
- **Inside path** (RED): 0.56m offset, tighter, apex-side
- **Outside path** (BLUE): 0.88m offset, wider, faster
- Automatic feasibility checking
- Intelligent path selection (prefers inside)

### 5. OUT-IN-OUT Enhancement ✓
- Lateral tolerance: 0.25m → 0.28m (+12%)
- More racing freedom
- Wall margins enforced (min 0.20m)
- Balanced safety vs performance

---

## RViz Visualization

### Colors
- **BRIGHT GREEN**: Overtake zones
- **RED line**: Inside overtake path
- **BLUE line**: Outside overtake path
- **YELLOW line**: Active trajectory (when overtaking)
- **GREEN box**: Ego car footprint
- **RED box**: Opponent car footprint

### Topics
```
/overtake_paths          - Inside, outside, and active paths
/detected_opponent       - Ego and opponent footprints
/nmpc_predicted_trajectory - NMPC prediction
/nmpc_reference_points   - Reference waypoints
```

---

## Key Parameters

### Geometry
```yaml
vehicle_width: 0.35          # Ego width (m)
vehicle_length: 0.50         # Ego length (m)
vehicle_safety_margin: 0.10  # Safety buffer (m)
overtake_opponent_width: 0.35 # Opponent width (m)
```

### NMPC Obstacle Avoidance
```yaml
w_obstacle_avoidance: 50.0   # Obstacle cost weight
min_wall_clearance: 0.20     # Min distance to walls (m)
lateral_tolerance: 0.28      # Lateral freedom (m)
```

### Overtaking
```yaml
overtake_path_width: 0.8     # Base offset (m)
inside_factor: 0.7           # 0.56m offset
outside_factor: 1.1          # 0.88m offset
overtake_safety_margin: 0.25 # Extra margin (m)
```

---

## Expected Behavior

### Normal Driving (No Opponent)
- Follows raceline with OUT-IN-OUT
- Maintains 0.20m wall clearance
- No unnecessary slowdowns

### Following Opponent
- FOLLOW mode activated
- Maintains safe distance
- Evaluates overtake options

### In Overtake Zone
- Checks inside path feasibility
- Checks outside path feasibility
- Enters OVERTAKE_CANDIDATE if path exists

### Executing Overtake
- Commits to INSIDE or OUTSIDE path
- Applies lateral offset to reference
- Speed boost applied
- Returns to CRUISE after completion

---

## Log Messages to Look For

```
# Path evaluation
[INFO] OVERTAKE_CANDIDATE: distance=2.30m, zone=true, path=INSIDE
[INFO] Both inside and outside paths feasible, preferring INSIDE

# Overtake commit
[WARN] OVERTAKE COMMITTED! path=INSIDE (apex-side), distance=2.10m

# During execution
[INFO] OVERTAKE INSIDE (apex-side): offset=0.56m, speed=3.20

# Completion
[INFO] OVERTAKE COMPLETE (passed opponent for 2.5s)
```

---

## Files Changed

### Main Implementation
- `src/control_pkg/src/nmpc_engine_node.cpp`
  - 384 lines changed
  - Added geometry structs
  - Obstacle-aware solver
  - Path generation and selection
  - Enhanced visualization

### Documentation
- `OBSTACLE_GEOMETRY_FIX_SUMMARY.md` - Technical details
- `RVIZ_VISUALIZATION_GUIDE.md` - Visualization guide
- `QUICK_REFERENCE_OBSTACLE_FIX.md` - This file

---

## Build & Run

```bash
# Build
cd /path/to/ros2_pj
colcon build --packages-select control_pkg
source install/setup.bash

# Run
ros2 run control_pkg nmpc_engine_node

# View in RViz
# Add topics: /overtake_paths, /detected_opponent
```

---

## Troubleshooting

### No overtake paths visible
- Check `/overtake_paths` topic published
- Verify global raceline received
- Ensure `enable_overtaking=true`

### Footprints not showing
- Check `/detected_opponent` topic
- Verify opponent odometry: `/opp_racecar/odom`
- Confirm opponent detected in logs

### Paths seem incorrect
- Default `overtake_path_width=0.8m`
- Inside = 0.7 × 0.8 = 0.56m
- Outside = 1.1 × 0.8 = 0.88m

---

## Key Benefits

1. **Accurate Collision Detection**: Proper vehicle geometry prevents false positives/negatives
2. **Smart Path Selection**: Inside/outside options based on actual track geometry
3. **Safe Overtaking**: Feasibility checks ensure safe execution
4. **Clear Visualization**: See exactly what the system is doing
5. **No Speed Penalties**: Obstacles shape path, don't slow car arbitrarily
6. **Balanced Racing**: OUT-IN-OUT with safety margins

---

## Summary

✅ Fixed geometry understanding
✅ Reintroduced obstacle detection (no slowdown)
✅ NMPC obstacle-aware planning
✅ Inside/outside overtake paths
✅ Strengthened OUT-IN-OUT (+12%)
✅ Comprehensive visualization

The system now correctly understands vehicle dimensions, uses obstacles intelligently, and provides clear overtaking options with full visualization.

See `OBSTACLE_GEOMETRY_FIX_SUMMARY.md` for technical details.
See `RVIZ_VISUALIZATION_GUIDE.md` for visualization setup.
