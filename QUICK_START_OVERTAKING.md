# Quick Start Guide: New Overtaking Features

## TL;DR

**What Changed:**
1. Ego car has slightly more aggressive OUT-IN-OUT cornering
2. Opponent can be positioned away from walls using parameters
3. Overtaking intelligently chooses inside/outside based on where opponent is
4. New visualization shows opponent position in real-time

## Quick Test

### 1. Build the Code
```bash
cd /path/to/ros2_pj
colcon build --packages-select planning_pkg utilities
source install/setup.bash
```

### 2. Launch the System
```bash
ros2 launch project_launch racing_stack.launch.py
```

### 3. Open RViz
Add these visualizations:
- Topic: `/racing_agent/visualization` (MarkerArray)
- Topic: `/racing_agent/inside_overtake_lane` (Path, RED)
- Topic: `/racing_agent/outside_overtake_lane` (Path, ORANGE)

### 4. Position the Opponent
To make opponent run to the LEFT of raceline:
```bash
ros2 param set /opponent_follower lateral_offset 0.3
```

To make opponent run to the RIGHT:
```bash
ros2 param set /opponent_follower lateral_offset -0.3
```

To center opponent:
```bash
ros2 param set /opponent_follower lateral_offset 0.0
```

### 5. Watch the Overtake
- Follow opponent into an OVERTAKE_ZONE
- Watch the **magenta sphere** showing opponent position
- Check the text label: "OPP LEFT", "OPP RIGHT", or "OPP CENTER"
- When overtaking:
  - If opponent LEFT → ego goes RIGHT (inside, red lane)
  - If opponent RIGHT → ego goes LEFT (outside, orange lane)

## Visual Guide

```
Track View (Top Down):
======================

Opponent LEFT of raceline:
┌─────────────────┐
│ WALL            │
│                 │  ← Outside lane (ORANGE)
│ ●────●────●     │  ← Opponent (running left)
│   ╲  ╱          │  ← Raceline (GREEN)
│    ●            │  ← Inside lane (RED) ★ EGO CHOOSES THIS
│                 │
│ WALL            │
└─────────────────┘

Opponent RIGHT of raceline:
┌─────────────────┐
│ WALL            │
│                 │  ← Outside lane (ORANGE) ★ EGO CHOOSES THIS
│     ●────●────● │  ← Raceline (GREEN)
│          ╲  ╱   │  
│           ●     │  ← Opponent (running right)
│                 │  ← Inside lane (RED)
│ WALL            │
└─────────────────┘
```

## Key Parameters

### Racing Agent
- `safe_follow_distance`: 5.2m (safe gap behind opponent)
- `corner_speed_reduction`: 0.85 (15% slower in corners)
- `cruise_speed`: 6.0 m/s (base speed)

### Opponent Publisher
- `lateral_offset`: 0.0 (shift left/right, in meters)
- `opponent_width`: 0.35m (vehicle width)
- `wall_margin`: 0.25m (target distance from walls)

## Troubleshooting

### "Opponent not detected"
- Check opponent is within 6m in front
- Verify LiDAR topic `/scan` is publishing
- Check opponent is in front sector (±30°)

### "No overtake happening"
- Verify in OVERTAKE_ZONE
- Check lateral clearance is sufficient
- Ensure opponent distance in range [1.0m, 6.0m]
- Watch logs for "No safe overtake path"

### "Opponent marker not visible"
- Add `/racing_agent/visualization` to RViz
- Set marker namespace filter to "opponent_indicator"
- Check ego has detected opponent (check FOLLOW mode)

### "Wrong overtake side chosen"
- Check opponent lateral offset with text label
- If centered, falls back to track geometry
- Increase lateral_offset to make position clearer

## Advanced Tuning

### Make Ego More Aggressive
Increase OUT-IN-OUT effect (already at 1.0, max recommended):
```cpp
// In racing_agent.cpp (already set)
CORNER_EXIT_LATERAL_SOFTEN = 1.0  // Range: 0.8-1.0
INSIDE_OVERTAKE_FACTOR = 0.65     // Range: 0.6-0.75
OUTSIDE_OVERTAKE_FACTOR = 1.15    // Range: 1.0-1.2
```

### Adjust Opponent Positioning
Make opponent run wider (more outside):
```bash
ros2 param set /opponent_follower lateral_offset 0.4
```

Make opponent run tighter (more inside):
```bash
ros2 param set /opponent_follower lateral_offset -0.3
```

### Change Overtake Sensitivity
Adjust threshold for detecting opponent position (in racing_agent.cpp):
```cpp
// Currently 0.15m - increase for less sensitivity, decrease for more
constexpr double LATERAL_THRESHOLD = 0.15;  // meters
```

## What to Expect

### Normal Cornering (No Opponent)
- Entry: Car moves OUT toward track edge
- Apex: Car cuts IN sharply
- Exit: Car drifts OUT with good margin from wall
- Effect: ~10-15% more pronounced than before

### Following Opponent
- Maintains 5.2m gap on straights
- Increases to 7.8m gap in corners
- No abrupt slowdowns
- Smooth, predictable behavior

### Overtaking
- Only attempts in OVERTAKE_ZONEs
- Chooses side based on opponent position
- Uses appropriate lane (inside/outside)
- Speed boost: 25% during pass
- Returns to main raceline after clearing opponent

## Debug Info

Enable debug logging:
```bash
ros2 run planning_pkg racing_agent --ros-args --log-level DEBUG
```

Watch for:
- "Opponent LEFT (d=X.XXm), overtaking RIGHT (INSIDE)"
- "Opponent RIGHT (d=X.XXm), overtaking LEFT (OUTSIDE)"
- "Overtake prep (medium): Evaluating INSIDE/OUTSIDE side"
- "Global overtake lane available: using INSIDE/OUTSIDE lane"

## Known Issues

1. **Opponent lateral detection best when directly ahead**
   - LiDAR-based estimation works in ±30° front sector
   - Accuracy decreases if opponent very far to side

2. **Static lateral offset**
   - Opponent's offset set at launch, not dynamic
   - Need to restart node to change (or use ros2 param set)

3. **Build requires ROS2 Humble**
   - Full testing needs proper ROS2 environment
   - Docker container recommended for consistent builds

## Quick Checklist

- [ ] Code built successfully
- [ ] Racing agent node running
- [ ] Opponent publisher running  
- [ ] RViz showing visualizations
- [ ] Opponent marker visible (magenta sphere)
- [ ] Opponent lateral offset configured
- [ ] Ego following opponent smoothly
- [ ] Overtake zones marked (green segments)
- [ ] Overtaking works in zones
- [ ] Correct side chosen based on opponent position

## Contact & Support

For issues or questions:
1. Check logs with `--log-level DEBUG`
2. Verify all visualizations in RViz
3. Test with different lateral_offset values
4. Review OVERTAKE_IMPROVEMENTS_SUMMARY.md for details
