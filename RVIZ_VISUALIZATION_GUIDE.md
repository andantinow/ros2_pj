# RViz Visualization Guide - Overtaking Paths and Footprints

## Quick Setup

Add these topics to your RViz configuration to see all the new visualizations:

### Topics to Add

1. **Overtake Paths** (MarkerArray)
   - Topic: `/overtake_paths`
   - Shows: Inside (RED), Outside (BLUE), and Active (YELLOW) paths
   
2. **Ego/Opponent Footprints** (Marker)
   - Topic: `/detected_opponent`
   - Shows: Vehicle boxes (GREEN=ego, RED=opponent)

3. **NMPC Trajectory** (Marker) - existing
   - Topic: `/nmpc_predicted_trajectory`
   - Shows: Predicted path (GREEN line)

4. **Reference Points** (Marker) - existing
   - Topic: `/nmpc_reference_points`
   - Shows: Target waypoints (BLUE spheres)

---

## Color Legend

### Paths
- **BRIGHT GREEN**: Overtake zones on main raceline
- **RED line**: Inside overtake path (tighter, apex-side, 0.56m offset)
- **BLUE line**: Outside overtake path (wider, faster, 0.88m offset)
- **YELLOW thick line**: Currently active overtake trajectory (when executing)

### Vehicles
- **GREEN box**: Ego car footprint (0.50m × 0.35m)
- **RED box**: Opponent car footprint (0.50m × 0.35m)

### NMPC (existing)
- **GREEN line**: NMPC predicted trajectory
- **BLUE spheres**: NMPC reference points

---

## What You'll See in Different Scenarios

### 1. No Opponent (CRUISE Mode)
- Main raceline with OUT-IN-OUT shape
- No opponent footprint
- Ego footprint following raceline
- Overtake paths visible but not active

### 2. Following Opponent (FOLLOW Mode)
- RED opponent box ahead
- GREEN ego box maintaining distance
- No overtake paths highlighted
- Smooth following behavior

### 3. In Overtake Zone (OVERTAKE_CANDIDATE Mode)
- BRIGHT GREEN segments showing overtake zone
- RED path (inside) visible if feasible
- BLUE path (outside) visible if feasible
- System evaluating which path to use

### 4. Executing Overtake (OVERTAKE Mode)
- **YELLOW thick line**: The active overtake path
  - If INSIDE selected: follows RED path geometry
  - If OUTSIDE selected: follows BLUE path geometry
- Ego box moving along selected path
- Speed boost visible in speed readings

### 5. Overtake Complete (Return to CRUISE)
- Ego ahead of opponent
- Smooth merge back to main raceline
- Overtake paths fade out

---

## Path Characteristics

### Inside Overtake Path (RED)
```
Offset: 0.56m from raceline (0.7 × 0.8m)
Use: When space exists between opponent and inside wall
Benefit: Shorter distance, better racing line
Risk: Less margin for error
Visualization: Thin RED line, semi-transparent
```

### Outside Overtake Path (BLUE)
```
Offset: 0.88m from raceline (1.1 × 0.8m)
Use: When inside is blocked but outside is clear
Benefit: More space, can carry higher speed
Risk: Longer distance around
Visualization: Thin BLUE line, semi-transparent
```

### Active Overtake Path (YELLOW)
```
Width: Thicker line (0.15m vs 0.06m)
Height: Elevated (z=0.25m for visibility)
Color: Bright yellow (1.0, 1.0, 0.0)
Duration: Active only during OVERTAKE mode
```

---

## Footprint Visualization Details

### Ego Car (GREEN)
```
Dimensions: 0.50m length × 0.35m width × 0.30m height
Color: (0.0, 1.0, 0.0) RGB, 60% opacity
Position: Current ego position from odometry
Orientation: Current ego heading
Update Rate: 5 Hz (every 0.2s)
```

### Opponent Car (RED)
```
Dimensions: 0.50m length × 0.35m width × 0.30m height
Color: (1.0, 0.0, 0.0) RGB, 60% opacity
Position: Opponent position from odometry or LiDAR
Orientation: Assumed straight (could be enhanced)
Update Rate: 2 Hz (every 0.5s)
```

---

## Debugging with Visualization

### Check 1: Are Footprints Correct Size?
- Ego and opponent boxes should be similar size
- Width: 0.35m (about 1 wheel spacing wide)
- Length: 0.50m (slightly longer than wide)
- If boxes look too small/large, check scale in RViz

### Check 2: Are Overtake Paths Visible?
- RED and BLUE paths should run parallel to main raceline
- Inside (RED) closer to raceline (0.56m offset)
- Outside (BLUE) farther from raceline (0.88m offset)
- If not visible, check `/overtake_paths` topic is published

### Check 3: Is Active Path Highlighted?
- During OVERTAKE mode, YELLOW path should appear
- YELLOW path should match either RED or BLUE geometry
- If INSIDE selected: YELLOW follows RED path
- If OUTSIDE selected: YELLOW follows BLUE path

### Check 4: Do Footprints Align with Reality?
- Ego box should align with actual car position
- Opponent box should align with detected opponent
- If misaligned, check odometry topics and coordinate frames

---

## Expected Log Messages

When overtaking, you should see logs like:

```
[INFO] OVERTAKE_CANDIDATE: distance=2.30m, zone=true, path=INSIDE
[INFO] Both inside and outside paths feasible, preferring INSIDE
[WARN] OVERTAKE COMMITTED! path=INSIDE (apex-side), distance=2.10m
[INFO] OVERTAKE INSIDE (apex-side): offset=0.56m, speed=3.20 (boost=0.25 above opponent)
[INFO] OVERTAKE COMPLETE (passed opponent for 2.5s)
```

Or if outside path is used:

```
[INFO] OVERTAKE_CANDIDATE: distance=2.30m, zone=true, path=OUTSIDE
[INFO] OUTSIDE overtake path feasible
[WARN] OVERTAKE COMMITTED! path=OUTSIDE (wider), distance=2.10m
[INFO] OVERTAKE OUTSIDE (wider): offset=0.88m, speed=3.20 (boost=0.25 above opponent)
```

---

## Troubleshooting

### Problem: No overtake paths visible
**Solution**: 
- Check `/overtake_paths` topic: `ros2 topic echo /overtake_paths`
- Verify global raceline received: Check logs for "RECEIVED GLOBAL RACELINE"
- Ensure overtaking enabled: `enable_overtaking` parameter should be true

### Problem: Footprints not showing
**Solution**:
- Check `/detected_opponent` topic: `ros2 topic echo /detected_opponent`
- Verify opponent detected: Check logs for "FOLLOW: distance=..."
- Ensure opponent odometry publishing: `ros2 topic echo /opp_racecar/odom`

### Problem: Active path (YELLOW) not appearing during overtake
**Solution**:
- Check driving mode in logs: Should say "OVERTAKE INSIDE" or "OVERTAKE OUTSIDE"
- Verify overtake committed: Look for "OVERTAKE COMMITTED!" message
- Check marker namespace: Should have "active_overtake_path" in MarkerArray

### Problem: Paths look offset incorrectly
**Solution**:
- Check `overtake_path_width` parameter (default: 0.8m)
- Inside should be at 0.7 × width = 0.56m
- Outside should be at 1.1 × width = 0.88m
- Verify coordinate frame is "map"

---

## Advanced Visualization Tips

### 1. Adjust Transparency
In RViz marker settings:
- Increase alpha for clearer paths (try 1.0 instead of 0.7-0.8)
- Decrease alpha for less obstruction (try 0.4-0.5)

### 2. Change Line Thickness
- Default: 0.06m for paths, 0.15m for active
- Increase for better visibility on large tracks
- Decrease for precision viewing on small tracks

### 3. Add Grid for Scale Reference
- Enable Grid display in RViz
- Set cell size to 0.5m or 1.0m
- Helps verify footprint and path dimensions

### 4. Use Top-Down View
- Set RViz camera to top-down (orthographic)
- Makes path selection clearer
- Easier to see lateral offsets

---

## RViz Configuration Snippet

Add this to your RViz config file (`.rviz`):

```yaml
- Class: rviz_default_plugins/MarkerArray
  Name: Overtake Paths
  Topic:
    Depth: 5
    Durability Policy: Transient Local
    History Policy: Keep Last
    Reliability Policy: Reliable
    Value: /overtake_paths
  Value: true

- Class: rviz_default_plugins/Marker
  Name: Vehicle Footprints
  Topic:
    Depth: 5
    Durability Policy: Volatile
    History Policy: Keep Last
    Reliability Policy: Reliable
    Value: /detected_opponent
  Value: true
```

---

## Summary

With all visualizations enabled, you should clearly see:
1. **Main raceline** with OUT-IN-OUT shape
2. **Inside/outside overtake options** (RED/BLUE)
3. **Active overtake execution** (YELLOW)
4. **Ego and opponent** as physical boxes
5. **Overtake zones** as green segments

This provides complete visibility into the overtaking decision and execution process!
