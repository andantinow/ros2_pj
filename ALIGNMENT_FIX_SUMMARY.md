# Global Path Alignment Fix - Summary

## Problem

After the recent merge (PR #77: "Widen raceline and add opponent outside bias"), the global paths were displayed in the wrong location in RViz:

- **Expected**: Global raceline and overtaking lanes should overlay on the black track (small_hall_orig map)
- **Actual**: Paths were floating far above the track in a different location
- **Root Cause**: The raceline files were in local track coordinates instead of map frame coordinates

## Analysis

### Coordinate System Comparison

**Before the fix:**
- `raceline.csv`: Local coordinates (e.g., x=2.116, y=-0.842)
- `opponent_raceline.csv`: Local coordinates (e.g., x=2.154, y=-0.571)
- `raceline_widened.csv`: Local coordinates (same as raceline.csv)

**After the fix:**
- `raceline.csv`: Map coordinates (e.g., x=5.938, y=-9.419) ✓
- `opponent_raceline.csv`: Map coordinates (e.g., x=6.009, y=-9.128) ✓
- `raceline_widened.csv`: Map coordinates (same as raceline.csv) ✓

### Map Frame Reference

From `small_hall_orig.yaml`:
```yaml
origin: [-0.268, -11.2, 0]
```

The map-aligned raceline coordinates (5.9, -9.4) are consistent with this origin.

## Solution

### Changes Made

1. **Restored map-aligned raceline coordinates**
   - Replaced `raceline.csv` with `raceline_original.csv` content
   - This file contains the original, correctly-aligned raceline in map frame
   - Coordinates: x ≈ 5.9, y ≈ -9.4

2. **Restored raceline_widened.csv**
   - Also replaced with `raceline_original.csv` content
   - The original raceline is already optimized for the track

3. **Regenerated opponent_raceline.csv**
   - Generated from `raceline_original.csv` with 0.3m lateral offset to the right
   - This creates space on the left (inside) for overtaking
   - Coordinates: x ≈ 6.0, y ≈ -9.1
   - Uses simple perpendicular offset based on heading angle

4. **Enhanced logging**
   - Added detailed logging to `racing_agent.cpp`:
     - Logs raceline receipt with frame_id and coordinates
     - Logs global overtaking lane generation with frame_id and coordinates
   - Existing logging in `raceline_server_node.cpp` already shows coordinates

### Frame Consistency

All paths now correctly use `frame_id = "map"`:
- `/global_raceline` published by `raceline_server` → frame_id: "map"
- Global inside overtaking lane created by `racing_agent` → frame_id: "map"
- Global outside overtaking lane created by `racing_agent` → frame_id: "map"

## Verification

### Expected Log Output

When the system starts, you should see:

```
[raceline_server]: === PUBLISHING RACELINE === Points: 273, Length: XX.XX m, Frame: map
[raceline_server]: First point: (5.938, -9.419), Last: (6.036, -9.436)

[racing_agent]: === RECEIVED GLOBAL RACELINE === Points: 273, Frame: map
[racing_agent]: First point: (5.938, -9.419), Last: (6.036, -9.436)

[racing_agent]: === GENERATED GLOBAL OVERTAKING LANES ===
[racing_agent]: Inside lane: 0.56m offset, 273 points, frame: map
[racing_agent]:   First point: (5.XXX, -9.XXX), Last: (X.XXX, -X.XXX)
[racing_agent]: Outside lane: 0.88m offset, 273 points, frame: map
[racing_agent]:   First point: (5.XXX, -9.XXX), Last: (X.XXX, -X.XXX)
```

### RViz Verification

In RViz, you should now see:
1. **Black points (map)**: The track walls at the bottom
2. **Green line (global raceline)**: Overlaid exactly on the track
3. **Magenta/Cyan lines (overtaking lanes)**: Close to the raceline, also on the track
4. **Ego car (red)**: On the track
5. **Opponent car (orange)**: On the track

**All elements should be co-located on the same physical track** - no floating paths!

## What About the Widening Feature?

The previous implementation attempted to widen the raceline by generating it from a `centerline_with_bounds.csv` file. However, that centerline was in **local track coordinates**, not map coordinates. This caused the alignment bug.

**Current approach:**
- Use the original, map-aligned raceline as-is
- The original raceline is already optimized for the track
- Opponent is offset to the right to create overtaking space
- Racing agent's OUT-IN-OUT logic still works correctly on this aligned raceline

**Future improvements** (if needed):
- If widening is still desired, generate `centerline_with_bounds.csv` in map coordinates
- Or apply a coordinate transformation when loading the centerline
- The key requirement: **all racelines must be in the map coordinate system**

## Files Modified

1. `src/planning_pkg/data/raceline.csv` - Restored to map coordinates
2. `src/planning_pkg/data/raceline_widened.csv` - Restored to map coordinates  
3. `src/planning_pkg/data/opponent_raceline.csv` - Regenerated with map coordinates
4. `src/planning_pkg/src/racing_agent.cpp` - Added coordinate logging

## Testing Recommendations

1. **Build the workspace**:
   ```bash
   cd /home/runner/work/ros2_pj/ros2_pj
   colcon build --symlink-install
   ```

2. **Launch the system**:
   ```bash
   source install/setup.bash
   ros2 launch project_launch main.launch.py
   ```

3. **Check logs** for coordinate values matching map frame

4. **Open RViz** and verify all paths overlay on the track

5. **Take a screenshot** showing the aligned paths

## Maintaining Coordinate System Integrity

To prevent this issue in the future:

1. **Always verify frame_id**: All global paths should use `frame_id = "map"`
2. **Check coordinates**: Map-aligned racelines should have coordinates near (5-6, -9 to -10)
3. **Verify in RViz**: Visual inspection is the final test
4. **Log coordinates**: The enhanced logging will catch misalignment issues early
5. **Document transforms**: If generating racelines from other sources, document any coordinate transformations needed

---

**Status**: ✅ Fix implemented and committed
**Next Steps**: Build, run, and verify in RViz
