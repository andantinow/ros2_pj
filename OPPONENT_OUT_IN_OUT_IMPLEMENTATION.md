# Opponent OUT-IN-OUT Path and LiDAR FSM Implementation

## Overview

This document describes the implementation of the requirements specified for:
1. Opponent using OUT-IN-OUT racing line style
2. Ego car staying on global raceline + overtaking lanes
3. Increased follow distance for safer behavior
4. LiDAR virtual box FSM for state transitions

## Changes Made

### 1. Opponent Raceline Generation (`generate_opponent_raceline.py`)

**Key Changes:**
- Added OUT-IN-OUT racing line logic based on track curvature
- Opponent now follows classic racing line:
  - **OUTSIDE** approach before corners (wider entry for better vision)
  - **INSIDE** at apex (cutting close to inner wall/apex)
  - **OUTSIDE** on corner exit (wide exit for acceleration)
- Maintains small margin from walls for safety
- Creates clear overtaking opportunities for ego car

**New Parameters:**
- `--out-in-out-strength`: Controls strength of OUT-IN-OUT bias (0.0-1.0, default 0.5)
- `--lane-position`: Changed default from -0.3 to -0.2 for slight outside bias on straights

**Implementation Details:**
```python
# Curvature-based corner detection
CURVATURE_THRESHOLD = 0.08  # Threshold to detect corners

# For each point, compute:
# 1. Curvature (smoothed to reduce noise)
# 2. Base offset from lane position
# 3. OUT-IN-OUT adjustment based on corner phase
# 4. Total offset = base + OUT-IN-OUT adjustment
```

**Benefits:**
- Opponent follows a natural, predictable racing line
- Creates clear space for ego to overtake on alternative lines (inside or outside)
- More realistic racing behavior

### 2. Ego Car Configuration (simple_controller.hpp/cpp)

**Key Changes:**
- Disabled OUT-IN-OUT for ego car: `enable_out_in_out_ = false`
- Added comment: "OUT-IN-OUT is now used by OPPONENT only"
- Ego now follows global raceline + overtaking lanes exclusively

**Rationale:**
- Ego uses strategic overtaking paths (inside/outside lanes)
- Opponent uses tactical racing line (OUT-IN-OUT)
- Clear separation of behaviors prevents conflicts

### 3. Follow Distance Increase

**Updated Parameters in `simple_controller.hpp`:**
```cpp
// Before → After
follow_distance_threshold_: 4.0m → 5.0m   // Detection distance increased
follow_min_distance_:       0.5m → 0.8m   // Safety buffer increased
target_follow_gap_:         3.0m → 3.5m   // Target gap increased
```

**Benefits:**
- Larger safety gap prevents getting too close to opponent
- More time to react to opponent maneuvers
- Less aggressive following behavior
- Reduces risk of collision on corner entry

### 4. LiDAR Virtual Box FSM

**Virtual Box Definitions:**

```cpp
// Box 1: Safety Box (very close) - triggers emergency stop/reverse
LIDAR_BOX_SAFETY_FRONT:  0.18m  // Matches A1 threshold
LIDAR_BOX_SAFETY_WIDTH:  0.7m
Angle range: ±30 degrees

// Box 2: Detection/Follow Box - triggers FOLLOW state
LIDAR_BOX_FOLLOW_FRONT:  5.0m   // Matches follow_distance_threshold
LIDAR_BOX_FOLLOW_WIDTH:  1.0m
Angle range: ±45 degrees

// Box 3: Overtake Assessment Box - checks if path is clear
LIDAR_BOX_OVERTAKE_FRONT: 4.0m
LIDAR_BOX_OVERTAKE_WIDTH: 2.0m
Angle range: ±60 degrees (wider for overtake check)
```

**New Functions:**

1. `check_lidar_box_occupancy(front_dist, lateral_width, angle_range)`
   - Checks if any LiDAR points fall within specified virtual box
   - Returns true if box is occupied, false if clear
   - Used for all box checks

2. `update_virtual_boxes()`
   - Updates all virtual box states
   - Sets FSM state flags: `opponent_in_safety_box_`, `opponent_in_follow_box_`, `overtake_path_clear_`
   - Logs FSM state transitions for debugging

**FSM State Transitions:**

```
SAFETY BOX OCCUPIED     → STOP/REVERSE (emergency)
  ↓ (clear)
FOLLOW BOX OCCUPIED     → FOLLOW (maintain distance)
  ↓ (opponent present)
OVERTAKE PATH CLEAR     → ASSESS_OVERTAKE
  ↓ (path validated)
                        → EXECUTE_OVERTAKE
  ↓ (complete)
                        → MERGE_BACK
```

**LiDAR Usage Philosophy:**
- LiDAR is used ONLY for opponent detection and FSM triggers
- NO wall repulsion or generic slowdown near walls
- Wall distance information comes from track data (centerline_with_bounds.csv)
- Clean separation: LiDAR = opponent detection, Track data = wall avoidance

## Testing Recommendations

1. **Generate New Opponent Raceline:**
   ```bash
   cd src/planning_pkg/scripts
   python3 generate_opponent_raceline.py \
     --centerline tracks/centerline_with_bounds.csv \
     --output data/opponent_raceline.csv \
     --out-in-out-strength 0.5 \
     --wall-margin 0.3
   ```

2. **Visualize Paths in RViz:**
   - Global raceline (ego) should be clean, optimized line
   - Opponent raceline should show OUT-IN-OUT through corners
   - Inside/outside overtaking lanes should be visible

3. **Test FSM Transitions:**
   - Monitor `/crsm_state` topic for state changes
   - Check console logs for "FSM: Safety box", "FSM: Follow box", etc.
   - Verify smooth transitions: CRUISE → FOLLOW → ASSESS_OVERTAKE → EXECUTE_OVERTAKE

4. **Verify Follow Behavior:**
   - Ego should maintain ~3.5m gap when following
   - Should not get closer than 0.8m
   - Should detect opponent at ~5.0m distance

## Integration Points

### With Racing Agent
- Racing agent publishes mode on `/racing_agent/mode`
- Simple controller respects mode (CRUISE, FOLLOW, OVERTAKE, etc.)
- LiDAR virtual boxes provide additional low-level safety

### With Opponent Publisher
- Opponent follows raceline from `opponent_raceline.csv`
- Uses `simple_controller.cpp` (opponent instance)
- OUT-IN-OUT behavior embedded in raceline itself

### With NMPC Controller
- NMPC receives reference path from racing agent
- Virtual boxes provide safety overrides
- Follow distance ensures smooth tracking

## Key Benefits Summary

1. **Realistic Racing Dynamics:**
   - Opponent uses proven OUT-IN-OUT racing technique
   - Ego uses strategic overtaking paths
   - Natural, predictable behaviors

2. **Improved Safety:**
   - Larger follow distance (3.5m vs 3.0m)
   - Better safety margins (0.8m vs 0.5m)
   - Earlier opponent detection (5.0m vs 4.0m)

3. **Clear FSM Structure:**
   - Simple virtual box triggers
   - Easy to understand state transitions
   - Debuggable with logging

4. **Focused LiDAR Usage:**
   - Opponent detection only
   - No wall repulsion confusion
   - Clean separation of concerns

## Files Modified

1. `src/planning_pkg/scripts/generate_opponent_raceline.py`
   - Added OUT-IN-OUT logic
   - Added curvature-based corner detection
   - Updated parameters and documentation

2. `src/control_pkg/src/simple_controller.hpp`
   - Disabled OUT-IN-OUT for ego
   - Increased follow distance parameters
   - Added virtual box definitions and states

3. `src/control_pkg/src/simple_controller.cpp`
   - Implemented `check_lidar_box_occupancy()`
   - Implemented `update_virtual_boxes()`
   - Added virtual box update to control loop

## Next Steps

1. Build and test the updated code
2. Generate new opponent raceline with OUT-IN-OUT style
3. Run simulation and verify FSM transitions
4. Tune OUT-IN-OUT strength parameter if needed
5. Adjust follow distances based on track characteristics
