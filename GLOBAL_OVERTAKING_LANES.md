# Global Overtaking Lanes Implementation

## Overview

This document describes the implementation of **global overtaking lanes** for the racing agent, addressing the requirements for balanced tuning and visualization.

## Key Changes Made

### 1. Global Overtaking Lanes Generation

**Location**: `src/planning_pkg/src/racing_agent.cpp` - `generate_global_overtaking_lanes()`

The system now generates **two global overtaking lane paths**:

- **Inside Overtaking Lane** (Red in RViz)
  - Lateral offset: ~0.56m (INSIDE_OVERTAKE_FACTOR × 0.8m = 0.7 × 0.8)
  - Used for tighter, apex-side overtaking maneuvers
  - Suitable for cutting inside on corners

- **Outside Overtaking Lane** (Orange in RViz)
  - Lateral offset: ~0.88m (OUTSIDE_OVERTAKE_FACTOR × 0.8m = 1.1 × 0.8)
  - Used for wider, outside overtaking maneuvers
  - Suitable for going around the outside

These lanes are:
- Generated once when the global raceline is received
- Published with latched QoS for persistent visualization
- Full continuous paths that run along the entire track

### 2. Visualization in RViz

**Topics Published**:
- `/racing_agent/inside_overtake_lane` - Inside lane (RED markers)
- `/racing_agent/outside_overtake_lane` - Outside lane (ORANGE markers)
- `/racing_agent/visualization` - Combined markers including:
  - Main raceline (GREEN)
  - Overtake zones
  - Currently active trajectory (highlighted)
  - Mode indicator text

**Colors**:
- Main raceline: GREEN
- Inside overtaking lane: RED
- Outside overtaking lane: ORANGE
- Active trajectory during overtake: Highlighted
- Overtake zones: GREEN segments on raceline

### 3. Overtaking Lane Selection Logic

**Function**: `should_use_global_overtake_lane()`

The system switches to global overtaking lanes when ALL conditions are met:

1. **Zone Requirement**: Must be in an OVERTAKE_ZONE
2. **Opponent Presence**: Must have a preceding vehicle
3. **Lateral Clearance**: At least one side (left or right) has sufficient clearance
4. **Longitudinal Window**: Sufficient distance remaining in overtake zone
5. **Distance Range**: Opponent distance in range [1.0m, 6.0m]

**Function**: `determine_best_overtake_side()`

Determines whether to use INSIDE or OUTSIDE lane based on:
- Track curvature (corner direction)
- Available clearance on each side
- Opponent lateral position

**Function**: `get_global_overtake_lane_segment()`

Extracts the relevant segment from the selected global lane for NMPC to follow.

### 4. Speed Profile Tuning (Small, Balanced Adjustments)

#### Straights - Faster
- **Cruise mode boost**: Increased from 15% to **20%** (cruise_speed × 1.20)
- **Overtake mode boost**: **25%** (cruise_speed × 1.25) for decisive passing

#### Corners - Slower
- **corner_speed_reduction**: Changed from 0.90 to **0.85**
  - This means 15% speed reduction in corners (down from 10%)
  - More conservative cornering for safety

### 5. Following Distance Tuning (Larger Safe Gap)

- **safe_follow_distance**: Increased from 4.5m to **5.2m** (+15%)
  - Provides more breathing room behind opponent
  - Reduces risk of collision on corner entry
  - Allows more time to react

- **corner_follow_distance_factor**: Remains at **1.5**
  - In corners, target distance becomes 5.2m × 1.5 = **7.8m**
  - Much safer margin when entering corners behind opponent

### 6. OUT-IN-OUT Balance Preserved

**No extreme changes** - existing parameters maintained:
- `corner_exit_lateral_soften_`: **0.95** (unchanged)
- `corner_exit_wall_margin_`: **0.35m** (unchanged)
- `INSIDE_OVERTAKE_FACTOR`: **0.7** (unchanged)
- `OUTSIDE_OVERTAKE_FACTOR`: **1.1** (unchanged)

All adjustments were kept within **10-20% range** to avoid extreme swings.

## How It Works

### Normal Operation Flow

1. **CRUISE Mode**:
   - Follow main raceline (GREEN path)
   - Speed: cruise_speed × 1.20 (faster on straights)

2. **FOLLOW Mode**:
   - Follow main raceline at safe distance (5.2m, or 7.8m in corners)
   - Speed: Adjusted based on gap error and corner detection
   - Corner speed: cruise_speed × 0.85 (slower in corners)

3. **OVERTAKE_CANDIDATE Mode**:
   - Evaluate if global overtake lane should be used
   - Check all safety conditions
   - Select inside or outside lane based on geometry

4. **OVERTAKE Mode**:
   - Switch reference path to selected global overtaking lane
   - Speed boost: cruise_speed × 1.25
   - Continue until opponent is passed (distance > 7.8m)
   - Then merge back to main raceline (CRUISE mode)

### Visualization Checking

To verify the implementation is working:

1. **In RViz**, you should see:
   - One **GREEN** line: main global raceline
   - One **RED** line: inside overtaking lane (slightly offset from main)
   - One **ORANGE** line: outside overtaking lane (slightly more offset)
   - **GREEN segments**: showing overtake zones
   - **Text indicator**: current mode (CRUISE, FOLLOW, OVERTAKE, etc.)

2. During overtaking:
   - The active path should switch from main raceline to one of the overtaking lanes
   - Speed should increase
   - After passing opponent, return to main raceline

## Configuration Parameters

All parameters can be adjusted in the launch file or via ROS2 parameters:

```yaml
racing_agent:
  ros__parameters:
    # Following distance (increased for safer gap)
    safe_follow_distance: 5.2  # Was 4.5
    
    # Corner handling (slower in corners)
    corner_speed_reduction: 0.85  # Was 0.9
    corner_follow_distance_factor: 1.5
    corner_curvature_threshold: 0.15
    
    # Cruise speed base value
    cruise_speed: 6.0
    
    # Overtake feasibility (conservative defaults)
    opponent_width: 0.35
    overtake_width_factor: 2.5
    overtake_lateral_margin: 0.5
    min_longitudinal_window: 15.0
```

## Summary of Changes

### Small, Balanced Adjustments (No Extremes)

| Parameter | Before | After | Change |
|-----------|--------|-------|--------|
| safe_follow_distance | 4.5m | 5.2m | +15% |
| corner_speed_reduction | 0.90 | 0.85 | -5% (slower) |
| cruise_mode_boost | 1.15× | 1.20× | +5% |
| overtake_mode_boost | 1.20× | 1.25× | +5% |

All changes kept within **10-20% range** as requested.

### New Features

1. ✅ Global inside overtaking lane generation and visualization (RED)
2. ✅ Global outside overtaking lane generation and visualization (ORANGE)
3. ✅ Main raceline visualization updated to GREEN
4. ✅ Smart lane selection based on track geometry and clearance
5. ✅ Automatic switching between main raceline and overtaking lanes
6. ✅ Persistent visualization topics for easier debugging

## Testing Recommendations

1. **Verify Visualization**:
   - Launch the racing_agent node
   - Open RViz and subscribe to `/racing_agent/visualization`
   - Subscribe to `/racing_agent/inside_overtake_lane` and `/racing_agent/outside_overtake_lane`
   - Confirm you see GREEN (main), RED (inside), and ORANGE (outside) paths

2. **Test Following Behavior**:
   - Place opponent vehicle ahead
   - Observe that ego maintains larger gap (5.2m on straights, 7.8m in corners)
   - Verify no aggressive closing in corners

3. **Test Overtaking**:
   - Place opponent in overtake zone
   - Verify ego switches to overtaking lane when conditions are met
   - Check speed boost during overtake
   - Confirm merge back to main raceline after passing

4. **Test Speed Profile**:
   - Observe higher speed on straights (cruise × 1.20)
   - Observe lower speed in corners (cruise × 0.85)
   - Confirm clear difference between straight and corner speeds

## Files Modified

1. `src/planning_pkg/include/planning_pkg/racing_agent.hpp`
   - Added global overtaking lane member variables
   - Added new method declarations

2. `src/planning_pkg/src/racing_agent.cpp`
   - Implemented global lane generation
   - Implemented lane selection logic
   - Updated visualization
   - Tuned parameters for balanced behavior

## Build Instructions

```bash
cd /path/to/ros2_pj
colcon build --packages-select planning_pkg
source install/setup.bash
ros2 run planning_pkg racing_agent
```

## Troubleshooting

### Overtaking lanes not visible
- Check that `/racing_agent/inside_overtake_lane` and `/racing_agent/outside_overtake_lane` topics are being published
- Verify global raceline has been received
- Check RViz marker settings (namespace: "global_overtaking_lanes")

### Ego not using overtaking lanes
- Verify all conditions in `should_use_global_overtake_lane()` are met
- Check that you're in an OVERTAKE_ZONE
- Ensure lateral clearance is sufficient
- Confirm opponent distance is in range [1.0m, 6.0m]

### OUT-IN-OUT line too aggressive or too conservative
- Existing OUT-IN-OUT parameters were preserved
- If further tuning needed, adjust `corner_exit_lateral_soften_` by small increments (±0.05)
- Avoid extreme values (keep in range 0.8 - 1.0)
