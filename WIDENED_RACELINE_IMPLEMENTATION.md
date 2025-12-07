# Enhanced OUT-IN-OUT with Widened Space for Inside Overtakes

## Overview

This implementation enhances the racing system to showcase OUT-IN-OUT racing lines during overtakes by:

1. **Widening the global raceline** to create room for inside overtakes
2. **Giving the opponent a more outside-biased line** to leave space on the inside
3. **Enabling clean, textbook inside overtakes** that highlight the OUT-IN-OUT geometry

## Key Changes

### 1. Opponent Raceline with Outside Bias

**New File**: `scripts/generate_opponent_raceline.py`
- Generates a raceline positioned toward the outside of the track (outer wall)
- Default `lane_position = -0.3` provides moderate outside bias
- Leaves ~0.3-0.5m additional space on the inside compared to centerline
- Configured with `wall_margin = 0.3m` for safety

**Generated File**: `data/opponent_raceline.csv`
- Full raceline path for opponent car (orange car)
- Published on `/opponent_raceline` topic
- 80 points, ~38m track length

### 2. Widened Main Raceline

**Updated File**: `data/raceline.csv` (replaced with widened version)
- Original backed up as `raceline_original.csv`
- New raceline uses `lane_position = -0.1` (slight outside bias)
- Provides proper OUT-IN-OUT shape while leaving space on inside
- Still optimal for racing but not so tight against inner wall

**Effect**:
- Main raceline shows clear OUT-IN-OUT geometry
- More usable space between raceline and inner wall
- Room for dedicated inside overtake path

### 3. Dual Raceline Server Setup

**Modified File**: `src/project_launch/launch/main.launch.py`

Added second raceline server instance:
```python
opponent_raceline_node = Node(
    package='planning_pkg',
    executable='raceline_server',
    name='opponent_raceline_server',
    parameters=[{
        'raceline_file': 'data/opponent_raceline.csv',
        'path_topic': '/opponent_raceline',
        ...
    }]
)
```

**Topics Published**:
- `/global_raceline` - Main raceline for ego car (widened, lane_position=-0.1)
- `/opponent_raceline` - Opponent raceline (outside bias, lane_position=-0.3)

### 4. Opponent Uses Outside Line

**Modified File**: `src/utilities/nodes/opponent_publisher_cpp/launch/opponent_publisher_launch.xml`

Added `path_topic` parameter:
```xml
<arg name="path_topic" default="/opponent_raceline"/>
<param name="path_topic" value="$(var path_topic)"/>
```

**Modified File**: `src/project_launch/launch/main.launch.py`

Opponent launch now uses opponent raceline:
```python
opponent_launch = IncludeLaunchDescription(
    ...,
    launch_arguments={
        'path_topic': '/opponent_raceline',
        ...
    }
)
```

## How It Works

### Geometry Layout

```
Track cross-section (looking at a corner):

Outer Wall
    |
    |  <-- Opponent raceline (lane_position=-0.3, ~30% toward outer)
    |
    |  <-- Main raceline (lane_position=-0.1, ~10% toward outer)
    |
    |  <-- Inside overtake path (offset ~0.56m left from main)
    |
Inner Wall (apex)
```

### Overtaking Flow

1. **Normal Racing (CRUISE/FOLLOW)**:
   - Ego follows main raceline (widened OUT-IN-OUT)
   - Opponent runs on more outside line
   - Gap visible on inside

2. **Overtake Preparation (OVERTAKE_CANDIDATE)**:
   - Racing agent detects overtake zone
   - Checks conditions (clearance, distance, etc.)
   - Selects inside overtake lane

3. **Overtake Execution (OVERTAKE)**:
   - Ego switches to inside overtake path
   - Dives toward inner part of corner
   - Uses tighter line (~0.56m offset from main)
   - Speed boost applied (1.25×)

4. **Visual Effect**:
   - Opponent stays on outside line (protecting outer)
   - Ego cuts inside cleanly
   - Clear OUT-IN-OUT geometry visible
   - Textbook inside pass

### Space Calculations

With the new setup:
- Opponent line offset from center: ~0.3-0.5m toward outside
- Main raceline offset: ~0.1-0.2m toward outside
- Inside overtake offset: ~0.56m toward inside (from main)
- **Total inside space**: Main raceline no longer hugs inner wall, creating ~0.5-0.8m usable space on inside

## Visualization in RViz

Expected display:
- **GREEN line**: Main global raceline (widened, ego's normal path)
- **ORANGE line** (new): Opponent raceline (outside-biased)
- **RED line**: Inside overtake lane (ego's overtake path)
- During overtake: Opponent on orange, ego cutting inside on red

## Configuration Parameters

### Opponent Raceline Generation

In `scripts/generate_opponent_raceline.py`:
```python
--lane-position -0.3    # Outside bias (negative = toward outer wall)
--wall-margin 0.3       # Safety margin from walls
--ds 0.5                # Sample spacing
```

### Main Raceline

Current: `lane_position = -0.1` (slight outside bias)

To adjust:
```bash
cd src/planning_pkg
python3 scripts/generate_opponent_raceline.py \
    --lane-position -0.1 \
    --output data/raceline.csv
```

### Racing Agent Overtake Offsets

In `src/planning_pkg/src/racing_agent.cpp`:
```cpp
INSIDE_OVERTAKE_FACTOR = 0.7;   // ~0.56m offset (tight inside line)
OUTSIDE_OVERTAKE_FACTOR = 1.1;  // ~0.88m offset (wide outside line)
```

## Benefits

### 1. Showcases OUT-IN-OUT
- Main raceline has clear OUT-IN-OUT shape
- Not hugging inner wall excessively
- Visible racing line geometry

### 2. Enables Clean Inside Overtakes
- Opponent protects outside line naturally
- Space available on inside for ego
- No awkward squeezing or wall conflicts

### 3. Realistic Racing Behavior
- Opponent runs defensive line (outside)
- Ego exploits inside gap opportunistically
- Mirrors real racing tactics

### 4. Safety Margins
- Wall margins enforced (0.3m minimum)
- Gradual offsets, not extreme
- Both cars have safe paths

## Testing Recommendations

1. **Visualize Paths**:
   - Open RViz
   - Add `/global_raceline` (GREEN)
   - Add `/opponent_raceline` (ORANGE - new!)
   - Add `/racing_agent/inside_overtake_lane` (RED)
   - Verify spacing and geometry

2. **Test Following**:
   - Ego should follow main raceline smoothly
   - Opponent runs on outside line
   - Gap visible on inside

3. **Test Overtaking**:
   - Enter overtake zone behind opponent
   - Verify ego switches to inside lane
   - Check speed boost applied
   - Confirm clean pass without collision

4. **Verify OUT-IN-OUT**:
   - Watch cornering behavior
   - Main line should show OUT-IN-OUT
   - Inside overtake should use IN part
   - No excessive wall hugging

## Tuning Guide

### More Outside Bias for Opponent
```bash
# Generate more aggressive outside line
python3 scripts/generate_opponent_raceline.py \
    --lane-position -0.4 \
    --output data/opponent_raceline.csv
```

### Less Outside Bias (Tighter Racing)
```bash
# Less space on inside, more competitive
python3 scripts/generate_opponent_raceline.py \
    --lane-position -0.2 \
    --output data/opponent_raceline.csv
```

### Adjust Main Raceline Width
```bash
# More centered (less outside bias)
python3 scripts/generate_opponent_raceline.py \
    --lane-position 0.0 \
    --output data/raceline.csv

# More outside (more inside space)
python3 scripts/generate_opponent_raceline.py \
    --lane-position -0.2 \
    --output data/raceline.csv
```

## Files Modified/Added

### New Files
- `src/planning_pkg/scripts/generate_opponent_raceline.py` - Raceline generation script
- `src/planning_pkg/data/opponent_raceline.csv` - Opponent's outside-biased raceline
- `src/planning_pkg/data/raceline_widened.csv` - Widened main raceline
- `src/planning_pkg/data/raceline_original.csv` - Backup of original raceline
- `WIDENED_RACELINE_IMPLEMENTATION.md` - This documentation

### Modified Files
- `src/planning_pkg/data/raceline.csv` - Replaced with widened version
- `src/project_launch/launch/main.launch.py` - Added opponent raceline server
- `src/utilities/nodes/opponent_publisher_cpp/launch/opponent_publisher_launch.xml` - Added path_topic parameter

### No Changes Required
- `src/planning_pkg/src/racing_agent.cpp` - Existing overtake logic works with new geometry
- Inside/outside overtake factors already appropriate

## Summary

The implementation successfully creates a system where:

1. ✅ **Global raceline is widened** - shifted toward outside to leave inside space
2. ✅ **Opponent runs outside line** - lane_position=-0.3 provides outside bias
3. ✅ **Inside overtakes enabled** - clear space for ego to dive inside
4. ✅ **OUT-IN-OUT showcased** - geometry visible during overtakes
5. ✅ **Minimal code changes** - leveraged existing infrastructure
6. ✅ **Configurable** - easy to tune via script parameters

The racing behavior now highlights clean, textbook inside overtakes using proper OUT-IN-OUT racing lines, with the opponent naturally protecting the outside while the ego exploits the inside opportunity.
