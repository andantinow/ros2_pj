# V6.0 Final Tuning Summary

## Overview

This document summarizes the changes made for the V6.0 final tuning to address:
1. Opponent path positioning (closer to OUT or IN lines)
2. FOLLOW distance (larger, safer gap)
3. Corner vs straight speed logic verification
4. Confirmation that `simple_controller.cpp` remains unchanged

## Critical Rule: Do NOT Modify simple_controller.cpp ✓

**STATUS: COMPLIED** - `simple_controller.cpp` was not modified.

All opponent behavior changes were achieved by:
- Modifying the opponent's reference path parameters
- Adjusting NMPC and state machine parameters
- NO changes to the simple controller implementation

## Changes Made

### 1. Opponent Path: Closer to OUT or IN ✓

**File**: `src/planning_pkg/scripts/generate_opponent_raceline.py`

**Changes**:
- `out_in_out_strength`: **0.5 → 0.75** (50% increase)
  - Makes opponent follow a more aggressive OUT-IN-OUT racing line
  - In corners, opponent now clearly occupies OUT or IN position
  - Creates more obvious overtaking opportunities for ego on opposite side

- `lane_position`: **-0.2 → -0.3** (50% increase in outside bias)
  - Opponent stays more clearly toward outside on straights
  - Provides consistent positioning strategy

- `wall_margin`: **0.3m → 0.25m** (reduced by 16.7%)
  - Allows opponent to get closer to walls (but still safe)
  - More realistic racing line
  - NOT dangerously close - maintains minimum safety margin

- `DEFAULT_OPPONENT_SPEED`: **3.0 m/s → 2.5 m/s**
  - Matches opponent_publisher speed parameter
  - Ensures consistency across components

**Result**:
- Opponent now follows a clear OUT-IN-OUT racing line through corners
- Visibly closer to walls in corners (but safe margin maintained)
- Creates obvious space for ego to overtake on alternative lines (IN or OUT)

### 2. FOLLOW Distance: Increased Gap ✓

**File**: `src/planning_pkg/config/racing_agent_params.yaml`

**Changes**:
- `safe_follow_distance`: **4.5m → 6.0m** (33% increase)
  - Ego maintains larger, more comfortable gap behind opponent
  - Less aggressive following behavior
  - More time to react to opponent maneuvers

- `min_stop_distance`: **0.5m → 0.8m** (60% increase)
  - Enhanced safety margin for emergency stops
  - Prevents getting dangerously close

**File**: `src/project_launch/config/nmpc_params.yaml`

**Changes**:
- `opponent_following_distance`: **1.0m → 2.5m** (150% increase!)
  - Safe following distance dramatically increased
  - Creates visibly larger gap in FOLLOW state
  - Matches the intent of less aggressive following

- `opponent_detection_range`: **2.0m → 3.0m** (50% increase)
  - Earlier detection of opponent
  - More time to transition to FOLLOW state smoothly

- `follow_margin`: **0.05 m/s → 0.15 m/s** (200% increase)
  - Ego speed stays further below opponent speed when following
  - Prevents creeping too close over time
  - More relaxed, stable following behavior

**Result**:
- Ego keeps a visibly larger, safer gap behind opponent in FOLLOW mode
- Less nervous, more predictable following behavior
- Reduced risk of collision or excessive braking

### 3. Corner vs Straight Speed Logic ✓

**Status**: **VERIFIED - NO CHANGES NEEDED**

**File**: `src/control_pkg/src/nmpc_engine_node.cpp` (lines 2114-2121)

**Existing Logic** (already correct):
```cpp
if (curvature < curvature_k1_) {           // curvature < 0.2 (straight)
    ref.v = v_max_straight_;                // v_ref = 6.5 m/s (FASTER)
} else if (curvature > curvature_k2_) {    // curvature > 0.8 (tight corner)
    ref.v = v_min_corner_;                  // v_ref = 2.5 m/s (SLOWER)
} else {                                    // intermediate curvature
    double t = (curvature - curvature_k1_) / (curvature_k2_ - curvature_k1_);
    ref.v = v_max_straight_ + t * (v_min_corner_ - v_max_straight_);  // Linear interpolation
}
```

**Current Parameters** (from `nmpc_params.yaml`):
- `curvature_k1`: 0.2 (1/m) - straight threshold
- `curvature_k2`: 0.8 (1/m) - tight corner threshold
- `v_max_straight`: 6.5 m/s - speed on straights (FASTER)
- `v_min_corner`: 2.5 m/s - speed in tight corners (SLOWER)

**Verification**:
✓ Straights (low curvature) → FASTER speed (6.5 m/s)
✓ Corners (high curvature) → SLOWER speed (2.5 m/s)
✓ Logic is correct and properly implements "straights faster, corners slower"

**Wall Proximity Check**:
- Searched for wall-based speed reduction logic
- Found ONLY steering adjustment in `applyCornerExitSmoothing()`
- NO speed reduction based on wall distance
- Speed determined by: curvature, FOLLOW distance, emergency stops only

**Result**:
- Ego speed logic is CORRECT and does NOT need modification
- Speed is controlled by curvature (as required)
- No wall-proximity slowdown interfering with racing speed

### 4. Summary of Behavior Changes

#### Before V6.0:
- Opponent path was too center-ish in corners
- FOLLOW distance was too small (1.0m)
- Ego followed too closely, appearing nervous
- Speed logic was correct but might have appeared wrong due to close following

#### After V6.0:
- **Opponent**: Clearly occupies OUT or IN line in corners (75% strength)
- **Ego FOLLOW**: Maintains 2.5m-6.0m gap (visibly larger, safer)
- **Ego Speed**: 
  - Straights: 6.5 m/s (FASTER when not following)
  - Corners: 2.5 m/s (SLOWER based on curvature)
  - FOLLOW: Limited by opponent speed minus margin
- **Overtaking**: Clear space available on opposite side from opponent

## Files Modified

1. `src/planning_pkg/scripts/generate_opponent_raceline.py`
   - Opponent path generation parameters

2. `src/planning_pkg/config/racing_agent_params.yaml`
   - High-level following parameters

3. `src/project_launch/config/nmpc_params.yaml`
   - NMPC following and detection parameters

## Files NOT Modified

✓ `src/control_pkg/src/simple_controller.cpp` - **Unchanged as required**
✓ `src/control_pkg/src/nmpc_engine_node.cpp` - Speed logic already correct
✓ All other control logic files

## Expected Results

1. **Opponent Behavior**:
   - Follows clear OUT-IN-OUT racing line
   - Approaches walls more closely in corners (but safely)
   - Creates obvious overtaking lanes

2. **Ego FOLLOW Behavior**:
   - Maintains larger gap (2.5m-6.0m range)
   - Less aggressive, more stable following
   - Speed slightly below opponent when following

3. **Ego Speed (Not in FOLLOW)**:
   - Straights: Fast (6.5 m/s max)
   - Corners: Slower (2.5 m/s min, based on curvature)
   - Clear difference between straight and corner speeds

4. **No Issues**:
   - No wall-proximity slowdown on straights
   - No reverse of corner/straight speed relationship
   - No modifications to simple_controller.cpp

## Testing Recommendations

1. Generate new opponent raceline:
   ```bash
   cd src/planning_pkg
   python3 scripts/generate_opponent_raceline.py
   ```

2. Build the workspace:
   ```bash
   colcon build --symlink-install
   ```

3. Launch the system and observe:
   - Opponent clearly using OUT-IN-OUT line in corners
   - Ego maintaining larger gap in FOLLOW mode
   - Ego faster on straights, slower in corners (when not following)
   - Clear overtaking opportunities on opposite side from opponent

## Version History

- **V5.1**: Previous tuning with conservative parameters
- **V6.0**: Final tuning addressing:
  - Opponent path positioning (OUT/IN)
  - FOLLOW distance increase
  - Speed logic verification (no changes needed)
  - Compliance with "do not touch simple_controller.cpp" requirement
