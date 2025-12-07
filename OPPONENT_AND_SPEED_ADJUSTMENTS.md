# Opponent Behavior and Ego Speed Logic Adjustments

## Summary
This document describes the changes made to address the following requirements:
1. Adjust opponent car behavior to stay farther from walls but not exactly centered
2. Reduce opponent speed for easier overtaking
3. Verify ego speed is NOT reduced by wall/obstacle proximity
4. Ensure ego runs faster on straights when no opponent is ahead

## Changes Made

### 1. Opponent Car Adjustments

**File:** `src/utilities/nodes/opponent_publisher_cpp/src/opponent_publisher.cpp`

**Changes:**
- **Speed Reduction:** Reduced from 3.0 m/s to 2.5 m/s
  - Line 21: `speed_ = declare_parameter<double>("speed", 2.5);`
  - Line 242: `double speed_{2.5};`
  - Makes opponent clearly slower than ego (which can reach 4-6.5 m/s)

- **Wall Margin Increase:** From 0.25m to 0.35m
  - Line 28: `wall_margin_ = declare_parameter<double>("wall_margin", 0.35);`
  - Line 249: `double wall_margin_{0.35};`
  - Keeps opponent farther from walls with a more visible margin

- **Lateral Offset:** From 0.0m to 0.15m
  - Line 29: `lateral_offset_ = declare_parameter<double>("lateral_offset", 0.15);`
  - Line 250: `double lateral_offset_{0.15};`
  - Creates an offset from the exact center of the track
  - Leaves room for ego to overtake on either side

**Result:**
- Opponent runs slower (2.5 m/s vs ego's 4-6.5 m/s)
- Opponent stays 0.35m from walls (increased safety margin)
- Opponent runs 0.15m offset from center (creates overtake lanes)

### 2. Ego Speed Logic Verification

**Files Analyzed:**
- `src/control_pkg/src/nmpc_engine_node.cpp`
- `src/control_pkg/src/simple_controller.cpp`

**Finding: NO CHANGES NEEDED**

The ego controller already implements the desired behavior:

#### A. Curvature-Based Speed (NMPC)

**Location:** `nmpc_engine_node.cpp`, lines 2114-2121

```cpp
if (curvature < curvature_k1_) {
  ref.v = v_max_straight_;  // 6.5 m/s on straights
} else if (curvature > curvature_k2_) {
  ref.v = v_min_corner_;    // 2.5 m/s in tight corners
} else {
  // Linear interpolation between straight and corner speeds
  double t = (curvature - curvature_k1_) / (curvature_k2_ - curvature_k1_);
  ref.v = v_max_straight_ + t * (v_min_corner_ - v_max_straight_);
}
```

**Key Points:**
- Speed is determined PURELY by track curvature
- `v_max_straight_ = 6.5 m/s` (fast on straights)
- `v_min_corner_ = 2.5 m/s` (slower in tight corners)
- `curvature_k1_ = 0.2` (threshold for straight)
- `curvature_k2_ = 0.8` (threshold for tight corner)

#### B. Opponent-Based Speed Adjustments (NMPC)

**Location:** `nmpc_engine_node.cpp`, lines 1134-1195

Speed is adjusted based on driving mode:

1. **CRUISE Mode:** Uses curvature-based reference speed (no reduction)
2. **FOLLOW Mode:** Limited to `opponent.speed - follow_margin_`
3. **OVERTAKE Mode:** Boosted to `opponent.speed + overtake_boost_`
4. **OBSTACLE_STOP Mode:** Hard stop at 0.0 m/s (collision avoidance)

**Key Points:**
- FOLLOW mode only activates when opponent is detected ahead
- No generic wall/obstacle proximity slowdown
- Speed reductions ONLY from: curvature (corners) or opponent (FOLLOW mode)

#### C. Simple Controller Speed Logic

**Location:** `simple_controller.cpp`, lines 1168-1185

```cpp
double speed_factor = 1.0 / (1.0 + 2.0 * std::abs(path_curvature));

if (is_corner && !is_overtaking_) {
  speed_factor *= corner_speed_factor_;  // 0.65
}

double base_adjusted_speed = target_speed_ * speed_factor;

if (is_overtaking_ || is_post_overtake_) {
  adjusted_speed = compute_overtake_speed(base_adjusted_speed);
} else {
  adjusted_speed = compute_opponent_following_speed(base_adjusted_speed);
}
```

**Key Points:**
- Uses path curvature for speed factor
- Corners reduce speed by `corner_speed_factor_ = 0.65`
- Opponent following logic applies when opponent detected
- NO wall/obstacle proximity speed reduction

### 3. Wall/Obstacle Proximity - What It DOES Affect

**The obstacle distances ARE used for:**

1. **Hard Collision Detection (A1 Zone):**
   - `a1_threshold_ = 0.15m` - triggers immediate STOP and reverse
   - This is a safety feature, not a gradual slowdown

2. **Overtaking Clearance Checks:**
   - Validates there's enough space on left/right to overtake safely
   - Does NOT reduce speed, only decides if overtaking is possible

3. **Corner Exit Steering Smoothing:**
   - Slightly reduces steering when too close to walls during corner exit
   - Affects STEERING only, not speed
   - Located in `nmpc_engine_node.cpp`, lines 1944-1959

**What obstacle distances DO NOT affect:**
- Base reference speed on straights
- Speed in corners (controlled by curvature only)
- Any gradual slowdown as walls approach
- Speed when following opponent (ACC logic handles this)

### 4. Behavior Analysis

#### On Straights (Low Curvature)

**When NO opponent ahead:**
- Curvature < 0.2 → Speed = 6.5 m/s (v_max_straight)
- Wall proximity has NO effect on speed
- Only hard collision (A1 zone < 0.15m) triggers emergency stop

**When opponent ahead (FOLLOW mode):**
- Speed limited to `opponent.speed - follow_margin_`
- ACC (Adaptive Cruise Control) maintains safe gap
- Target gap = 3.0m, uses proportional control
- Wall proximity still has NO effect on speed

#### In Corners (High Curvature)

**All scenarios:**
- Curvature > 0.8 → Speed = 2.5 m/s (v_min_corner)
- 0.2 < Curvature < 0.8 → Linear interpolation
- Speed determined by geometry, not obstacles
- Wall proximity does NOT cause additional slowdown

## Configuration Parameters

### Opponent (opponent_publisher.cpp)
```yaml
speed: 2.5           # m/s - reduced from 3.0
wall_margin: 0.35    # m - increased from 0.25
lateral_offset: 0.15 # m - new, creates overtake lanes
```

### Ego NMPC (nmpc_params.yaml)
```yaml
# Curvature-based speed control
curvature_k1: 0.2              # Straight threshold
curvature_k2: 0.8              # Corner threshold
v_max_straight: 6.5            # m/s - fast on straights
v_min_corner: 2.5              # m/s - slower in corners

# Collision avoidance (hard stop only)
a1_threshold: 0.15             # m - hard STOP only
a2_threshold: 0.25             # m - NOT USED (avoidance disabled)

# Opponent following
opponent_detection_range: 2.0  # m
opponent_following_distance: 1.0  # m
follow_margin: 0.05            # m/s
overtake_boost: 0.4            # m/s
```

### Ego Simple Controller (control_params.yaml)
```yaml
target_speed: 4.0              # m/s base speed
max_speed: 6.0                 # m/s maximum
corner_speed_factor: 0.65      # Corner speed reduction

# Following system
follow_distance_threshold: 4.0 # m - start following
follow_min_distance: 0.5       # m
target_follow_gap: 3.0         # m
```

## Testing Recommendations

1. **Opponent Behavior:**
   - Verify opponent runs at ~2.5 m/s (slower than before)
   - Check opponent maintains 0.35m margin from walls
   - Confirm opponent runs slightly offset from center (0.15m)
   - Ensure clear overtake lanes exist on either side

2. **Ego Straight-Line Speed:**
   - On straights with no opponent: should reach 6.5 m/s
   - Verify speed NOT reduced when passing near walls
   - Confirm only curvature affects speed on empty straights

3. **Ego Following Behavior:**
   - When behind opponent: should maintain safe gap (~3.0m)
   - Speed should match opponent (2.5 m/s) minus small margin
   - Verify wall proximity doesn't cause additional slowdown

4. **Ego Overtaking:**
   - Should attempt overtakes when lanes are clear
   - Verify speed boost during overtake (2.5 + 0.4 = 2.9 m/s+)
   - Check overtake validation uses clearance, not slowing down

5. **Corner Behavior:**
   - Both ego and opponent should slow in corners
   - Slowdown should be based on curvature, not wall distance
   - Verify ego can corner faster than opponent when appropriate

## Conclusion

The changes made focus on the opponent car behavior:
- Slower speed (2.5 m/s)
- Farther from walls (0.35m margin)
- Offset from center (0.15m)

The ego speed logic was verified to already implement the desired behavior:
- Fast on straights (6.5 m/s when clear)
- Slow in corners (2.5-6.5 m/s based on curvature)
- Follow opponent when detected (ACC logic)
- NO wall/obstacle proximity slowdown
- Only hard collision triggers emergency stop

These changes should make:
- Opponent easier to follow and overtake
- Ego behavior more predictable (fast on straights, slow in corners)
- Clear overtake opportunities on both sides of opponent
- Racing more dynamic and realistic
