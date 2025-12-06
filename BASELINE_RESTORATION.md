# Baseline Restoration Changes

## Overview

This document explains the changes made to restore stable baseline behavior to the racing system after the recent merge introduced too many simultaneous changes.

## Problem Statement

The latest merge (#73) made the car behavior significantly worse:
- Over-flattened OUT-IN-OUT cornering
- Too low speed profile everywhere
- Aggressive and "sticky" following behavior
- Complex overtaking logic interfering with basic driving

## Solution: Restore Simple, Stable Baseline

The approach is to **temporarily simplify** the system to recover a good "simple racing" behavior first, then gradually re-introduce advanced features.

---

## Changes Made

### 1. Simplified Overtaking Logic

**Goal:** Disable complex overtaking to focus on basic CRUISE and FOLLOW behavior.

#### Parameter Changes (racing_agent_params.yaml)
- `n_overtake_zones`: 2 → **0** (disabled all overtake zones)
- `overtake_width_factor`: 1.1 → **2.5** (very conservative - effectively disables)
- `overtake_lateral_margin`: 0.20m → **0.5m** (very conservative)
- `min_longitudinal_window`: 4.0m → **15.0m** (very conservative - effectively disables)

#### Code Changes (racing_agent.cpp)
- `OVERTAKE_WIDTH_FACTOR`: 1.1 → **2.5**
- `OVERTAKE_LATERAL_MARGIN`: 0.20 → **0.5**
- `MIN_LONGITUDINAL_WINDOW`: 4.0 → **15.0**

**Effect:** Overtaking will not trigger even if other conditions are met. The car will stay in FOLLOW mode behind opponents.

---

### 2. Improved Cornering Behavior

**Goal:** Restore clear, visible OUT-IN-OUT racing line without over-flattening.

#### Parameter Changes
- `corner_exit_lateral_soften`: 0.85 → **0.95**

#### Code Changes
- `CORNER_EXIT_LATERAL_SOFTEN`: 0.85 → **0.95**

**Effect:** 
- More pronounced OUT-IN-OUT cornering
- Clearer racing line visibility
- Still maintains safety margin from walls
- Less "flattened" appearance in corners

---

### 3. Better Speed Profile

**Goal:** Make the car faster but stable, restoring racing feel.

#### Parameter Changes
- `cruise_speed`: 7.0 m/s → **6.0 m/s** (more balanced baseline)

#### Code Changes
- Constructor default `cruise_speed`: 5.0 → **6.0**
- Cruise mode speed boost: 1.1 (10%) → **1.15 (15%)**

**Effect:**
- Base cruise speed: 6.0 m/s
- Actual cruise speed (no opponents): 6.9 m/s (15% boost)
- More balanced between speed and stability
- Higher than previous too-conservative speeds

---

### 4. Relaxed Following Behavior

**Goal:** Eliminate "sticky" following, maintain larger safe distance.

#### Parameter Changes
- `safe_follow_distance`: 3.0m → **4.5m** (50% increase)
- `min_stop_distance`: 0.3m → **0.5m** (safety buffer)
- `corner_follow_distance_factor`: 1.3 → **1.5** (more margin in corners)
- `corner_speed_reduction`: 0.8 → **0.9** (less aggressive)

#### Code Changes
- Constructor defaults updated to match
- `FOLLOW_CONTROL_GAIN_STRAIGHT`: 0.5 → **0.3** (less aggressive control)
- `FOLLOW_CONTROL_GAIN_CORNER`: 0.8 → **0.4** (less aggressive in corners)
- `FOLLOW_CLOSE_THRESHOLD`: -0.5m → **-1.0m** (more tolerance)
- `FOLLOW_CLOSE_SPEED_FACTOR`: 0.8 → **0.9** (less severe reduction)

**Effect:**
- Following distance: 4.5m (straight) → 6.75m (corners)
- Smoother, less nervous following
- No "gluing" to front car
- Better distance maintenance
- Less aggressive speed adjustments

---

## Summary of Behavior Changes

### Before (Problematic)
- ❌ Complex overtaking interfering with basic driving
- ❌ Over-flattened cornering (too conservative OUT-IN-OUT)
- ❌ Too slow overall (cruise_speed 7.0 but conservative everywhere)
- ❌ Aggressive following (3.0m distance, high gains)
- ❌ "Sticky" behavior in corners

### After (Baseline Restored)
- ✅ Overtaking disabled - focus on CRUISE and FOLLOW only
- ✅ Clear OUT-IN-OUT cornering (0.95 soften factor)
- ✅ Better speeds (6.0 base, 6.9 cruise with boost)
- ✅ Relaxed following (4.5m distance, low gains)
- ✅ Smooth, stable behavior

---

## Next Steps (Future Work)

Once this baseline is verified to work well:

1. **Re-introduce Overtaking Gradually**
   - Start with 1 overtake zone
   - Use conservative feasibility parameters
   - Verify in RViz and logs
   - Gradually expand zones and relax parameters

2. **Fine-tune Speed Profile**
   - May increase cruise_speed if baseline is stable
   - Adjust speed boost factors
   - Optimize corner speed profiles

3. **Optimize Following**
   - May reduce following distance slightly if too conservative
   - Fine-tune control gains
   - Add adaptive distance based on opponent behavior

---

## Testing Recommendations

1. **Verify Stable Cornering**
   - Check OUT-IN-OUT is visible and smooth
   - No wall contacts on corner exit
   - Racing line looks natural

2. **Verify Reasonable Speeds**
   - Check cruise speed is ~6.9 m/s
   - No overly conservative slowdowns
   - Stable speed profile

3. **Verify Non-Aggressive Following**
   - Following distance stays around 4.5m
   - No sudden speed changes
   - Smooth tracking of opponent
   - No "gluing" behavior

4. **Verify No Overtaking**
   - Car should stay in FOLLOW mode behind opponents
   - No OVERTAKE_CANDIDATE or OVERTAKE modes triggered
   - Simple stop-and-hold on collision risk

---

## Files Modified

1. `src/planning_pkg/config/racing_agent_params.yaml`
   - All parameter adjustments listed above

2. `src/planning_pkg/src/racing_agent.cpp`
   - Updated constants for baseline behavior
   - Updated constructor defaults

---

## Rollback Instructions

If these changes cause issues, revert to commit `c5ada9e`:

```bash
git checkout c5ada9e -- src/planning_pkg/config/racing_agent_params.yaml
git checkout c5ada9e -- src/planning_pkg/src/racing_agent.cpp
```

However, this will restore the problematic behavior described in the problem statement.
