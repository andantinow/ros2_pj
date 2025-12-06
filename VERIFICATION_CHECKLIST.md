# Verification Checklist

This checklist should be used to verify that the baseline restoration changes achieve the desired behavior.

## 🎯 Primary Goals

- [ ] **Simple racing**: Car drives stably without opponent interference
- [ ] **Clear cornering**: OUT-IN-OUT is visible and natural
- [ ] **Good speeds**: Faster than before but stable
- [ ] **Relaxed following**: No "sticking" to opponent
- [ ] **Simple collision handling**: Stop and hold, no weird behavior

---

## ✅ Verification Steps

### 1. Build and Launch

```bash
cd /path/to/ros2_pj
colcon build --packages-select planning_pkg
source install/setup.bash
# Launch your system with the racing_agent node
```

### 2. Verify Cornering Behavior

**Expected:**
- Clear OUT-IN-OUT pattern visible
- Racing line looks natural, not over-flattened
- No wall contacts on corner exit
- Smooth steering transitions

**How to verify:**
- [ ] Watch in RViz: global raceline visualization
- [ ] Check steering commands: smooth, no jitter
- [ ] Observe wall distances: maintains safe margin

**Metrics:**
- `corner_exit_lateral_soften = 0.95` (very slight softening)
- Should see pronounced racing line

### 3. Verify Speed Profile

**Expected:**
- Cruise speed (no opponents): ~6.9 m/s (6.0 * 1.15)
- Following speed: varies based on opponent and distance
- Corner speeds: 90% of target (not 80%)

**How to verify:**
- [ ] Monitor `/racing_agent/mode` topic
- [ ] Check commanded speed in CRUISE mode
- [ ] Verify speed doesn't drop excessively in corners

**Metrics:**
- Base `cruise_speed = 6.0 m/s`
- Cruise boost = 15% → actual ~6.9 m/s
- Corner reduction = 90% (not 80%)

### 4. Verify Following Behavior

**Expected:**
- Following distance: ~4.5m on straights
- Following distance: ~6.75m in corners (4.5 * 1.5)
- Smooth speed adjustments (no jerky behavior)
- No "gluing" to opponent

**How to verify:**
- [ ] Place opponent car in front
- [ ] Observe following distance via LiDAR visualization
- [ ] Check speed adjustments are gradual
- [ ] Verify mode stays in FOLLOW (not switching to OVERTAKE)

**Metrics:**
- `safe_follow_distance = 4.5m`
- `corner_follow_distance_factor = 1.5`
- Control gains reduced (0.3 straight, 0.4 corner)

### 5. Verify Overtaking is Disabled

**Expected:**
- Mode never switches to OVERTAKE_CANDIDATE or OVERTAKE
- Always stays in FOLLOW when behind opponent
- Even in wide-open areas, no overtaking attempted

**How to verify:**
- [ ] Monitor `/racing_agent/mode` topic
- [ ] Check logs for any overtake messages
- [ ] Verify no overtake trajectory visualizations appear

**Metrics:**
- `n_overtake_zones = 0`
- Very conservative feasibility parameters

### 6. Verify Collision Handling

**Expected:**
- When obstacle < 0.5m: switch to OBSTACLE_STOP
- Vehicle decelerates smoothly to zero
- No erratic steering or "bouncing"
- Holds position until obstacle clears

**How to verify:**
- [ ] Place obstacle in path
- [ ] Observe smooth deceleration
- [ ] Verify mode switches to OBSTACLE_STOP
- [ ] Check steering remains stable

**Metrics:**
- `min_stop_distance = 0.5m`
- Simple stop-and-hold behavior

---

## 📊 Key Parameter Summary

### Speed Parameters
- `cruise_speed`: 6.0 m/s
- Effective cruise (CRUISE mode): 6.9 m/s (15% boost)

### Following Parameters
- `safe_follow_distance`: 4.5 m
- `min_stop_distance`: 0.5 m
- Effective follow distance in corners: 6.75 m (50% increase)

### Corner Parameters
- `corner_exit_lateral_soften`: 0.95 (very slight softening)
- `corner_curvature_threshold`: 0.15 (1/m)
- `corner_speed_reduction`: 0.9 (10% reduction, not 20%)

### Control Gains (Code Constants)
- `FOLLOW_CONTROL_GAIN_STRAIGHT`: 0.3 (reduced from 0.5)
- `FOLLOW_CONTROL_GAIN_CORNER`: 0.4 (reduced from 0.8)
- `FOLLOW_CLOSE_THRESHOLD`: -1.0 m (increased tolerance)
- `FOLLOW_CLOSE_SPEED_FACTOR`: 0.9 (less severe, from 0.8)

### Overtaking (Disabled)
- `n_overtake_zones`: 0
- `overtake_width_factor`: 2.5 (very conservative)
- `min_longitudinal_window`: 15.0 m (very conservative)

---

## 🔍 Debugging Tips

### If cornering is still too flat:
- Consider increasing `corner_exit_lateral_soften` further (0.97 or 0.99)
- Check if global raceline itself is over-smoothed

### If speeds are too low:
- Increase `cruise_speed` (try 6.5 or 7.0)
- Increase cruise boost factor (try 1.2 instead of 1.15)

### If following is too distant/slow:
- Decrease `safe_follow_distance` (try 4.0 or 3.5)
- Slightly increase control gains (try 0.4/0.5)

### If overtaking still triggers:
- Verify parameter file is being loaded
- Check logs for zone detection
- Ensure n_overtake_zones = 0

---

## ✨ Success Criteria

The baseline is considered **successfully restored** when:

1. ✅ Car completes full laps without crashes
2. ✅ OUT-IN-OUT cornering is clearly visible
3. ✅ Cruise speeds are ~6.5-7.0 m/s range
4. ✅ Following maintains ~4-5m distance smoothly
5. ✅ No overtaking attempts occur
6. ✅ Collision handling is simple stop-and-hold
7. ✅ Overall behavior feels "smooth and predictable"

Once these criteria are met, the system is ready for gradual re-introduction of overtaking features.

---

## 📝 Next Steps After Verification

If baseline is stable:

1. **Re-introduce Overtaking Gradually**
   ```yaml
   n_overtake_zones: 1  # Start with just one zone
   
   overtake_zone_0:
     name: "straight_1"
     s_start: 10.0
     s_end: 25.0
     type: "OVERTAKE_ZONE"
   ```

2. **Relax Feasibility Parameters Step-by-Step**
   ```yaml
   overtake_width_factor: 1.5  # Less conservative than 2.5
   min_longitudinal_window: 8.0  # Less than 15.0
   ```

3. **Monitor and Tune**
   - Watch RViz visualizations
   - Check logs for overtake decisions
   - Verify safety is maintained

4. **Expand Gradually**
   - Add second zone only after first is proven
   - Continue relaxing parameters slowly
   - Always verify in simulation first

---

## 🛠️ Troubleshooting Commands

```bash
# Check current mode
ros2 topic echo /racing_agent/mode

# Monitor reference path
ros2 topic echo /racing_agent/reference_path

# Check parameter values
ros2 param list /racing_agent
ros2 param get /racing_agent safe_follow_distance
ros2 param get /racing_agent cruise_speed

# View logs
ros2 run planning_pkg racing_agent --ros-args --log-level debug

# Visualize in RViz
# Look for markers in /racing_agent/visualization topic
```

---

## 📄 Related Documentation

- `BASELINE_RESTORATION.md` - Detailed change explanation
- `TUNING_GUIDE.md` - General tuning guidelines
- `src/planning_pkg/config/racing_agent_params.yaml` - Current parameters
