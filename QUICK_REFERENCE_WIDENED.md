# Quick Reference: Widened Raceline for Inside Overtakes

## What Changed?

The racing system now uses **two racelines** to create space for inside overtakes:

1. **Main Raceline** (`/global_raceline`): Widened, ego car's normal path
2. **Opponent Raceline** (`/opponent_raceline`): Outside-biased, opponent car's path

This creates a natural gap on the inside for clean overtaking maneuvers.

## Visualizing in RViz

Add these topics to see the complete picture:

```
Topic: /global_raceline
Type: nav_msgs/Path
Color: Green
Description: Main raceline (ego's normal path, widened)

Topic: /opponent_raceline
Type: nav_msgs/Path
Color: Orange
Description: Opponent's path (outside-biased)

Topic: /racing_agent/inside_overtake_lane
Type: nav_msgs/Path
Color: Red
Description: Ego's inside overtake path

Topic: /racing_agent/outside_overtake_lane
Type: nav_msgs/Path
Color: Orange
Description: Ego's outside overtake path (alternate)
```

## Expected Behavior

### Normal Racing (CRUISE/FOLLOW)
- **Ego**: Follows green main raceline
- **Opponent**: Follows orange opponent raceline (more outside)
- **Result**: Visible gap on inside of corners

### Overtaking (OVERTAKE)
- **Ego**: Switches to red inside overtake lane
- **Opponent**: Continues on orange outside line
- **Result**: Clean inside pass, showcasing OUT-IN-OUT geometry

## Regenerating Racelines

If you need to adjust the spacing or bias:

### Opponent Raceline (Orange Car)
```bash
cd src/planning_pkg
python3 scripts/generate_opponent_raceline.py \
    --lane-position -0.3 \
    --wall-margin 0.3 \
    --output data/opponent_raceline.csv
```

**Parameters**:
- `--lane-position`: -1.0 (full outside) to 1.0 (full inside)
  - `-0.3` = moderate outside bias (default for opponent)
  - `-0.5` = strong outside bias (more inside space)
  - `-0.1` = slight outside bias
- `--wall-margin`: Safety margin from walls (meters)
- `--ds`: Sample spacing (meters)

### Main Raceline (Ego's Normal Path)
```bash
cd src/planning_pkg
python3 scripts/generate_opponent_raceline.py \
    --lane-position -0.1 \
    --wall-margin 0.3 \
    --output data/raceline.csv
```

**Note**: Use less outside bias than opponent to maintain racing character.

## Tuning for Different Overtaking Styles

### More Space for Inside Overtakes
Increase opponent's outside bias:
```bash
python3 scripts/generate_opponent_raceline.py --lane-position -0.4 --output data/opponent_raceline.csv
```

### Tighter Racing (Less Inside Space)
Reduce opponent's outside bias:
```bash
python3 scripts/generate_opponent_raceline.py --lane-position -0.2 --output data/opponent_raceline.csv
```

### Centered Racing (Original Style)
Both cars on centerline:
```bash
# Main raceline
python3 scripts/generate_opponent_raceline.py --lane-position 0.0 --output data/raceline.csv
# Opponent raceline
python3 scripts/generate_opponent_raceline.py --lane-position 0.0 --output data/opponent_raceline.csv
```

## Launch System

The system is automatically configured in `main.launch.py`:
- **raceline_server**: Publishes `/global_raceline` (main)
- **opponent_raceline_server**: Publishes `/opponent_raceline` (opponent)
- **opponent_publisher**: Follows `/opponent_raceline`

No manual configuration needed for normal use.

## Troubleshooting

### Opponent not following outside line
- Check `/opponent_raceline` topic is being published
- Verify opponent_publisher subscribed to correct topic
- Look for "Opponent Publisher ready" message in logs

### Ego not using inside overtake
- Check overtake zone configuration
- Verify clearance conditions met
- Ensure opponent distance in range [1.0m, 6.0m]
- Check `/racing_agent/visualization` for mode indicator

### Paths look wrong in RViz
- Verify frame_id is "map"
- Check RViz fixed frame is "map"
- Try hiding/showing paths to refresh
- Restart raceline servers if needed

## Key Files

- **Generator Script**: `src/planning_pkg/scripts/generate_opponent_raceline.py`
- **Main Raceline**: `src/planning_pkg/data/raceline.csv`
- **Opponent Raceline**: `src/planning_pkg/data/opponent_raceline.csv`
- **Launch File**: `src/project_launch/launch/main.launch.py`
- **Full Documentation**: `WIDENED_RACELINE_IMPLEMENTATION.md`

## Quick Test

1. Launch the system:
   ```bash
   ros2 launch project_launch main.launch.py
   ```

2. Open RViz and add the paths (green, orange, red)

3. Watch the behavior:
   - Opponent should run outside line (orange)
   - Ego follows main line (green) normally
   - In overtake zones, ego dives inside (red)

4. Expected visual: Clear OUT-IN-OUT with opponent protecting outside, ego cutting inside

## Summary

✅ **Widened main raceline** - leaves space on inside
✅ **Opponent on outside line** - natural defensive position
✅ **Inside overtake path** - exploits the gap
✅ **OUT-IN-OUT showcase** - geometry visible during overtakes
✅ **Easy to regenerate** - simple Python script
✅ **Fully configured** - works out of the box

For detailed implementation info, see `WIDENED_RACELINE_IMPLEMENTATION.md`.
