# Opponent Raceline Configuration Guide

## Overview

This guide explains how to configure and use the new opponent raceline system for F1TENTH racing. The system allows the opponent to run on predictable OUT or IN lines at half the ego's speed, enabling proper overtaking behavior.

## Quick Start

### Launch with Outer Opponent Line (Default)

```bash
ros2 launch project_launch main.launch.py opponent_line_mode:=outer opponent_speed:=3.25
```

### Launch with Inner Opponent Line

```bash
ros2 launch project_launch main.launch.py opponent_line_mode:=inner opponent_speed:=3.25
```

## Opponent Raceline Files

The system uses pre-generated opponent racelines located in `src/planning_pkg/data/`:

- **opponent_outer.csv**: Opponent runs on the outer side of the track (0.9m offset)
- **opponent_inner.csv**: Opponent runs on the inner side of the track (-0.9m offset)

Both files have speed profiles at 50% of ego's maximum speed (3.25 m/s).

## Generating New Opponent Racelines

If you need to regenerate the opponent racelines with different parameters:

```bash
cd src/planning_pkg
python3 scripts/generate_opponent_racelines.py \
    --input data/raceline.csv \
    --outer-offset 0.9 \
    --inner-offset -0.9 \
    --speed-factor 0.5 \
    --output-dir data
```

### Script Parameters

- `--input`: Base raceline CSV file (default: data/raceline.csv)
- `--outer-offset`: Lateral offset for outer line in meters (default: 0.9)
- `--inner-offset`: Lateral offset for inner line in meters (default: -0.9)
- `--speed-factor`: Speed reduction factor (default: 0.5 for half speed)
- `--output-dir`: Output directory for generated files (default: data)

### How It Works

The script:
1. Reads the base raceline waypoints (x, y, psi, kappa)
2. For each waypoint, calculates the heading to the next waypoint
3. Computes the normal direction (perpendicular to heading)
4. Applies lateral offset in the normal direction
5. Computes speeds from curvature if base raceline has no speed data
6. Applies the speed reduction factor (default 0.5)
7. Saves opponent_outer.csv and opponent_inner.csv

## Launch Parameters

### opponent_line_mode

- **Type**: String
- **Default**: 'outer'
- **Options**: 'outer' | 'inner'
- **Description**: Determines which opponent raceline the opponent follows

### opponent_speed

- **Type**: Float
- **Default**: 3.25
- **Units**: m/s
- **Description**: Opponent target speed (should be ~50% of ego max speed)

## Expected Behavior

### With Outer Line (opponent_line_mode:=outer)

- Opponent runs on the outer side of the track
- Creates overtaking corridor on the inside
- Ego can overtake by taking the inside line
- Clear visual separation between ego and opponent paths

### With Inner Line (opponent_line_mode:=inner)

- Opponent runs on the inner side of the track
- Creates overtaking corridor on the outside
- Ego can overtake by taking the outside line
- Useful for testing different overtaking scenarios

## Speed Configuration

The opponent speed is set in two places:

1. **CSV file speed profile**: The generated opponent racelines have curvature-based speeds:
   - Straights: 3.25 m/s (50% of ego's 6.5 m/s)
   - Corners: Reduced based on curvature (minimum 1.25 m/s)

2. **Launch parameter**: The `opponent_speed` parameter sets the target speed for the opponent publisher

For consistent behavior, ensure both values are aligned (typically 3.25 m/s for half of ego's maximum speed).

## Ego Follow Behavior

When NMPC detects the opponent ahead, it:

1. **CRUISE → FOLLOW**: Enters FOLLOW mode when opponent within 2.5m
2. **Speed Control**: Maintains speed at `opponent_speed - 0.15 m/s`
3. **Following Distance**: Keeps 2.5m gap behind opponent
4. **Overtake Decision**: When in overtake zone with sufficient corridor clearance

## Overtaking Corridor Requirements

For ego to attempt overtaking, the following conditions must be met:

- **Lateral clearance**: ≥ 1.3m between wall and opponent (or opponent and wall)
  - Calculated as: opponent_width (0.35m) + safety_margin (0.20m) + path_width (0.75m)
- **Longitudinal window**: ≥ 3.0m remaining in overtake zone
- **Opponent ahead**: Detected within 3.0m range

## Troubleshooting

### Opponent Not Following the Correct Line

**Check**:
1. Verify the correct CSV file is being loaded:
   ```bash
   ros2 param get /opponent_raceline_server raceline_file
   ```
2. Ensure the launch argument is correct: `opponent_line_mode:=outer` or `inner`

### Opponent Running Too Fast/Slow

**Check**:
1. Verify the speed parameter:
   ```bash
   ros2 param get /opponent_publisher speed
   ```
2. Ensure it matches the intended speed (typically 3.25 m/s)

### Ego Not Overtaking

**Check**:
1. Verify overtaking is enabled:
   ```bash
   ros2 param get /nmpc_engine_node enable_overtaking
   ```
2. Check lateral clearance in the corridor
3. Ensure ego is in an overtake zone (default: full track)
4. Review NMPC logs for overtake decision messages

### Regenerating with Different Offsets

If the opponent is too close to walls or too far from walls:

1. Adjust `--outer-offset` and `--inner-offset` values
2. Regenerate the CSV files
3. Relaunch the system
4. Recommended range: 0.7m to 1.1m depending on track width

## Integration with Simulation

### F1Tenth Gym Bridge

The opponent publisher integrates with the F1Tenth Gym simulation:

1. Subscribes to `/opponent_raceline` for the path
2. Publishes `/opp_drive` for opponent control
3. Uses odometry from `/opp_racecar/odom`

### Visualization in RViz

- **Ego raceline**: Published on `/global_raceline` (blue)
- **Opponent raceline**: Published on `/opponent_raceline` (varies)
- **Detected opponent**: Red sphere marker on `/detected_opponent`
- **Overtake paths**: Magenta (left) and cyan (right) on `/overtake_paths`

## Advanced Configuration

### Custom Speed Profiles

To create opponent racelines with custom speed profiles:

1. Edit the base raceline CSV to include desired `v_ref` values
2. Run the generation script with appropriate `--speed-factor`
3. Or modify the script to implement custom speed logic

### Track-Specific Offsets

For tracks with varying widths:

1. Analyze track width at different sections
2. Adjust offsets to maintain consistent wall margins
3. Consider creating multiple opponent racelines for different scenarios

### Multiple Opponents

To support multiple opponents:

1. Generate additional raceline variations
2. Launch multiple opponent_publisher instances with different namespaces
3. Configure NMPC to handle multiple opponent detections (requires code modifications)

## References

- Main launch file: `src/project_launch/launch/main.launch.py`
- Generation script: `src/planning_pkg/scripts/generate_opponent_racelines.py`
- NMPC configuration: `src/project_launch/config/nmpc_params.yaml`
- Opponent publisher: `src/utilities/nodes/opponent_publisher_cpp/src/opponent_publisher.cpp`
