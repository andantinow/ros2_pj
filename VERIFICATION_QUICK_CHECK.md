# Quick Verification Checklist for Alignment Fix

## Before Building

- [x] Verify raceline.csv has map coordinates (x≈5.9, y≈-9.4)
- [x] Verify opponent_raceline.csv has map coordinates (x≈6.0, y≈-9.1)
- [x] Verify raceline_widened.csv has map coordinates (same as raceline.csv)

## Expected Console Output

When launching the system, you should see:

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

## RViz Visual Verification

Open RViz and verify:

1. **Map (black points)**: Visible at bottom of view
2. **Global raceline (green)**: Overlaid EXACTLY on the black track
3. **Inside overtake lane (magenta/red)**: Close to raceline, on the track
4. **Outside overtake lane (cyan/orange)**: Close to raceline, on the track
5. **Ego car (red car)**: On the track, following the raceline
6. **Opponent car (orange car)**: On the track

## Success Criteria

✅ All paths and cars are on the SAME track
✅ No paths floating in a different location
✅ Green raceline follows the black track geometry
✅ Overtaking lanes are lateral offsets from the main raceline
✅ Coordinates in logs match map frame (x≈5-6, y≈-9 to -10)

## If Paths Are Still Misaligned

1. Check the console logs for actual coordinate values
2. Verify the frame_id is "map" for all paths
3. Check if the correct raceline file is being loaded
4. Ensure colcon build was run after the changes
5. Restart the launch file

## Common Issues

- **Paths still in wrong place**: Check if you're loading a different raceline file via launch parameters
- **No paths visible**: Check RViz fixed frame is set to "map"
- **Partial alignment**: Check if there's a mix of old and new data files

## Contact

If alignment issues persist after this fix, the problem may be:
- Launch file loading wrong raceline file
- RViz configuration issue (wrong fixed frame)
- Build cache issue (run `./clean_build.sh` and rebuild)
