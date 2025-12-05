# planning_pkg

C++ path planning package for F1TENTH autonomous racing.

## Features

- **Raceline Server**: Publishes pre-computed raceline paths with velocity references
- **Path Utilities**: Reusable library for path manipulation (smoothing, resampling, interpolation)
- **Simple Path Planner**: Basic path planner with configurable parameters
- **Raceline Generator**: CLI tool to generate raceline from centerline
- **Frenet Overtake Planner**: Sampling-based overtaking planner with Frenet coordinates and visualization

## Build

```bash
colcon build --packages-select planning_pkg
```

## Components

### 1. Raceline Server (`raceline_server`)

Publishes raceline path from CSV file with optional smoothing and resampling.

**Usage:**
```bash
ros2 run planning_pkg raceline_server \
  --ros-args \
  -p raceline_file:=data/raceline.csv \
  -p frame_id:=map \
  -p publish_vref:=true \
  -p enable_smoothing:=false \
  -p smoothing_window:=5 \
  -p resample_spacing:=0.0
```

**Parameters:**
- `raceline_file` (string): Path to raceline CSV file (relative to package share or absolute)
- `frame_id` (string): Frame ID for published path (default: "map")
- `publish_vref` (bool): Whether to publish velocity reference array (default: true)
- `enable_smoothing` (bool): Enable path smoothing (default: false)
- `smoothing_window` (int): Smoothing window size, must be odd >= 3 (default: 5)
- `resample_spacing` (double): Resample path with uniform spacing in meters, 0.0 = no resampling (default: 0.0)

**Topics:**
- `/global_raceline` (nav_msgs/Path): Published path with TransientLocal QoS
- `/global_vref` (std_msgs/Float32MultiArray): Velocity reference array (if enabled)

### 2. Simple Path Planner (`simple_path_planner`)

Basic path planner that can generate fixed or dynamic paths.

**Usage:**
```bash
ros2 run planning_pkg simple_path_planner \
  --ros-args \
  -p odom_topic:=/car_state/odom_GT \
  -p path_topic:=/planning/path \
  -p frame_id:=map \
  -p path_spacing:=0.1 \
  -p enable_smoothing:=true \
  -p smoothing_window:=5 \
  -p publish_fixed_path:=true \
  -p publish_rate:=1.0
```

**Parameters:**
- `odom_topic` (string): Odometry topic to subscribe (default: "/car_state/odom_GT")
- `path_topic` (string): Path topic to publish (default: "/planning/path")
- `frame_id` (string): Frame ID for published path (default: "map")
- `path_spacing` (double): Spacing between path points in meters (default: 0.1)
- `enable_smoothing` (bool): Enable path smoothing (default: true)
- `smoothing_window` (int): Smoothing window size (default: 5)
- `publish_fixed_path` (bool): Publish fixed test path (default: true)
- `publish_rate` (double): Path publishing rate in Hz (default: 1.0)

### 3. Generate Raceline (CLI)

Generate raceline from centerline CSV with velocity profile. Supports track boundary-aware positioning.

**Usage:**
```bash
# Generate raceline with track boundaries (recommended)
ros2 run planning_pkg generate_raceline \
  --centerline_csv tracks/centerline_with_bounds.csv \
  --out_csv data/raceline.csv \
  --lane_position 0.0 \
  --wall_margin 0.3 \
  --mu 1.0 \
  --v_max 5.0 \
  --ds 0.2

# Generate from simple centerline (legacy mode)
ros2 run planning_pkg generate_raceline \
  --centerline_csv tracks/centerline.csv \
  --out_csv data/raceline.csv \
  --wall_offset 0.0 \
  --mu 1.0 \
  --v_max 20.0 \
  --ds 0.5
```

**Parameters:**
- `--centerline_csv`: Input CSV file (supports both simple and boundary-aware formats)
- `--out_csv`: Output raceline CSV file
- `--lane_position`: Position between walls (-1.0=left/outer, 0.0=center, 1.0=right/inner)
- `--wall_margin`: Minimum distance from walls in meters (default: 0.3)
- `--ds`: Sample spacing in meters (default: 0.5)
- `--mu`: Friction coefficient (default: 1.0)
- `--v_max`: Maximum speed in m/s (default: 20.0)
- `--ax_max`: Maximum acceleration in m/s^2 (default: 4.0)
- `--ax_min`: Maximum deceleration in m/s^2 (default: -6.0)
- `--wall_offset`: [DEPRECATED] Use lane_position instead

**Input centerline CSV formats:**

1. **Simple format** (`x,y`):
   ```csv
   x,y
   1.0,2.0
   1.5,2.5
   ...
   ```

2. **Boundary-aware format** (`x,y,d_left,d_right,psi`):
   ```csv
   x,y,d_left,d_right,psi
   1.0,2.0,0.8,0.9,0.1
   1.5,2.5,0.7,0.8,0.15
   ...
   ```
   - `d_left`: Distance to left (outer) wall from centerline
   - `d_right`: Distance to right (inner) wall from centerline
   - `psi`: Heading angle (optional)

**Generating centerline with boundaries:**

Use the provided script to extract centerline with track boundaries from stack_master's global_waypoints.json:

```bash
python3 scripts/generate_centerline_with_bounds.py \
  --json /path/to/stack_master/maps/teras/global_waypoints.json \
  --out tracks/centerline_with_bounds.csv
```

**Output raceline CSV format:**
- Columns: `s,x,y,psi,kappa,v_ref`
  - `s`: Arc length along path
  - `x,y`: Position coordinates
  - `psi`: Heading angle (yaw)
  - `kappa`: Curvature (1/radius)
  - `v_ref`: Reference velocity

### 4. Path Utilities Library

Reusable functions for path manipulation:

- `find_closest_point()`: Find closest point on path to given position
- `interpolate_path()`: Interpolate point at given distance along path
- `smooth_path()`: Apply moving average smoothing
- `resample_path()`: Resample path with uniform spacing
- `calculate_curvature()`: Calculate path curvature at point
- `validate_path()`: Validate path for NaN/Inf values
- `path_length()`: Calculate total path length

## Launch File

```bash
ros2 launch planning_pkg planner_launch.py \
  raceline_file:=data/raceline.csv \
  frame_id:=map \
  enable_smoothing:=false
```

## Notes

- **QoS**: Raceline server uses `TransientLocal` + `Reliable` QoS for latched publishing
- **NMPC Codegen**: This package does NOT include acados codegen (Python-based). Use separate tooling for that.
- **Path Validation**: All paths are validated before publishing to prevent NaN/Inf values

## Dependencies

- `rclcpp`
- `nav_msgs`
- `geometry_msgs`
- `std_msgs`
- `tf2`
- `tf2_ros`
- `visualization_msgs`

---

## Frenet Overtake Planner (`frenet_overtake_planner`)

A sampling-based path planner that generates overtaking trajectories using Frenet coordinates (s, d).

### Architecture

The planner operates in the Frenet coordinate system where:
- `s`: Longitudinal distance along track centerline
- `d`: Lateral deviation from centerline (+left, -right)

This simplifies overtaking to a 1D problem: smoothly transitioning `d` from 0 (center) to `d_target` (overtake lane).

### Usage

```bash
ros2 run planning_pkg frenet_overtake_planner \
  --ros-args \
  -p max_road_width:=2.0 \
  -p d_road_w:=0.5 \
  -p planning_horizon:=3.0 \
  -p collision_radius:=0.5 \
  -p frame_id:=base_link
```

### Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `max_road_width` | double | 2.0 | Maximum lateral sampling range (±m) |
| `d_road_w` | double | 0.5 | Lateral sampling step (m) |
| `planning_horizon` | double | 3.0 | Planning time horizon (s) |
| `planning_dt` | double | 0.2 | Time step for trajectory (s) |
| `planning_speed` | double | 5.0 | Assumed forward speed (m/s) |
| `collision_radius` | double | 0.5 | Vehicle collision check radius (m) |
| `weight_centerline` | double | 0.5 | Cost weight for centerline deviation |
| `weight_obstacle` | double | 1.0 | Cost weight for obstacle proximity |
| `weight_smoothness` | double | 0.3 | Cost weight for lane change smoothness |
| `planning_rate` | double | 10.0 | Planning loop frequency (Hz) |
| `frame_id` | string | "base_link" | TF frame for visualization |

### Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/planner/candidate_paths` | MarkerArray | All candidate paths (color-coded) |
| `/planner/selected_path` | Path | Selected optimal path |
| `/planner/path_cost` | Float64 | Cost of selected path |

### Visualization Color Coding

In Rviz, candidate paths are color-coded for debugging:
- **Green (thick)**: Selected optimal path
- **Gray (thin)**: Valid candidate paths
- **Red (thin)**: Collision paths (rejected)
- **Orange cylinder**: Detected obstacle

### Cost Function

The path cost is computed as:
```
cost = w_center * |d_target| + w_obs * (1/min_dist) + w_smooth * |Δd|
```

Where:
- `w_center`: Penalty for deviation from centerline
- `w_obs`: Penalty for obstacle proximity (inverse distance)
- `w_smooth`: Penalty for rapid lane changes
