# planning_pkg

C++ path planning package for F1TENTH autonomous racing.

## Features

- **Raceline Server**: Publishes pre-computed raceline paths with velocity references
- **Path Utilities**: Reusable library for path manipulation (smoothing, resampling, interpolation)
- **Simple Path Planner**: Basic path planner with configurable parameters
- **Raceline Generator**: CLI tool to generate raceline from centerline

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

Generate raceline from centerline CSV with velocity profile.

**Usage:**
```bash
ros2 run planning_pkg generate_raceline \
  --centerline_csv tracks/centerline.csv \
  --out_csv data/raceline.csv \
  --mu 1.0 \
  --ax_max 4.0 \
  --ax_min -6.0 \
  --v_max 20.0 \
  --ds 0.5
```

**Input centerline CSV format:**
- Header: `x,y` (optional)
- Rows: Closed-loop polyline coordinates in meters

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
