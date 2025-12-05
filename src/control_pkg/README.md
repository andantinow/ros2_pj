# control_pkg

C++ vehicle control package for F1TENTH autonomous racing.

## Features

- **Simple Controller**: Basic Pure Pursuit path following controller
- **NMPC Engine Node**: Nonlinear Model Predictive Control engine
- **NMPC Lifecycle Node**: Production-ready NMPC with LifecycleNode pattern
- **Collision Recovery Node**: Deterministic Stop-Wait-Reverse FSM for collision recovery
- **Adaptive Cruise Control**: Time-headway based ACC with velocity estimation

## Build

```bash
colcon build --packages-select control_pkg
```

---

## Collision Recovery Node (`collision_recovery_node`)

A deterministic finite state machine (FSM) for collision recovery that implements a Stop-Wait-Reverse sequence.

### Overview

When a collision or stuck situation is detected, this node takes highest priority in the Ackermann MUX to perform a safe recovery sequence without relying on path planning (which may fail when localization is compromised).

### State Machine

```
IDLE → EMERGENCY_STOP → WAIT → BLIND_REVERSE → RECOVERY_COMPLETE → IDLE
```

| State | Duration | Description |
|-------|----------|-------------|
| IDLE | - | Normal monitoring (IMU jerk, LiDAR proximity) |
| EMERGENCY_STOP | 0.5s | Immediate motor cutoff |
| WAIT | 1.0s | Allow particle filter convergence, obstacles to clear |
| BLIND_REVERSE | 2.0s | Open-loop reverse (sensor-independent) |
| RECOVERY_COMPLETE | - | Return control to main planner |

### Usage

```bash
ros2 run control_pkg collision_recovery_node \
  --ros-args \
  -p jerk_threshold:=18.0 \
  -p min_proximity:=0.15 \
  -p wait_duration:=1.0 \
  -p reverse_duration:=2.0 \
  -p reverse_speed:=-1.5
```

### Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `jerk_threshold` | double | 18.0 | Collision detection threshold (m/s³) |
| `min_proximity` | double | 0.15 | LiDAR proximity trigger distance (m) |
| `wait_duration` | double | 1.0 | Wait time before reversing (s) |
| `reverse_duration` | double | 2.0 | Reverse duration (s) |
| `reverse_speed` | double | -1.5 | Reverse speed (m/s, negative) |
| `emergency_stop_duration` | double | 0.5 | Emergency stop hold time (s) |
| `scan_window` | int | 30 | LiDAR scan window (±indices) |
| `imu_dt` | double | 0.01 | IMU sample time for jerk calc (s) |

### Topics

**Subscribed:**
| Topic | Type | Description |
|-------|------|-------------|
| `/imu` | Imu | IMU data for collision detection |
| `/scan` | LaserScan | LiDAR for proximity detection |

**Published:**
| Topic | Type | Description |
|-------|------|-------------|
| `/drive_recovery` | AckermannDriveStamped | High-priority drive commands |
| `/recovery/active` | Bool | Recovery state indicator |

### Integration with Ackermann MUX

Configure the MUX to give `/drive_recovery` the highest priority:

```yaml
ackermann_mux:
  topics:
    - topic: /drive_recovery
      priority: 255  # Highest priority
    - topic: /drive_teleop
      priority: 100
    - topic: /drive
      priority: 10
```

---

## Adaptive Cruise Control Node (`adaptive_cruise_control_node`)

Time-headway based ACC that synchronizes with the leading vehicle's velocity.

### Overview

This node implements adaptive cruise control using:
1. **LiDAR clustering**: Detects lead vehicle in forward region
2. **Velocity estimation**: Calculates relative velocity via distance differentiation
3. **Time-headway control**: Maintains dynamic following distance based on ego speed

### Control Law

```
d_desired = d_min + τ_gap × v_ego
v_cmd = v_ego + Kp × (d - d_desired) + Kd × v_rel
```

Where:
- `d_desired`: Target following distance (increases with speed)
- `τ_gap`: Time headway constant
- `v_rel`: Estimated relative velocity (negative = closing)

### Usage

```bash
ros2 run control_pkg adaptive_cruise_control_node \
  --ros-args \
  -p min_safety_dist:=0.8 \
  -p time_headway:=0.4 \
  -p kp_dist:=1.2 \
  -p kd_vel:=0.1 \
  -p max_speed:=6.0
```

### Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `min_safety_dist` | double | 0.8 | Minimum safety distance (m) |
| `time_headway` | double | 0.4 | Time gap for speed-proportional distance (s) |
| `kp_dist` | double | 1.2 | Distance error P-gain |
| `kd_vel` | double | 0.1 | Relative velocity D-gain |
| `max_speed` | double | 6.0 | Maximum cruise speed (m/s) |
| `min_speed` | double | 0.0 | Minimum speed (m/s) |
| `scan_window_deg` | int | 20 | Forward scan window (±degrees) |
| `free_range_threshold` | double | 6.0 | Range beyond which is "free" (m) |
| `min_valid_range` | double | 0.1 | Minimum valid LiDAR range (m) |
| `enable_visualization` | bool | true | Enable Rviz markers |

### Topics

**Subscribed:**
| Topic | Type | Description |
|-------|------|-------------|
| `/scan` | LaserScan | LiDAR data |
| `/odom` | Odometry | Ego vehicle odometry |

**Published:**
| Topic | Type | Description |
|-------|------|-------------|
| `/drive/acc` | AckermannDriveStamped | ACC speed commands |
| `/acc/target_speed` | Float64 | Computed target speed |
| `/acc/target_marker` | Marker | Lead vehicle visualization |

### Time-Headway Policy

The time-headway policy ensures safer following at higher speeds:
- At 2 m/s: following distance = 0.8 + 0.4×2 = 1.6m
- At 6 m/s: following distance = 0.8 + 0.4×6 = 3.2m

This prevents the "accordion effect" where vehicles alternately brake and accelerate.

---

## Dependencies

- `rclcpp`
- `rclcpp_lifecycle`
- `lifecycle_msgs`
- `nav_msgs`
- `ackermann_msgs`
- `sensor_msgs`
- `std_msgs`
- `visualization_msgs`
- `tf2`, `tf2_ros`, `tf2_geometry_msgs`
- `eigen`
