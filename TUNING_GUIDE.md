# 튜닝 가이드 (Tuning Guide)

이 문서는 레포지토리 구조가 복잡하더라도 튜닝을 쉽게 할 수 있도록 핵심 포인트만 정리한 가이드입니다.

---

## 📋 목차

1. [빌드 및 실행 기본 원칙](#빌드-및-실행-기본-원칙)
2. [Racing Agent 아키텍처 (v3.0)](#racing-agent-아키텍처-v30)
3. [글로벌 레이스라인 튜닝](#글로벌-레이스라인-튜닝)
4. [센서 설정 튜닝](#센서-설정-튜닝)
5. [컨트롤러 파라미터 튜닝](#컨트롤러-파라미터-튜닝)
6. [NMPC 아키텍처 (v2.0)](#nmpc-아키텍처-v20)
7. [트러블슈팅](#트러블슈팅)

---

## 빌드 및 실행 기본 원칙

### ⚠️ 중요: 항상 `ros2 run` 기준으로 실행하기

`/tmp` 같은 임시 바이너리는 절대 사용하지 않습니다. 항상 아래 루틴을 따릅니다:

```bash
# 1. 워크스페이스 루트로 이동
cd <YOUR_WORKSPACE>  # 예: ~/ros2_pj, ~/ws/ros2_pj 등

# 2. 특정 패키지만 빌드
colcon build --packages-select planning_pkg

# 3. 환경 소싱 (빌드 후 항상!)
source install/setup.bash

# 4. 실행
ros2 run planning_pkg <노드이름> ...
```

### 전체 빌드가 필요한 경우

```bash
# 클린 빌드
./clean_build.sh                    # 문제가 있는 디렉토리 정리
colcon build --symlink-install      # 전체 빌드

source install/setup.bash
```

---

## Racing Agent 아키텍처 (v3.0)

### 🏎️ 핵심 철학

Racing Agent는 경로 계획 + 추월 의사결정 + 장애물 대응을 담당하는 상위 레벨 컨트롤러입니다.

**핵심 원칙:**

1. **추월은 임기응변이 아닙니다** - 글로벌 레이스 라인 상에 미리 정의된 OVERTAKE ZONE에서만, 사전에 준비된 추월 경로 후보들 중 하나를 선택하여 수행합니다.

2. **앞차/장애물 대응은 안전 우선** - 무리하게 피하려 하지 않고, "안전 추종 + 필요시 정지"를 기본 전략으로 합니다.

3. **로컬 컨트롤러는 추종에만 집중** - NMPC/로컬 컨트롤러는 상위 레벨이 준 모드/참조 경로/속도를 안정적으로 추종하며, 자기 멋대로 추월이나 급격한 회피를 시도하지 않습니다.

### 모드(State/Mode) 구조

| 모드 | 설명 |
|------|------|
| **CRUISE** | 글로벌 레이스 라인을 기준으로 목표 속도로 일반 주행 |
| **FOLLOW** | 앞차를 안전거리 내에서 부드럽게 추종 (ACC 스타일) |
| **OVERTAKE_CANDIDATE** | OVERTAKE ZONE에서 추월 경로 후보 평가 중 |
| **OVERTAKE** | 사전 계산된 추월 궤적 실행 중 |
| **OBSTACLE_STOP** | 장애물/충돌 위험 감지로 안전 정지 (v_ref=0) |

### OVERTAKE ZONE 구조

```yaml
# racing_agent_params.yaml 예시
n_overtake_zones: 2

overtake_zone_0:
  name: "straight_1"
  s_start: 0.0     # 시작 위치 (arc length, m)
  s_end: 15.0      # 종료 위치
  type: "OVERTAKE_ZONE"
  
overtake_zone_1:
  name: "straight_2"  
  s_start: 50.0
  s_end: 65.0
  type: "OVERTAKE_ZONE"
```

### 추월 궤적 구조

각 OVERTAKE_ZONE에 대해 여러 개의 추월 궤적 후보가 사전 계산됩니다:

| 궤적 ID | 설명 |
|---------|------|
| `zone_name_left_outside` | 왼쪽 외곽 라인으로 추월 (110% 오프셋) |
| `zone_name_left_inside` | 왼쪽 내곽 라인으로 추월 (70% 오프셋, apex 근처) |
| `zone_name_right_outside` | 오른쪽 외곽 라인으로 추월 (110% 오프셋) |
| `zone_name_right_inside` | 오른쪽 내곽 라인으로 추월 (70% 오프셋, apex 근처) |

### 인-코스(Inside) vs 아웃-코스(Outside) 추월

추월 시 내곽(inside)과 외곽(outside) 중 어디로 추월할지는 다음 요소를 기반으로 결정됩니다:

1. **트랙 지오메트리**: 코너 방향 (좌회전/우회전)
2. **상대 차량 위치**: 상대방의 횡방향 위치
3. **벽 거리**: 각 방향의 여유 공간

| 코너 방향 | 인사이드 추월 | 아웃사이드 추월 |
|-----------|---------------|-----------------|
| 좌회전 | 오른쪽으로 | 왼쪽으로 |
| 우회전 | 왼쪽으로 | 오른쪽으로 |
| 직선 | 공간이 더 넓은 쪽 | 반대쪽 |

### 거리 기반 추월 준비

앞차와의 거리에 따라 추월 경로가 사전 준비됩니다:

| 거리 구간 | 동작 |
|-----------|------|
| > 4.0m | 준비 없음, 단순 FOLLOW |
| 2.5m ~ 4.0m | 추월 경로 후보 평가 시작 |
| 1.5m ~ 2.5m | 추월 경로 준비 완료, 조건 대기 |
| < 1.5m | OVERTAKE ZONE 진입 시 즉시 실행 |

### OBSTACLE_STOP 동작

⚠️ **중요**: 장애물/벽 충돌 시 **절대로** 다음을 하지 않습니다:
- "반대 방향으로 강한 힘/토크를 줘서 튕겨나가는" 반발 로직
- 급격한 조향 변경

대신:
- **v_ref = 0**: 속도 목표를 0으로 설정
- **안정적 조향**: 조향은 급변하지 않고 천천히 중립으로 이동
- **상위 레벨 대기**: 재경로 계획 또는 수동 개입을 기다림

### 코너 핸들링 (Corner Handling)

코너에서 앞차를 추종할 때 더 안전하게 동작합니다:

| 파라미터 | 기본값 | 설명 |
|----------|--------|------|
| `corner_curvature_threshold` | 0.15 1/m | 코너 감지 곡률 임계값 |
| `corner_follow_distance_factor` | 1.3 | 코너에서 추종 거리 증가율 (30% 증가) |
| `corner_speed_reduction` | 0.8 | 코너에서 속도 감소율 (80%) |

### 시각화 토픽

| 토픽 | 설명 |
|------|------|
| `/racing_agent/visualization` | 레이스라인, OVERTAKE ZONE, 추월 궤적 마커 |
| `/racing_agent/mode` | 현재 모드 (String) |
| `/racing_agent/reference_path` | 로컬 컨트롤러에 전달되는 참조 경로 |

### Racing Agent 실행

```bash
# Racing Agent 실행 (증가된 추종 거리 사용)
ros2 run planning_pkg racing_agent --ros-args \
  -p decision_rate:=20.0 \
  -p cruise_speed:=5.0 \
  -p safe_follow_distance:=3.0
```

---

## NMPC 아키텍처 (v2.0)

### 🚀 새로운 기능: acados 연동 및 LifecycleNode

v2.0에서는 다음과 같은 아키텍처 개선이 적용되었습니다:

#### 1. acados 솔버 지원

| 항목 | 기존 (Ipopt 스타일) | 신규 (acados) |
|------|---------------------|---------------|
| **QP 솔버** | MUMPS (일반 sparse) | HPIPM (horizon-structured) |
| **알고리즘** | Interior Point | SQP/RTI |
| **웜스타트** | 제한적 | 기본 설계 |
| **코드생성** | 없음 | CasADi → C 코드 |
| **타이밍** | 변동 (30~50ms) | 예측 가능 (<20ms RTI) |

acados가 설치되어 있으면 자동으로 사용되며, 없으면 fallback 솔버가 동작합니다.

#### 2. LifecycleNode 패턴

```
Unconfigured → configure → Inactive → activate → Active
                            ↑                      ↓
                         cleanup ←─ deactivate ←───
```

**Lifecycle 사용법:**

```bash
# 노드 시작 (Unconfigured 상태)
ros2 run control_pkg nmpc_lifecycle_node

# Configure: 솔버 초기화, 메모리 할당
ros2 lifecycle set /nmpc_lifecycle_node configure

# Activate: 제어 루프 시작
ros2 lifecycle set /nmpc_lifecycle_node activate

# Deactivate: 제어 루프 중지 (정차)
ros2 lifecycle set /nmpc_lifecycle_node deactivate

# Cleanup: 리소스 해제
ros2 lifecycle set /nmpc_lifecycle_node cleanup
```

#### 3. MultiThreadedExecutor 지원

NMPC 솔버가 30~50ms 걸려도 다른 콜백이 블로킹되지 않도록:

- **센서 콜백 그룹** (Reentrant): odometry, LiDAR, path 콜백 병렬 실행
- **솔버 콜백 그룹** (MutuallyExclusive): NMPC solve는 단일 스레드

```cpp
// 내부 구현
sensor_callback_group_ = create_callback_group(Reentrant);
solver_callback_group_ = create_callback_group(MutuallyExclusive);
```

### acados 설치 방법

```bash
# 1. acados 설치
pip install acados-template casadi numpy

# 2. OCP 코드 생성
ros2 run control_pkg generate_acados_ocp.py --horizon 1.5 --steps 15

# 3. 생성된 C 코드를 CMake에 추가하고 빌드
colcon build --packages-select control_pkg
```

### NMPC 노드 선택 가이드

| 노드 | 용도 | 특징 |
|------|------|------|
| `nmpc_engine_node` | 레거시/테스트 | rclcpp::Node, SingleThreadedExecutor |
| `nmpc_lifecycle_node` | **프로덕션 권장** | LifecycleNode, MultiThreadedExecutor, acados |

---

### 📁 핵심 파일 위치

| 용도 | 파일 경로 |
|------|-----------|
| **센터라인 (Ground Truth)** | `src/planning_pkg/tracks/teras_centerline.csv` |
| **경계 포함 센터라인** | `src/planning_pkg/tracks/centerline_with_bounds.csv` |
| **생성된 레이스라인** | `src/planning_pkg/data/raceline.csv` |
| **레이스라인 생성기** | `src/planning_pkg/src/generate_raceline.cpp` |
| **레이스라인 서버** | `src/planning_pkg/src/raceline_server_node.cpp` |

### 레이스라인 생성 절차

#### Step 1: 센터라인 준비

센터라인 CSV 파일이 `tracks/` 폴더에 있어야 합니다.

```bash
# stack_master의 global_waypoints.json에서 경계 포함 센터라인 생성
# 워크스페이스 루트에서 실행
python3 src/planning_pkg/scripts/generate_centerline_with_bounds.py \
  --json src/stack_master/maps/teras/global_waypoints.json \
  --out src/planning_pkg/tracks/centerline_with_bounds.csv

# 또는 ros2 pkg prefix 사용
python3 $(ros2 pkg prefix planning_pkg)/share/planning_pkg/scripts/generate_centerline_with_bounds.py \
  --json $(ros2 pkg prefix stack_master)/share/stack_master/maps/teras/global_waypoints.json \
  --out $(ros2 pkg prefix planning_pkg)/share/planning_pkg/tracks/centerline_with_bounds.csv
```

#### Step 2: 레이스라인 생성

```bash
# 빌드
colcon build --packages-select planning_pkg
source install/setup.bash

# 레이스라인 생성 (경계 인식 모드 - 권장)
ros2 run planning_pkg generate_raceline \
  --centerline_csv $(ros2 pkg prefix planning_pkg)/share/planning_pkg/tracks/centerline_with_bounds.csv \
  --out_csv $(ros2 pkg prefix planning_pkg)/share/planning_pkg/data/raceline.csv \
  --lane_position 0.0 \
  --wall_margin 0.3 \
  --mu 1.0 \
  --v_max 5.0 \
  --ds 0.2
```

**주요 파라미터:**

| 파라미터 | 설명 | 기본값 |
|----------|------|--------|
| `--lane_position` | 트랙 위치 (-1.0=외곽, 0.0=중앙, 1.0=내곽) | 0.0 |
| `--wall_margin` | 벽과의 최소 거리 (m) | 0.3 |
| `--mu` | 마찰 계수 | 1.0 |
| `--v_max` | 최대 속도 (m/s) | 20.0 |
| `--ax_max` | 최대 가속도 (m/s²) | 4.0 |
| `--ax_min` | 최대 감속도 (m/s²) | -6.0 |
| `--ds` | 샘플 간격 (m) | 0.5 |

#### Step 3: 레이스라인 서버 실행

```bash
ros2 run planning_pkg raceline_server \
  --ros-args \
  -p raceline_file:=data/raceline.csv \
  -p frame_id:=map \
  -p publish_vref:=true
```

### ⚠️ 흔한 실수 방지

1. **`src/` vs `install/` 경로 혼동**
   - 코드 수정은 `src/planning_pkg/...` 에서
   - 실행 시 사용되는 데이터는 `install/planning_pkg/share/planning_pkg/...` 에서
   - 빌드 후 `install/` 경로의 파일이 사용됨

2. **수정 후 빌드 안 함**
   - CSV 데이터 파일을 `src/` 에서 수정했다면 **반드시 다시 빌드** 해야 함
   - 또는 `--symlink-install` 로 빌드하면 `src/` 수정이 바로 반영됨

3. **환경 소싱 누락**
   - 빌드 후 `source install/setup.bash` 필수!

---

## 센서 설정 튜닝

### 📁 핵심 파일 위치

| 용도 | 파일 경로 |
|------|-----------|
| **LiDAR 센서 파라미터** | `src/stack_master/config/sensors.yaml` |
| **차량별 설정** | `src/stack_master/config/NUC*/` |
| **상태 추정 (EKF)** | `src/state_estimation/config/ekf.yaml` |

### LiDAR 각도 범위 조정

`src/stack_master/config/sensors.yaml` 에서:

```yaml
urg_node:
  ros__parameters:
    angle_max: 3.14      # 최대 각도 (라디안)
    angle_min: -3.14     # 최소 각도 (라디안)
    # 측면 감지를 줄이려면 범위를 좁힘
    # 예: angle_max: 2.0, angle_min: -2.0 (약 ±115도)
```

**측면 감지 줄이기 예시:**

```yaml
# 전방 집중 모드 (약 ±90도)
angle_max: 1.57
angle_min: -1.57

# 전방 + 약간의 측면 (약 ±120도)
angle_max: 2.09
angle_min: -2.09
```

### 차량별 설정 (NUC2, NUC5, NUC6 등)

각 차량의 설정은 별도 폴더에 있습니다:

```
src/stack_master/config/
├── NUC2/
├── NUC5/
├── NUC6/
├── NUC7/
├── OrinNano/
└── SIM/
```

### 상태 추정 (EKF) 튜닝

`src/state_estimation/config/ekf.yaml` 에서 공분산 조정:

```yaml
# VESC 오도메트리 공분산
# 값이 작을수록 해당 센서를 더 신뢰
odom_covariance: [0.25, 0.5, 0.4]  # x, y, yaw

# IMU 공분산
imu_covariance: [0.0, 0.0, 0.0]  # 선형가속도, 각속도, 방향
```

---

## 컨트롤러 파라미터 튜닝

### 📁 핵심 파일 위치

| 용도 | 파일 경로 |
|------|-----------|
| **NMPC 파라미터 (권장)** | `src/project_launch/config/nmpc_params.yaml` |
| **Pure Pursuit 파라미터** | `src/planning_pkg/config/pure_pursuit_params.yaml` |
| **Simple Controller 파라미터** | `src/project_launch/config/control_params.yaml` |
| **NMPC 엔진 코드** | `src/control_pkg/src/nmpc_engine_node.cpp` |
| **Simple Controller 코드** | `src/control_pkg/src/simple_controller.cpp` |

### NMPC 파라미터 (권장 제어기)

NMPC(Nonlinear Model Predictive Control)는 고성능 자율 주행에 권장되는 제어기입니다.

> **📌 설정 파일**: `src/project_launch/config/nmpc_params.yaml`

**핵심 파라미터 (ROS2 파라미터로 설정):**

```yaml
nmpc_engine_node:
  ros__parameters:
    # 예측 수평선 설정 (개선됨)
    # ⚠️ 중요: dt = prediction_horizon / prediction_steps
    # 예: 1.5s / 15 = 0.1s (100ms) 시간 해상도
    # horizon 증가 → 더 먼 미래 예측 (코너 미리 인지)
    # steps 증가 → 더 세밀한 시간 해상도 (계산 비용 증가)
    prediction_horizon: 1.5      # 예측 구간 (초) - 높으면 미리 계획, 낮으면 반응적 (1.0->1.5 증가)
    prediction_steps: 15         # 예측 단계 수 (10->15 증가) - dt=0.1s 유지
    control_rate_hz: 50.0        # 제어 루프 주파수 (Hz)
    nominal_speed: 2.5           # 목표 속도 (m/s) (2.0->2.5 증가)
    
    # 비용 함수 가중치 (Issue 3.1 해결: 조향 변화율 페널티)
    w_pos: 10.0                  # 위치 추적 가중치
    w_yaw: 10.0                  # 헤딩 추적 가중치 (안정성 위해 8->10 증가)
    w_vel: 3.0                   # 속도 추적 가중치 (2.0->3.0 증가)
    w_steer: 0.5                 # 조향 입력 가중치
    w_accel: 0.3                 # 가속도 입력 가중치
    w_steer_rate: 600.0          # ⚠️ 조향 변화율 가중치 - 진동 억제에 핵심! (500->600 증가)
    w_accel_rate: 60.0           # 가속도 변화율 가중치
    w_terminal: 30.0             # 종단 비용 가중치 (20->30 증가)
    
    # 횡방향 허용 튜브 (Issue 5.1: 최적 레이싱 라인 허용)
    lateral_tolerance: 0.25      # 중심선에서 ±0.25m 이탈 허용 (0.3->0.25 감소)
    
    # 지연 보상 (Issue 3.3: 계산 지연 보상)
    latency_compensation_sec: 0.05  # 50ms 전체 시스템 지연 보상 (0.02->0.05 증가)
    
    # 솔버 안정성 (NEW: Issue 3.1 해결)
    levenberg_marquardt: 0.01    # L-M 정칙화 - Status 3 오류 방지 (헤시안 특이점 해결)
    max_solver_iterations: 20    # 최대 반복 횟수 (15->20 증가)
    
    # 동역학 모델 (NEW: 고속 주행 지원)
    dynamic_model_threshold: 2.5 # 동역학 모델 전환 속도 [m/s] (이 속도 이상에서 타이어 슬립 고려)
    vehicle_mass: 3.5            # 차량 질량 [kg]
    vehicle_inertia: 0.04        # 요 관성 모멘트 [kg*m²]
    
    # 제약 조건 (개선됨)
    max_steer: 0.436             # 최대 조향각 [rad] (25도)
    max_steer_rate: 1.8          # 최대 조향 변화율 [rad/s] (1.5->1.8 증가)
    max_speed: 6.0               # 최대 속도 [m/s] (5.0->6.0 증가)
    max_accel: 4.0               # 최대 가속도 [m/s²] (3.0->4.0 증가)
    min_accel: -6.0              # 최대 감속도 [m/s²] (-5.0->-6.0 증가)
```

**NMPC 개선 사항 (v2.0):**

| 개선 사항 | 설명 | 효과 |
|----------|------|------|
| **Levenberg-Marquardt 정칙화** | 헤시안 행렬에 λI 추가 | Status 3 오류 (수치적 특이점) 방지 |
| **Soft Constraints** | 슬랙 변수로 제약 조건 완화 | Status 4 오류 (Infeasibility) 방지 |
| **동역학 모델 전환** | 속도 2.5m/s 이상에서 자동 전환 | 고속에서 타이어 슬립 고려 |
| **지연 보상 강화** | 50ms 시스템 지연 예측 | 고속 주행 시 위상 지연 보상 |
| **터미널 비용 강화** | 예측 끝단 가중치 증가 | 코너 직진 현상 방지 |

**NMPC 튜닝 가이드:**

| 증상 | 원인 | 조치 |
|------|------|------|
| 직진 시 좌우 진동 (Snaking) | `w_steer_rate` 너무 낮음 | `w_steer_rate`를 600 이상으로 증가 |
| 코너에서 반응 느림 | `w_steer_rate` 너무 높음 | `w_steer_rate`를 300~400으로 감소 |
| 중심선 과도 추종 | `lateral_tolerance` 없음 | `lateral_tolerance`를 0.2~0.3으로 설정 |
| 고속에서 불안정 | 지연 보상 부족 | `latency_compensation_sec` 0.05~0.1 |
| 코너 오버슈트 | `w_yaw` 너무 낮음 | `w_yaw`를 10~15로 증가 |
| 속도 유지 안됨 | `w_vel` 너무 낮음 | `w_vel`를 3~5로 증가 |
| Status 3 오류 (NaN) | 헤시안 특이점 | `levenberg_marquardt` 0.01~0.1 |
| Status 4 오류 (Infeasible) | 제약 조건 불만족 | Soft constraint가 자동 처리됨 |
| 고속 코너링 언더스티어 | 기구학 모델 한계 | `dynamic_model_threshold` 낮추기 |
| 코너 출구에서 벽에 너무 가까움 | 코너 출구 스무딩 비활성화 또는 마진 부족 | `corner_exit_wall_margin` 0.4~0.6으로 설정 |
| 직선 복귀 시 급격한 조향 보정 | 스무딩 설정 부족 | `corner_exit_steer_rate_limit` 0.6~0.8로 낮춤 |
| 코너 출구에서 S자 보정 | 측면 편차 한계 초과 | `corner_exit_lateral_limit` 0.1~0.15로 설정 |

### 코너 출구 스무딩 (v5.0)

코너를 빠져나와 직선 구간에 진입할 때 과도한 횡방향 움직임을 줄여줍니다.

**주요 파라미터:**

```yaml
# 코너 출구 스무딩 활성화/비활성화
enable_corner_exit_smoothing: true

# 벽과의 최소 거리 (m) - 코너 출구에서 벽에 너무 가까이 가지 않도록
corner_exit_wall_margin: 0.4

# 최대 횡방향 편차 (m) - 레퍼런스 경로에서 얼마나 벗어날 수 있는지
corner_exit_lateral_limit: 0.15

# 조향 제한 비율 (0-1) - 코너 출구에서 조향 크기 제한
corner_exit_steer_rate_limit: 0.8

# 전환 시간 (s) - 스무딩 효과 지속 시간
corner_exit_transition_time: 1.0
```

**작동 원리:**
1. 곡률 변화를 감지하여 코너 출구를 인식 (높은 곡률 → 낮은 곡률)
2. 전환 구간 동안:
   - 조향 크기를 제한하여 급격한 보정 방지
   - 횡방향 편차를 모니터링하여 벽에 너무 가까이 가지 않도록
   - 점진적으로 정상 제어로 복귀

### Pure Pursuit 파라미터 (백업 제어기)

`src/planning_pkg/config/pure_pursuit_params.yaml`:

```yaml
pure_pursuit_node:
  ros__parameters:
    lookahead_distance: 1.5  # 전방 주시 거리 (m) - 높으면 부드럽게, 낮으면 민감하게
    max_speed: 4.0           # 최대 속도 (m/s)
    kp: 0.5                  # 속도 비례 이득
    kd: 0.1                  # 속도 미분 이득
```

**튜닝 가이드:**

| 증상 | 조치 |
|------|------|
| 코너에서 오버슈트 | `lookahead_distance` 줄임 |
| 직진 시 불안정 | `lookahead_distance` 늘림 |
| 반응이 느림 | `kp` 늘림 |
| 오실레이션 발생 | `kd` 늘림, `kp` 줄임 |

---

## NMPC vs Pure Pursuit 선택 가이드

| 상황 | 권장 제어기 | 이유 |
|------|-------------|------|
| 고속 레이싱 | NMPC | 예측 제어로 미래 경로 최적화 |
| 급커브 구간 | NMPC | 전체 경로 고려한 조향 계획 |
| 저속 정밀 주행 | Pure Pursuit | 간단하고 안정적 |
| 계산 자원 제한 | Pure Pursuit | 낮은 계산 비용 |
| 장애물 회피 필요 | NMPC | 소프트 제약으로 유연한 회피 |

---

## 트러블슈팅

### 문제: NMPC 직진 구간에서 진동 (Oscillation)

**원인:** 조향 변화율(Slew Rate) 페널티 부족

**해결:**
```bash
# w_steer_rate 파라미터를 500 이상으로 설정
ros2 param set /nmpc_engine_node w_steer_rate 500.0
```

### 문제: NMPC가 코너를 직선으로 자르려 함 (Corner Cutting)

**원인:** 횡방향 허용 튜브가 너무 넓거나 참조 궤적 생성 문제

**해결:**
```bash
# lateral_tolerance 줄이기
ros2 param set /nmpc_engine_node lateral_tolerance 0.1
```

### 문제: 수정한 raceline.csv가 반영 안 됨

**원인:** `src/`와 `install/` 경로 혼동

**해결:**
```bash
# 방법 1: 다시 빌드
colcon build --packages-select planning_pkg
source install/setup.bash

# 방법 2: symlink-install 사용 (개발 시 권장)
colcon build --packages-select planning_pkg --symlink-install
source install/setup.bash
```

### 문제: generate_raceline 실행 시 파일 못 찾음

**원인:** 상대 경로 사용

**해결:**
```bash
# ros2 pkg prefix로 절대 경로 사용
ros2 run planning_pkg generate_raceline \
  --centerline_csv $(ros2 pkg prefix planning_pkg)/share/planning_pkg/tracks/teras_centerline.csv \
  --out_csv $(ros2 pkg prefix planning_pkg)/share/planning_pkg/data/raceline.csv
```

### 문제: 시뮬레이션에서 차량/스캔이 안 보임

**해결:**
```bash
# race_stack 설치 스크립트가 있다면 실행
# 경로는 환경에 따라 다를 수 있음
source <YOUR_WORKSPACE>/src/race_stack/.install_utils/f110_sim_setup.sh
```

### 문제: 조이스틱이 작동 안 함

**해결:**
```bash
sudo chmod 666 /dev/input/js0
sudo chmod 666 /dev/input/event*
```

### 문제: 벽을 인식하지 못하거나 회피 조향이 안 됨

**원인:** A1/A2 임계값이 너무 작거나, 조향 강도가 너무 낮음

**증상 확인:**
1. LiDAR 토픽 확인: `ros2 topic echo /scan`
2. 컨트롤러 로그 확인: 벽 근처에서 `[WALL AVOID]` 로그가 출력되는지 확인

**해결 (control_params.yaml에서 조정):**
```yaml
# 벽 인식 거리 증가 (더 일찍 감지)
a1_threshold: 0.15         # 후진 트리거 (0.15m 이내)
a2_threshold: 0.6          # 조향 회피 (0.6m 이내)

# 회피 조향 강도 증가
a2_steer_gain: 0.8         # 더 강한 회피 조향 (기본 0.4 -> 0.8)
a2_max_steer_ratio: 0.8    # 최대 조향 비율 (기본 0.5 -> 0.8)
```

**A1/A2 시스템 설명:**
| 범위 | 거리 | 동작 |
|------|------|------|
| A1 | < a1_threshold (0.15m) | 후진 + 반대방향 조향 |
| A2 | a1 < dist < a2_threshold (0.6m) | 반대방향 조향만 (속도 감소) |

**디버깅 팁:**
- 벽 감지 시 `[WALL AVOID] RIGHT (wall on LEFT)` 형태의 로그 확인
- 조향 값(steer), 거리(L/R/F), 긴급도(urgency) 값 확인

---

## 빠른 참조: 자주 쓰는 명령어

```bash
# 특정 패키지만 빌드
colcon build --packages-select planning_pkg

# 개발 모드 빌드 (src/ 수정이 바로 반영)
colcon build --packages-select planning_pkg --symlink-install

# 환경 소싱
source install/setup.bash

# 레이스라인 서버 실행
ros2 run planning_pkg raceline_server

# 레이스라인 생성
ros2 run planning_pkg generate_raceline --help

# 노드 목록 확인
ros2 node list

# 토픽 목록 확인
ros2 topic list

# 토픽 에코
ros2 topic echo /global_raceline

# 파라미터 확인
ros2 param list
ros2 param get <node_name> <param_name>

# 패키지 경로 확인
ros2 pkg prefix planning_pkg
```

---

## 파일 구조 요약

```
ros2_pj/
├── src/
│   ├── planning_pkg/           # 경로 계획
│   │   ├── src/
│   │   │   ├── generate_raceline.cpp    # 레이스라인 생성기
│   │   │   └── raceline_server_node.cpp # 레이스라인 서버
│   │   ├── tracks/                      # 센터라인 CSV
│   │   └── data/                        # 레이스라인 CSV
│   │
│   ├── stack_master/           # 메인 런치 및 설정
│   │   ├── config/             # 차량별 설정
│   │   │   ├── sensors.yaml    # 센서 파라미터
│   │   │   ├── NUC2/, NUC5/, ... # 차량별 설정
│   │   └── maps/               # 트랙 맵
│   │       └── teras/          # teras 트랙
│   │
│   ├── control_pkg/            # 컨트롤러
│   └── state_estimation/       # 상태 추정 (EKF)
│
├── install/                    # 빌드 출력 (실행 시 사용되는 경로)
├── build/                      # 빌드 중간 파일
└── TUNING_GUIDE.md             # 이 문서
```

---

**작성일:** 2025-12-04  
**목적:** 레포지토리가 복잡하더라도 튜닝을 효율적으로 할 수 있도록 핵심만 정리

---

## CRSM (Collision Recovery State Machine) 시스템

### 🚗 충돌 복구 아키텍처

CRSM은 벽 충돌 시 발생하는 후진 불량 및 조향 고착 현상을 해결하기 위해 설계된 상태 머신입니다.

#### 상태 정의

| 상태 | 설명 | 동작 |
|------|------|------|
| ST_NORMAL | 정상 주행 | 일반 제어기 명령 수행 |
| ST_CRASH_DETECTED | 충돌 감지 | 즉시 정지, 제어권 탈취 |
| ST_RECOVERY_REVERSE | 개루프 후진 | 조향 역보정 후진 |
| ST_RECOVERY_REALIGN | 재정렬 | 정지 후 경로 재생성 |

#### 충돌 감지 트리거

CRSM은 다음 세 가지 방법으로 충돌을 감지합니다:

1. **A1 Zone 감지**: LiDAR 전방 < 0.18m
2. **IMU 충돌 감지**: |a_total| > 9.5 m/s² (급격한 감속)
3. **스톨 감지**: 전진 명령 있으나 속도 0, 전방 장애물 있음

#### 조향 역보정 모드

`reverse_steering_mode` 파라미터로 설정:

| 모드 | 값 | 동작 | 용도 |
|------|-----|------|------|
| NEUTRAL | 0 | δ_rec = 0 | 안전한 기본값 |
| INVERT | 1 | δ_rec = -δ_last | 벽에서 빠른 탈출 |
| MAINTAIN | 2 | δ_rec = δ_last | 레거시 호환 |

**권장 설정:**
```yaml
simple_controller:
  ros__parameters:
    reverse_steering_mode: 0       # NEUTRAL (권장) 또는 1 (INVERT)
    reverse_speed: 0.8             # 후진 속도 (m/s)
    reverse_duration: 1.2          # 후진 시간 (초)
    a1_threshold: 0.18             # 후진 트리거 거리 (m)
```

### 시각화 토픽

| 토픽 | 메시지 타입 | 설명 |
|------|-------------|------|
| `/crsm_state` | std_msgs/String | 현재 CRSM 상태 |
| `/wall_collision_indicator` | Marker | 충돌 시 빨간 구 |
| `/safety_zones` | MarkerArray | 상대 차량 금지 구역 |

---

## ACC (Adaptive Cruise Control) Following 시스템

### 🚙 1.5m 간격 적응형 팔로잉

ACC 시스템은 상대 차량을 따라갈 때 1.5m 간격을 유지하면서도 추월 기회가 생기면 즉시 반응합니다.

#### 제어 법칙

보고서 기반 PD 제어:
```
v_cmd = v_opp + Kp * (gap - 1.5) + Kd * (v_opp - v_ego)
```

- 거리 > 1.5m: 가속하여 따라붙음
- 거리 < 1.5m: 감속하여 간격 유지
- 상대 속도에 동기화하여 안정적 Following

#### 핵심 파라미터

```yaml
simple_controller:
  ros__parameters:
    # ACC 제어 파라미터
    acc_kp: 0.5                    # 거리 오차 비례 게인
    acc_kd: 0.2                    # 상대 속도 미분 게인
    target_follow_gap: 1.5         # 목표 차간 거리 (m)
    follow_distance_threshold: 1.5 # Following 시작 거리 (m)
    follow_min_speed_ratio: 0.1    # 최소 속도 비율 (10%)
```

### 동적 추월/팔로잉 전환

시스템은 매 제어 주기(20Hz)마다 추월 가능성을 평가합니다:

1. **추월 경로 검증**: 상대 차량 폭 + 안전 마진 고려
2. **금지 구역 계산**: D_forbidden = [d_opp - W/2 - margin, d_opp + W/2 + margin]
3. **즉시 전환**: 유효한 추월 경로 발견 시 즉시 추월 모드

#### 차량 폭 기반 추월 판단

```yaml
# 차량 치수
vehicle_width: 0.3               # 차량 넓이 (m)
vehicle_length: 0.5              # 차량 길이 (m)

# 추월 조건 (자동 계산)
# min_overtake_clearance = vehicle_width * 2 + SAFETY_MARGIN (0.3m)
# 약 0.9m 이상의 간격이 있어야 추월 시도
```

### 시각화

| 마커 | 색상 | 설명 |
|------|------|------|
| 추월 경로 | 녹색 라인 | S자 곡선 추월 경로 |
| 추월 방향 | 노란 화살표 | 추월 방향 표시 |
| 금지 구역 | 빨간 반투명 박스 | 상대 차량 + 안전 마진 |
| Following 앵커 | 파란 구 | 목표 위치 (상대 차량 뒤 1.5m) |

---

## 데이터 및 파라미터 요약

### 보고서 기반 권장 파라미터 (Research Report Specifications)

| 파라미터 | 권장값 | 설명 |
|----------|--------|------|
| **CRSM (충돌 복구)** | | |
| reverse_steering_mode | 0 or 1 | 조향 모드 (0=중립-안전, 1=반전-빠른 탈출) |
| a1_threshold | 0.18m | 후진 트리거 거리 |
| reverse_speed | 1.5 m/s | 후진 속도 (강력한 토크) |
| reverse_duration | 1.2~1.5s | 후진 지속 시간 |
| IMU_CRASH_ACCEL_THRESHOLD | 9.5 m/s² | IMU 충돌 감지 임계값 |
| **ACC (적응형 순항)** | | |
| acc_kp | 0.5 | ACC 비례 게인 |
| acc_kd | 0.2 | ACC 미분 게인 |
| target_follow_gap | 1.5m | 목표 차간 거리 |
| **추월 시스템** | | |
| vehicle_width | 0.35m | 차량 폭 (F1TENTH) |
| SAFETY_MARGIN | 0.15m | 안전 마진 |
| narrow_road_threshold | 3.0m | 좁은 도로 판단 기준 (Max Lateral)
