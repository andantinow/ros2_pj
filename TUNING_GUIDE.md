# 튜닝 가이드 (Tuning Guide)

이 문서는 레포지토리 구조가 복잡하더라도 튜닝을 쉽게 할 수 있도록 핵심 포인트만 정리한 가이드입니다.

---

## 📋 목차

1. [빌드 및 실행 기본 원칙](#빌드-및-실행-기본-원칙)
2. [글로벌 레이스라인 튜닝](#글로벌-레이스라인-튜닝)
3. [센서 설정 튜닝](#센서-설정-튜닝)
4. [컨트롤러 파라미터 튜닝](#컨트롤러-파라미터-튜닝)
5. [트러블슈팅](#트러블슈팅)

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

## 글로벌 레이스라인 튜닝

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
| **Pure Pursuit 파라미터** | `src/planning_pkg/config/pure_pursuit_params.yaml` |
| **NMPC 엔진 코드** | `src/control_pkg/src/nmpc_engine_node.cpp` |
| **Simple Controller 코드** | `src/control_pkg/src/simple_controller.cpp` |

### NMPC 파라미터 (권장 제어기)

NMPC(Nonlinear Model Predictive Control)는 고성능 자율 주행에 권장되는 제어기입니다.

**핵심 파라미터 (ROS2 파라미터로 설정):**

```yaml
nmpc_engine_node:
  ros__parameters:
    # 예측 수평선 설정 (개선됨)
    prediction_horizon: 1.5      # 예측 구간 (초) - 높으면 미리 계획, 낮으면 반응적 (1.0->1.5 증가)
    prediction_steps: 15         # 예측 단계 수 (10->15 증가)
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
