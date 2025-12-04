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
| **컨트롤러 코드** | `src/control_pkg/src/` |

### Pure Pursuit 파라미터

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

## 트러블슈팅

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
