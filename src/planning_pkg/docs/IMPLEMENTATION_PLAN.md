# 3-Pillar Implementation Plan

## Overview
F1TENTH 자율주행을 위한 3가지 핵심 Pillar 구현 계획

## Pillar 1: 오프라인 글로벌 플래닝 (Minimum Time OCP)

### 현재 상태
- ✅ 기본 경로 생성 (`generate_raceline.cpp`)
- ✅ 곡률 기반 속도 제한
- ✅ Forward-Backward 가속/감속 제한
- ❌ G-G 다이어그램 제약 없음
- ❌ 동역학 제약 미고려
- ❌ 최적 제어 문제 (OCP) 미구현

### 구현 계획

#### 1.1 G-G 다이어그램 제약 추가
```cpp
// 제약: a_x^2 + a_y^2 <= (μ*g)^2
double g_g_limit = mu * G;
double a_lat = v^2 * kappa;  // 횡가속도
double a_lon_max = sqrt(g_g_limit^2 - a_lat^2);  // 가능한 종가속도
```

#### 1.2 동역학 제약 (Dynamic Bicycle Model)
- 현재: 단순 곡률 기반 속도 제한
- 개선: 실제 차량 동역학 모델 고려
  - 전륜/후륜 타이어 힘
  - 미끄러짐 각도 (slip angle)
  - 관성 모멘트

#### 1.3 Forward-Backward Solver 개선
- 현재: 단순 가속/감속 한계만 고려
- 개선: 
  - Forward pass: 곡률 → 최대 속도 계산
  - Backward pass: 감속 한계 고려
  - G-G 다이어그램 제약 적용

### 파일 구조
```
planning_pkg/
  src/
    generate_raceline.cpp (개선)
    minimum_time_ocp.cpp (신규)
  include/
    planning_pkg/
      gg_diagram.hpp (신규)
      vehicle_dynamics.hpp (신규)
```

---

## Pillar 2: 적응형 측위 (Dual/Joint EKF)

### 현재 상태
- ✅ 기본 EKF (`robot_localization` 사용)
- ✅ 단순 파라미터 추정 (`estimator_node.cpp` - mu, Cf, Cr)
- ❌ Dual EKF 구조 없음
- ❌ State와 Parameter 동시 추정 없음

### 구현 계획

#### 2.1 Dual EKF 구조
```
State EKF (100Hz)
  - 입력: IMU, Odometry
  - 상태: [x, y, θ, vx, vy, ω]
  - 빠른 업데이트

Parameter EKF (10Hz)
  - 입력: State EKF 예측 vs 실제 센서 오차
  - 상태: [μ, Cf, Cr]
  - 느린 업데이트
  - Feedback to State EKF
```

#### 2.2 Joint EKF 옵션
```
Joint EKF (50Hz)
  - 상태: [x, y, θ, vx, vy, ω, μ, Cf, Cr]
  - 하나의 필터로 통합 추정
  - 계산 복잡도 높음
```

### 파일 구조
```
localization_pkg/ (또는 state_estimation/)
  src/
    dual_ekf_node.cpp (신규)
    joint_ekf_node.cpp (신규)
    state_ekf.cpp (신규)
    parameter_ekf.cpp (신규)
  include/
    localization_pkg/
      dual_ekf.hpp
      joint_ekf.hpp
```

---

## Pillar 3: 실시간 통합 제어 (Contour-Following NMPC)

### 현재 상태
- ✅ Pure Pursuit 컨트롤러 (`simple_controller.cpp`)
- ❌ NMPC 없음
- ❌ acados 연동 없음
- ❌ Contour-Following 없음

### 구현 계획

#### 3.1 acados 연동
- acados 설치 및 CMake 연동
- C++ 인터페이스 설정
- 코드 생성 스크립트

#### 3.2 NMPC 모델 정의
```cpp
// Dynamic Bicycle Model
x = [x, y, θ, v, δ]  // 상태
u = [a, δ_dot]       // 제어 입력

// 제약
- G-G 다이어그램: a_x^2 + a_y^2 <= (μ*g)^2
- 속도: v_min <= v <= v_max
- 조향각: δ_min <= δ <= δ_max
```

#### 3.3 Contour-Following 비용 함수
```cpp
J = w1 * e_c^2 + w2 * (v - v_ref)^2 + w3 * δ_dot^2
  - e_c: 경로 이탈 오차 (contour error)
  - v_ref: Pillar 1에서 받은 속도 프로파일
  - δ_dot: 핸들 급조작 방지
```

#### 3.4 RTI (Real-Time Iteration)
- Warm Start: 이전 해를 초기값으로 사용
- 0.01초 내 계산 완료 보장

### 파일 구조
```
control_pkg/
  src/
    nmpc_controller.cpp (신규)
    acados_wrapper.cpp (신규)
  include/
    control_pkg/
      nmpc_model.hpp
      contour_following.hpp
  acados/
    (acados 생성 파일)
```

---

## 구현 우선순위

### Phase 1: 기초 (1-2주)
1. **Pillar 1 개선** (가장 영향 큼)
   - G-G 다이어그램 제약 추가
   - Forward-Backward Solver 개선

### Phase 2: 측위 (2-3주)
2. **Pillar 2 구현**
   - Dual EKF 또는 Joint EKF 선택
   - State/Parameter 동시 추정

### Phase 3: 제어 (3-4주)
3. **Pillar 3 구현**
   - acados 연동
   - NMPC 구현
   - RTI 최적화

---

## 의존성

### Pillar 1
- Eigen3 (행렬 연산)
- (선택) CasADi (최적화)

### Pillar 2
- robot_localization (기존 EKF 확장)
- Eigen3

### Pillar 3
- acados
- BLASFEO
- HPIPM

---

## 참고 자료
- Minimum Time OCP: "Time-optimal path following for robots" (Rajamani)
- Dual EKF: "Adaptive Vehicle Traction Force Control" (Rajamani)
- NMPC: "Contour-Following NMPC for Autonomous Racing" (Liniger)

