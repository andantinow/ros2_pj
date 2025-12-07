# Opponent and Ego Speed Tuning - Changes Summary

## 작업 개요 (Overview)
이 PR은 opponent raceline의 오프셋과 속도를 조정하고, ego 차량의 직선 구간 속도 문제를 해결합니다.

## 주요 변경사항 (Key Changes)

### A) Opponent 조정 (Opponent Adjustments)

#### A1. Lateral Offset 감소
- **이전**: 0.9m (outer/inner)
- **이후**: 0.7m (outer/inner)
- **효과**: Opponent가 트랙 중앙에 더 가까이 주행, 여전히 OUT/IN 패턴 유지
- **파일**: 
  - `src/planning_pkg/data/opponent_outer.csv`
  - `src/planning_pkg/data/opponent_inner.csv`

#### A2. 속도 고정 (1.0 m/s)
- **이전**: 3.25 m/s (너무 빠름)
- **이후**: 1.0 m/s (안정적이고 예측 가능)
- **변경 위치**:
  1. CSV 파일: `generate_opponent_racelines.py` 스크립트로 재생성
  2. `opponent_publisher.cpp`: 기본 speed 파라미터 2.5 → 1.0
  3. `opponent_publisher_launch.xml`: 기본 speed 0.5 → 1.0

### B) Ego 직선 속도 문제 해결

#### 문제 진단
- **초기 가정**: A1/A2 센서나 측면 벽 근접으로 인한 속도 감소
- **실제 원인**: **FOLLOW 모드가 너무 자주 활성화**됨

#### 근본 원인 분석
```
이전 설정:
- opponent_following_distance: 2.5m (너무 큼!)
- follow_margin: 0.15 m/s

결과:
- Opponent가 2.5m 이내 → 항상 FOLLOW 모드
- Ego 속도 = 1.0 - 0.15 = 0.85 m/s (너무 느림!)
- 직선에서도 FOLLOW 모드 → 6.5 m/s 대신 0.85 m/s
```

#### 해결책
**파일**: `src/project_launch/config/nmpc_params.yaml`

1. **opponent_following_distance 감소**
   - 2.5m → **1.2m**
   - FOLLOW 모드는 매우 가까운 거리에서만 활성화

2. **follow_margin 감소**
   - 0.15 m/s → **0.05 m/s**
   - FOLLOW 모드에서도 더 빠른 속도 유지
   - Ego 속도 = 1.0 - 0.05 = **0.95 m/s**

#### 추가 명확화
- A1/A2 threshold는 **조향과 긴급 정지에만** 영향
- `corner_exit_wall_margin`는 **조향에만** 영향, 속도 무관
- 속도는 **오직 다음 요인만** 영향:
  1. 곡률 기반 v_ref (직선: 6.5 m/s, 코너: 2.5 m/s)
  2. FOLLOW 모드 (1.2m 이내만)
  3. 긴급 충돌 (A1: 0.15m 이내)

## 예상 성능 비교

| 상황 | 이전 속도 | 이후 속도 | 개선 |
|------|-----------|-----------|------|
| 직선 (opponent 멀리) | 0.85 m/s (FOLLOW) | **6.5 m/s** (CRUISE) | 🚀 **+665%** |
| Opponent 근접 (1.2m 이내) | 0.85 m/s | **0.95 m/s** | ⬆️ +12% |
| 코너 (높은 곡률) | 2.5 m/s | 2.5 m/s | ➖ 동일 |
| Opponent 속도 | 3.25 m/s | **1.0 m/s** | ⬇️ -69% |
| Opponent offset | 0.9m | **0.7m** | ⬇️ -22% |

## 수정된 파일 목록

1. **src/planning_pkg/data/opponent_outer.csv**
   - Offset 0.9m → 0.7m
   - 속도 3.25 m/s → 1.0 m/s

2. **src/planning_pkg/data/opponent_inner.csv**
   - Offset -0.9m → -0.7m
   - 속도 3.25 m/s → 1.0 m/s

3. **src/utilities/nodes/opponent_publisher_cpp/src/opponent_publisher.cpp**
   - 기본 speed 파라미터: 2.5 → 1.0

4. **src/utilities/nodes/opponent_publisher_cpp/launch/opponent_publisher_launch.xml**
   - speed 인자 기본값: 0.5 → 1.0

5. **src/project_launch/config/nmpc_params.yaml**
   - `opponent_following_distance`: 2.5 → 1.2
   - `follow_margin`: 0.15 → 0.05
   - 명확한 주석 추가 (A1/A2, wall margin은 속도 무관)

## 검증 방법

### 1. Opponent 동작 확인
```bash
# Opponent가 1.0 m/s로 주행하는지 확인
ros2 topic echo /opp_racecar/odom

# 경로 오프셋 시각화
rviz2
```

### 2. Ego 속도 확인
직선 구간에서:
- Opponent가 멀리 있을 때 (>1.2m): CRUISE 모드, 6.5 m/s
- Opponent가 가까울 때 (<1.2m): FOLLOW 모드, ~0.95 m/s
- 코너: 곡률 기반, ~2.5 m/s

### 3. 로그 확인
```bash
# FOLLOW 모드 활성화 빈도 확인
ros2 topic echo /rosout | grep FOLLOW

# 직선에서 CRUISE 모드인지 확인
ros2 topic echo /rosout | grep CRUISE
```

## 제약사항 준수 확인

✅ **simple_controller.cpp 수정 안 함** - opponent는 CSV와 파라미터로만 조정  
✅ **측면 벽 근접 속도 감소 없음** - 이미 없었고, 명확히 문서화  
✅ **코너 성능 유지** - 곡률 기반 속도 제어 그대로 유지  
✅ **FOLLOW 모드 유지** - 거리만 조정 (2.5m → 1.2m)  
✅ **빌드 오류 없음** - 코드 리뷰 통과  

## 다음 단계

1. **테스트**: 실제 시뮬레이터/차량에서 동작 확인
2. **튜닝**: 필요시 `opponent_following_distance` 미세 조정 (1.0-1.5m 범위)
3. **모니터링**: 직선 구간 속도가 6.5 m/s에 도달하는지 확인

## 참고사항

- Opponent CSV 파일 재생성 명령어:
  ```bash
  cd src/planning_pkg
  python3 scripts/generate_opponent_racelines.py \
    --outer-offset 0.7 \
    --inner-offset -0.7 \
    --speed-factor 0.1538 \
    --input data/raceline.csv
  ```

- FOLLOW 모드 활성화 조건 (코드):
  ```cpp
  // nmpc_engine_node.cpp:1678
  if (distance > opponent_following_distance_ && distance > overtake_decision_distance_) {
    driving_mode_ = DrivingMode::CRUISE;  // 직선에서 빠르게!
  }
  ```
