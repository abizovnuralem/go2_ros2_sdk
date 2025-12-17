

## Nav2 파라미터 변경 로그 (go2_robot_sdk/config/nav2_params.yaml)

이 문서는 `nav2_params.yaml`의 변경을 **왜 했는지(문제/가설)**, **무엇을 바꿨는지(파라미터/Before/After)**, 그리고 **예상 기대결과/검증 방법/롤백 방법**까지 같이 기록하기 위한 로그입니다.

### 작성 규칙 (앞으로 계속 누적)

1. `nav2_params.yaml`을 수정하면, 동일한 커밋/PR에서 이 파일의 로그 테이블도 같이 갱신합니다.
2. “왜”는 가능한 한 관찰된 증상(사용자 발화/로그/영상)으로 적습니다.
3. “기대결과”는 측정 가능한 형태(RViz에서 pose jump 감소, CPU 사용률, 수렴 속도 등)로 적습니다.

### 변경 로그

#### AMCL (Localization)

##### AMCL 튜닝 전(원본) 값 요약

```yaml
amcl:
  ros__parameters:
    alpha1: 0.2
    alpha2: 0.2
    alpha3: 0.2
    alpha4: 0.2
    alpha5: 0.2
    max_particles: 2000
    update_min_a: 0.2
    update_min_d: 0.25
```

##### 변경 로그 테이블 (AMCL)

| 날짜 | 파라미터(경로) | Before | After | 변경 이유(관찰/사용자 발화) | 예상 기대결과 | 검증 방법(체크리스트) | 리스크/트레이드오프 | 롤백 |
|---|---|---|---|---|---|---|---|---|
| 2025-12-17 | `amcl.ros__parameters.alpha1~4` | `0.2` | `0.05` | 특징 없는 정사각형 맵에서 corner 대칭으로 라이다 매칭이 헷갈려 **pose가 다른 corner로 점프**하는 현상 | 오도메트리를 더 신뢰해서 **대칭 환경에서 순간이동/튐 감소** | RViz에서 `amcl_pose` / `tf(map->odom)` 점프 여부 확인, 같은 루트로 반복 주행 시 재현성 확인 | 라이다 기반 보정이 약해져 장거리 누적 drift가 커질 수 있음(환경에 따라) | `alpha1~4`를 `0.2`로 복귀 |
| 2025-12-17 | `amcl.ros__parameters.max_particles` | `2000` | `3000` | 대칭 환경에서 후보 위치가 여러 곳으로 분산되므로 particle 다양성 확보 필요 | ambiguity 상황에서 **진짜 위치 particle 생존 확률 증가**, 튐 감소 | CPU 사용률/AMCL 업데이트 주기 확인, pose 튐 빈도 비교 | CPU/메모리 사용 증가(저사양에서는 latency 증가 가능) | `max_particles: 2000`으로 복귀 |
| 2025-12-17 | `amcl.ros__parameters.update_min_d`, `update_min_a` | `0.25`, `0.2` | `0.2`, `0.2` | 너무 자주 업데이트하면 노이즈가 누적되어 jitter/맵 회전처럼 보이는 현상 유발 가능 | **정지/미세 움직임에서 jitter 감소**, 안정적인 pose | 제자리/저속 이동에서 pose 흔들림 감소 확인, tracking 안정성 확인 | 반응성이 떨어져 빠른 움직임에서 pose 갱신이 늦어 보일 수 있음 | `update_min_d: 0.25`, `update_min_a: 0.2`로 복귀 |

##### 변경 기록 템플릿 (AMCL)

| 날짜 | 파라미터(경로) | Before | After | 변경 이유(관찰/사용자 발화) | 예상 기대결과 | 검증 방법(체크리스트) | 리스크/트레이드오프 | 롤백 |
|---|---|---|---|---|---|---|---|---|
| YYYY-MM-DD | `amcl.ros__parameters.<param>` |  |  |  |  |  |  |  |

#### Planner Server (Global Planner)

##### 변경 로그 테이블 (Planner Server)

| 날짜 | 파라미터(경로) | Before | After | 변경 이유(관찰/사용자 발화) | 예상 기대결과 | 검증 방법(체크리스트) | 리스크/트레이드오프 | 롤백 |
|---|---|---|---|---|---|---|---|---|
| 2025-12-17 | `planner_server.ros__parameters.GridBased.plugin` | `nav2_smac_planner/SmacPlannerHybrid` | `nav2_navfn_planner/NavFnPlanner` | Go2(제자리 회전 가능) + Jetson(연산 자원 제한) 환경에서, Ackermann/차량형 전제의 Hybrid planner는 과하고 무거움. 단순/대칭 맵에서도 빠르게 안정적 경로가 필요 | planner CPU 부담 감소, 기본 그리드 기반 최단 경로로 **경로 생성 안정화/응답성 개선** | `ComputePathToPose` 응답 시간 비교, CPU 사용률 확인, 동일 start/goal 반복 시 경로 일관성 확인 | NavFn은 기본 2D 그리드 플래너라 복잡한 모션 제약(차량형, 최소 회전 반경 등) 반영이 약함 | `SmacPlannerHybrid` 설정 블록으로 복귀 |
| 2025-12-17 | `planner_server.ros__parameters.GridBased.allow_unknown` / `use_astar` / `tolerance` | `false` / (N/A) / `3.0` | `true` / `false(Dijkstra)` / `0.5` | “회색(Unknown) 영역도 지나가고 싶다” 요구 반영 + 단순/대칭 맵에서 경로가 이상해 보이는 문제를 줄이기 위해, Unknown 통과 허용 및 Dijkstra로 최단경로 보장, 목표점 주변 여유로 실패율 감소 | Unknown 영역 경로 생성 가능(조건 충족 시), 대칭 구조에서 경로 일관성↑, goal 근처 플래닝 실패↓ | RViz에서 Unknown(회색) 관통 경로 생성 여부 확인, 실패 로그 빈도 비교, `global_costmap.track_unknown_space` 설정과 함께 점검 | Unknown을 free로 취급하면 실제 장애물/낙차 영역 위험. `tolerance`가 크면 정확한 목표 도착이 흐려질 수 있음 | `allow_unknown: false`로 복귀, `use_astar`/`tolerance` 이전 값으로 복귀 |
| 2025-12-17 | `planner_server.ros__parameters.use_sim_time` | `True` | `False` | 실제 Go2 + Jetson 운용(실시간)에서 시뮬레이션 시간이 아닌 시스템 시간을 사용하도록 일관성 맞춤 | TF/센서 timestamp 불일치로 인한 nav2 경고/지연 감소 기대 | `/clock` 사용 여부 확인, nav2 로그에서 time 관련 warning 확인 | 시뮬 환경에서는 `/clock` 기반 노드들과 시간 불일치 가능 | 시뮬에서는 `True`로 복귀 |

##### 변경 기록 템플릿 (Planner Server)

| 날짜 | 파라미터(경로) | Before | After | 변경 이유(관찰/사용자 발화) | 예상 기대결과 | 검증 방법(체크리스트) | 리스크/트레이드오프 | 롤백 |
|---|---|---|---|---|---|---|---|---|
| YYYY-MM-DD | `planner_server.ros__parameters.<param>` |  |  |  |  |  |  |  |

#### Controller Server (Local Planner / DWB)

##### Controller Server 튜닝 전(원본) 값 요약

```yaml
controller_server:
  ros__parameters:
    controller_frequency: 3.0
    FollowPath:
      max_vel_x: 3.0
      max_vel_theta: 3.0
      max_speed_xy: 3.0
      acc_lim_x: 2.5
      acc_lim_y: 2.5
      acc_lim_theta: 3.2
      decel_lim_x: -2.5
      decel_lim_y: -2.5
      decel_lim_theta: -3.2
      sim_time: 1.2
      xy_goal_tolerance: 0.3
```

##### 변경 로그 테이블 (Controller Server)

| 날짜 | 파라미터(경로) | Before | After | 변경 이유(관찰/사용자 발화) | 예상 기대결과 | 검증 방법(체크리스트) | 리스크/트레이드오프 | 롤백 |
|---|---|---|---|---|---|---|---|---|
| 2025-12-17 | `controller_server.ros__parameters.controller_frequency` | `3.0` | `10.0` | Jetson에서 처리가 가능한 범위에서 제어 루프를 빠르게 돌려 **실시간 장애물 회피/경로 추종 반응성** 확보 | 장애물 등장/코너링에서 반응 지연 감소, local planner 흔들림 감소 | 장애물 급출현 상황에서 회피 성공률 확인, `cmd_vel` 갱신 주기/지연 확인, CPU 사용률 확인 | 주파수 증가로 CPU 부하↑ (환경에 따라) | `controller_frequency: 3.0`로 복귀 |
| 2025-12-17 | `controller_server.ros__parameters.FollowPath.max_vel_x` / `max_speed_xy` | `3.0` / `3.0` | `0.8` / `0.8` | 실내에서 속도가 너무 높으면 위험 + slip 증가로 odom 신뢰도↓ → **특징 없는 맵에서 AMCL 튐** 악화 가능 | 안전 속도 확보, slip 감소로 odom/AMCL 안정성↑ | 동일 경로 주행 시 slip/pose jump 빈도 비교, 목표 도달 안정성 확인 | 속도가 낮아져 이동 시간이 증가, 장애물 회피가 느리게 느껴질 수 있음 | `max_vel_x: 3.0`, `max_speed_xy: 3.0`로 복귀 |
| 2025-12-17 | `controller_server.ros__parameters.FollowPath.max_vel_theta` | `3.0` | `1.0` | 빠른 회전은 스캔 변화가 급격해 **AMCL yaw delocalization** 유발 가능 + 실내에서 회전이 과격 | 회전 시 pose 안정성↑, 제자리 회전 품질↑ | 제자리 회전/코너 회전 시 `amcl_pose` 흔들림 및 회전 실패 여부 확인 | 회전이 느려져 좁은 공간에서 회피 동작이 답답할 수 있음 | `max_vel_theta: 3.0`로 복귀 |
| 2025-12-17 | `controller_server.ros__parameters.FollowPath.acc_lim_x` / `acc_lim_theta` (+ decel) | `2.5` / `3.2` | `1.5` / `2.0` | 급가속/급감속은 slip/진동을 만들고 odom/IMU 안정성을 해침 → **대칭/특징 없는 환경에서 localization 불안정** | 가감속이 부드러워져 자세/odom 안정성↑, local planner 제어 안정화 | 급정지/출발 시 미끄러짐 감소 확인, IMU/odom 노이즈 체감 및 pose jump 빈도 비교 | 너무 낮으면 가속이 답답하고 회피가 느릴 수 있음 | 기존 값으로 복귀 (`acc_lim_x: 2.5`, `acc_lim_theta: 3.2`, decel도 원복) |
| 2025-12-17 | `controller_server.ros__parameters.FollowPath.sim_time` | `1.2` | `1.5` | 가까운 미래만 보면 장애물을 늦게 인지해 진동/충돌 위험. 예측 길이를 늘려 더 일찍 회피 곡선을 생성 | 장애물 회피가 미리 시작되고 local costmap과의 충돌/진동 감소 | 좁은 통로에서 oscillation 감소 확인, 충돌 직전 급회피 빈도 감소 확인 | 예측이 길어져 보수적 회피/우회가 늘 수 있음 | `sim_time: 1.2`로 복귀 |
| 2025-12-17 | `controller_server.ros__parameters.FollowPath.xy_goal_tolerance` | `0.3` | `0.25` | goal 근처에서 과도하게 넓은 허용 반경은 “도착”이 빨라 보이지만 위치 정밀도가 떨어질 수 있음 | goal 근처 도착 정밀도↑ | goal 도착 후 실제 위치 오차 확인, oscillation 여부 확인 | 너무 낮으면 도착 직전 미세 조정이 길어질 수 있음 | `xy_goal_tolerance: 0.3`로 복귀 |

##### 변경 기록 템플릿 (Controller Server)

| 날짜 | 파라미터(경로) | Before | After | 변경 이유(관찰/사용자 발화) | 예상 기대결과 | 검증 방법(체크리스트) | 리스크/트레이드오프 | 롤백 |
|---|---|---|---|---|---|---|---|---|
| YYYY-MM-DD | `controller_server.ros__parameters.<param>` |  |  |  |  |  |  |  |

#### Local Costmap

##### Local Costmap 튜닝 전(원본) 값 요약

```yaml
local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 3.0
      publish_frequency: 3.0
      width: 6
      height: 6
      resolution: 0.05
      plugins: ["static_layer", "voxel_layer", "inflation_layer"]
      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_subscribe_transient_local: True
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 1.0
        inflation_radius: 0.25
```

##### 변경 로그 테이블 (Local Costmap)

| 날짜 | 파라미터(경로) | Before | After | 변경 이유(관찰/사용자 발화) | 예상 기대결과 | 검증 방법(체크리스트) | 리스크/트레이드오프 | 롤백 |
|---|---|---|---|---|---|---|---|---|
| 2025-12-18 | `local_costmap.local_costmap.ros__parameters.plugins` / `static_layer` | `["static_layer", "voxel_layer", "inflation_layer"]` | `["voxel_layer", "inflation_layer"]` | “회색(Unknown) 영역도 지나가고 싶다” 요구에서, Local costmap이 map 기반(static)을 강하게 반영하면 Global path를 받았어도 local에서 주저/정지할 수 있음. 로컬은 **센서 기반(Voxel/Obstacle)만 믿고** 회피/추종하도록 단순화 | Unknown/회색 영역에서도 실제 장애물이 없으면 주행 지속, local planner의 불필요한 stop 감소 | RViz에서 local costmap 레이어 구성 확인, 회색 영역에서 로봇이 멈추는지/주저하는지 비교 | 센서가 못 보는 벽/유리/맵에만 있는 장애물은 로컬에서 반영이 약해질 수 있음(센서 품질/환경 의존) | plugins를 원래대로 복귀(`static_layer` 재추가) |
| 2025-12-18 | `local_costmap...update_frequency` / `publish_frequency` | `3.0` / `3.0` | `5.0` / `2.0` | Jetson이 감당 가능한 범위에서 local 장애물 반영을 더 자주(update) 하되, 시각화/외부 송신(publish)은 과도하게 올리지 않음 | 장애물 회피 반응성↑, CPU/네트워크 부담은 상대적으로 완화 | 장애물 급출현에서 회피 성공률 확인, local costmap 업데이트/지연 확인, CPU 사용률 확인 | update 증가로 CPU 부하↑ (환경에 따라) | `update_frequency: 3.0`, `publish_frequency: 3.0`로 복귀 |
| 2025-12-18 | `local_costmap...width` / `height` | `6` / `6` | `5` / `5` | update_frequency를 올린 만큼 연산량을 상쇄하기 위해 로컬 맵 범위를 축소(필수 근거리만) | 주기↑를 유지하면서도 CPU 부담 완화 | 동일 환경에서 CPU 사용률 비교, 회피 성능 저하 여부 확인 | 너무 작으면 멀리 있는 장애물을 늦게 인지해 급회피/진동 가능 | `width: 6`, `height: 6`으로 복귀 |
| 2025-12-18 | `local_costmap...inflation_layer.cost_scaling_factor` / `inflation_radius` | `1.0` / `0.25` | `3.0` / `0.45` | Go2 크기/실내 환경에서 너무 타이트한 inflation은 충돌 위험↑. 반면 지나치게 넓은 비용 확산은 좁은 곳 통과 실패를 부를 수 있어 tradeoff로 조정 | 장애물/벽과의 최소 안전거리 확보, 좁은 통로에서 주행 가능성 유지 | 벽/장애물 근처 주행 시 스치지 않는지 확인, 좁은 통로 통과 실패 여부 확인 | inflation이 과하면 통로를 “막힌 것”으로 판단할 수 있음 | `cost_scaling_factor: 1.0`, `inflation_radius: 0.25`로 복귀 |

##### 변경 기록 템플릿 (Local Costmap)

| 날짜 | 파라미터(경로) | Before | After | 변경 이유(관찰/사용자 발화) | 예상 기대결과 | 검증 방법(체크리스트) | 리스크/트레이드오프 | 롤백 |
|---|---|---|---|---|---|---|---|---|
| YYYY-MM-DD | `local_costmap.local_costmap.ros__parameters.<param>` |  |  |  |  |  |  |  |

#### Global Costmap

##### Global Costmap 튜닝 전(원본) 값 요약

```yaml
global_costmap:
  global_costmap:
    ros__parameters:
      resolution: 0.2
      track_unknown_space: False
      plugins: ["static_layer", "voxel_layer", "inflation_layer"]
      voxel_layer:
        plugin: "nav2_costmap_2d::VoxelLayer"
        enabled: True
        publish_voxel_map: True
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 1.0
        inflation_radius: 0.25
```

##### 변경 로그 테이블 (Global Costmap)

| 날짜 | 파라미터(경로) | Before | After | 변경 이유(관찰/사용자 발화) | 예상 기대결과 | 검증 방법(체크리스트) | 리스크/트레이드오프 | 롤백 |
|---|---|---|---|---|---|---|---|---|
| 2025-12-18 | `global_costmap.global_costmap.ros__parameters.track_unknown_space` | `False` | `True` | “회색(Unknown) 영역도 지나가고 싶다” 요구를 `planner_server.allow_unknown: true`와 짝으로 동작시키기 위해 Unknown을 costmap에서 유지 | Unknown(회색) 영역을 포함한 전역 경로 생성 가능(조건 충족 시) | RViz에서 global costmap에 Unknown이 유지되는지 확인, `allow_unknown` 켠 상태에서 Unknown 관통 경로 생성 확인 | Unknown을 free처럼 다루면 실제 장애물/낙차 등 위험(운용 환경 주의) | `track_unknown_space: False`로 복귀 |
| 2025-12-18 | `global_costmap...plugins` / `obstacle_layer` / `voxel_layer` | `voxel_layer` 사용 | `obstacle_layer` 사용(2D) + `voxel_layer` 주석 | Go2가 주로 2D `scan` 기반이면 3D voxel 처리가 Jetson에 불필요하게 무거움. 전역 경로에도 동적 장애물을 반영하기 위해 `obstacle_layer`를 사용 | Jetson 부하 감소, 맵에 없던 장애물 등장 시 전역 경로가 우회하도록 반영 | 장애물(박스/사람) 추가 시 global path가 우회하는지 확인, CPU 사용률 확인 | 2D 스캔만으로는 높이/머리 위 장애물 표현 한계. 잘못된 marking/clearing 튜닝 시 costmap 노이즈 가능 | plugins를 원래대로 복귀(`voxel_layer` 활성), `obstacle_layer` 제거 |
| 2025-12-18 | `global_costmap...resolution` | `0.2` | `0.05` | 좁은 공간/정교한 경로 필요 + 거친 격자에서는 통로가 막히거나 장애물이 뭉개져 보임 | 더 정교한 전역 경로(좁은 통로 인식/중앙 주행 유도 개선) | 같은 start/goal에서 경로 품질 비교, costmap/plan 생성 시간 확인 | 해상도↑로 메모리/CPU 증가(환경에 따라) | `resolution: 0.2`로 복귀 |
| 2025-12-18 | `global_costmap...inflation_layer.cost_scaling_factor` / `inflation_radius` | `1.0` / `0.25` | `3.0` / `0.55` | 벽에 너무 붙는 주행은 특징 없는 맵에서 라이다 매칭/자세 추정이 불리. 벽에서 떨어져 중앙 주행 유도 + 안전거리 확보 | 벽과의 거리 확보, 중앙 주행 경향 강화, 좁은 곳에서 충돌 위험 감소 | 벽 가까운 구간에서 경로가 중앙으로 이동하는지 확인, 통로 통과 실패 여부 확인 | inflation이 과하면 통로를 “막힌 것”으로 판단할 수 있음 | `cost_scaling_factor: 1.0`, `inflation_radius: 0.25`로 복귀 |

##### 변경 기록 템플릿 (Global Costmap)

| 날짜 | 파라미터(경로) | Before | After | 변경 이유(관찰/사용자 발화) | 예상 기대결과 | 검증 방법(체크리스트) | 리스크/트레이드오프 | 롤백 |
|---|---|---|---|---|---|---|---|---|
| YYYY-MM-DD | `global_costmap.global_costmap.ros__parameters.<param>` |  |  |  |  |  |  |  |

---

**AMCL (Adaptive Monte Carlo Localization)**은 로봇이 자신의 위치를 추정하는 확률적 알고리즘입니다. 쉽게 말해, **"지도(Map)와 현재 라이다 센서(Scan) 데이터, 그리고 로봇이 움직인 거리(Odom)를 비교해서 내가 어디에 있는지 찍어 맞추는 역할"**을 합니다.

사용자님이 말씀하신 **"특징 없는 정사각형 맵에서 위치가 튀는 현상"**을 해결하기 위해 AMCL 파라미터를 수정했습니다. 구체적으로 무엇을 바꿨고, 사용자의 어떤 말 때문에 그렇게 수정했는지 설명해 드리겠습니다.

---

### 1. `alpha1` ~ `alpha4` (오도메트리 노이즈 모델)

*   **변경 내용:** `0.2` (기본값) → **`0.05` (매우 낮춤)**
*   **사용자의 어떤 말 때문에 바꿨나?**
    > "특징 없는 정사각형 맵이면 각 모서리로 가도, 계속해서 다른 쪽 모서리로 이동한 것처럼 나오는 것 같아."
*   **이유 (핵심):**
    *   **`alpha` 값이 크면:** 로봇은 자신의 다리(Odometry)보다 **눈(Lidar)을 더 신뢰**합니다. 정사각형 방에서는 모든 모서리가 똑같이 생겼기 때문에, 라이다만 믿으면 "어? 저쪽 모서리랑 모양이 똑같네? 저기인가보다!" 하고 위치를 순간이동 시켜버립니다.
    *   **`alpha` 값을 줄이면:** "눈에 보이는 건 헷갈리지만, **내 다리(엔코더/IMU)는 분명히 이만큼만 움직였어. 그러니까 저쪽 모서리로 순간이동할 리가 없어.**"라고 판단하게 됩니다.
    *   **결과:** Unitree Go2의 내부 센서 성능을 믿고 위치 튀는 현상을 억제합니다.

### 2. `max_particles` (최대 입자 수)

*   **변경 내용:** `2000` → **`3000` (늘림)**
*   **사용자의 어떤 말 때문에 바꿨나?**
    > "특징 없는 맵... 튀는 현상..."
*   **이유:**
    *   AMCL은 수많은 점(Particle)을 지도 위에 뿌려서 로봇의 위치를 추측합니다.
    *   맵이 특징이 뚜렷하면 적은 수의 입자로도 위치를 잘 잡지만, **특징이 없는 대칭형 맵**에서는 확률이 여러 군데로 분산됩니다.
    *   입자 수를 늘리면 **"진짜 위치"에 살아남아 있는 입자가 있을 확률**을 높여줍니다. Nvidia Jetson은 3000개 정도의 파티클은 충분히 계산할 수 있습니다.

### 3. `update_min_d`, `update_min_a` (업데이트 주기)

*   **설정 내용:** `0.2` (0.2m 이동하거나 0.2rad 회전했을 때만 위치 갱신)
*   **사용자의 어떤 말 때문에 바꿨나?**
    > "준실시간 반영이 되면 좋겠어" (하지만 너무 민감하면 안 됨)
*   **이유:**
    *   이 값이 너무 작으면(예: 0.05), 로봇이 제자리에 있어도 센서 노이즈 때문에 위치가 계속 흔들립니다(Jittering).
    *   특히 대칭형 맵에서는 가만히 있는데 갑자기 맵이 돌아가는 현상이 생길 수 있습니다. 이를 방지하기 위해 **"확실히 움직였을 때만 위치를 계산해라"**라고 설정한 것입니다.

---

### **요약: 변경 전 vs 변경 후 로봇의 생각**

*   **변경 전 (`alpha: 0.2`):**
    🤖: *"내 다리는 조금 움직였다고 하는데, 눈(Lidar)으로 보니까 저쪽 벽이랑 모양이 완전 똑같네? 에라 모르겠다 저쪽으로 점프!"* (위치 튐 발생)

*   **변경 후 (`alpha: 0.05`):**
    🤖: *"눈(Lidar)으로 보니까 저쪽 벽이랑 비슷해 보이긴 하는데... 내 다리(Odom)가 분명히 아직 여기라고 말하고 있어. 비슷해 보일 뿐이지 난 여기 있는 게 맞아."* (위치 유지)