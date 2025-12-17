# Nav2 Parameters (`go2_robot_sdk/config/nav2_params.yaml`)

이 문서는 `nav2_params.yaml`의 변경을 **왜 했는지(문제/가설)**, **무엇을 바꿨는지(파라미터/Before/After)**, 그리고 **예상 기대결과/검증 방법/롤백 방법**까지 같이 기록하기 위한 로그입니다.

## Reference
[관련 문서 프롬프트](https://aistudio.google.com/app/prompts/13bQ00apmpNhlRA3CXkmPfvtdOs3NY-KP)

---

## 변경 로그

### AMCL (Localization)

**AMCL 튜닝 전(원본) 값 요약**

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

**변경 로그 테이블 (AMCL)**

| 날짜 | 파라미터(경로) | Before | After | 변경 이유(관찰/사용자 발화) | 예상 기대결과 | 검증 방법(체크리스트) | 리스크/트레이드오프 | 롤백 |
|---|---|---|---|---|---|---|---|---|
| 2025-12-17 | `amcl.ros__parameters.alpha1~4` | `0.2` | `0.05` | 특징 없는 정사각형 맵에서 corner 대칭으로 라이다 매칭이 헷갈려 **pose가 다른 corner로 점프**하는 현상 | 오도메트리를 더 신뢰해서 **대칭 환경에서 순간이동/튐 감소** | RViz에서 `amcl_pose` / `tf(map->odom)` 점프 여부 확인, 같은 루트로 반복 주행 시 재현성 확인 | 라이다 기반 보정이 약해져 장거리 누적 drift가 커질 수 있음(환경에 따라) | `alpha1~4`를 `0.2`로 복귀 |
| 2025-12-17 | `amcl.ros__parameters.max_particles` | `2000` | `3000` | 대칭 환경에서 후보 위치가 여러 곳으로 분산되므로 particle 다양성 확보 필요 | ambiguity 상황에서 **진짜 위치 particle 생존 확률 증가**, 튐 감소 | CPU 사용률/AMCL 업데이트 주기 확인, pose 튐 빈도 비교 | CPU/메모리 사용 증가(저사양에서는 latency 증가 가능) | `max_particles: 2000`으로 복귀 |
| 2025-12-17 | `amcl.ros__parameters.update_min_d`, `update_min_a` | `0.25`, `0.2` | `0.2`, `0.2` | 너무 자주 업데이트하면 노이즈가 누적되어 jitter/맵 회전처럼 보이는 현상 유발 가능 | **정지/미세 움직임에서 jitter 감소**, 안정적인 pose | 제자리/저속 이동에서 pose 흔들림 감소 확인, tracking 안정성 확인 | 반응성이 떨어져 빠른 움직임에서 pose 갱신이 늦어 보일 수 있음 | `update_min_d: 0.25`, `update_min_a: 0.2`로 복귀 |

**변경 기록 템플릿 (AMCL)**

| 날짜 | 파라미터(경로) | Before | After | 변경 이유(관찰/사용자 발화) | 예상 기대결과 | 검증 방법(체크리스트) | 리스크/트레이드오프 | 롤백 |
|---|---|---|---|---|---|---|---|---|
| YYYY-MM-DD | `amcl.ros__parameters.<param>` |  |  |  |  |  |  |  |

### Planner Server (Global Planner)

**변경 로그 테이블 (Planner Server)**

| 날짜 | 파라미터(경로) | Before | After | 변경 이유(관찰/사용자 발화) | 예상 기대결과 | 검증 방법(체크리스트) | 리스크/트레이드오프 | 롤백 |
|---|---|---|---|---|---|---|---|---|
| 2025-12-17 | `planner_server.ros__parameters.GridBased.plugin` | `nav2_smac_planner/SmacPlannerHybrid` | `nav2_navfn_planner/NavFnPlanner` | Go2(제자리 회전 가능) + Jetson(연산 자원 제한) 환경에서, Ackermann/차량형 전제의 Hybrid planner는 과하고 무거움. 단순/대칭 맵에서도 빠르게 안정적 경로가 필요 | planner CPU 부담 감소, 기본 그리드 기반 최단 경로로 **경로 생성 안정화/응답성 개선** | `ComputePathToPose` 응답 시간 비교, CPU 사용률 확인, 동일 start/goal 반복 시 경로 일관성 확인 | NavFn은 기본 2D 그리드 플래너라 복잡한 모션 제약(차량형, 최소 회전 반경 등) 반영이 약함 | `SmacPlannerHybrid` 설정 블록으로 복귀 |
| 2025-12-17 | `planner_server.ros__parameters.GridBased.allow_unknown` / `use_astar` / `tolerance` | `false` / (N/A) / `3.0` | `true` / `false(Dijkstra)` / `0.5` | “회색(Unknown) 영역도 지나가고 싶다” 요구 반영 + 단순/대칭 맵에서 경로가 이상해 보이는 문제를 줄이기 위해, Unknown 통과 허용 및 Dijkstra로 최단경로 보장, 목표점 주변 여유로 실패율 감소 | Unknown 영역 경로 생성 가능(조건 충족 시), 대칭 구조에서 경로 일관성↑, goal 근처 플래닝 실패↓ | RViz에서 Unknown(회색) 관통 경로 생성 여부 확인, 실패 로그 빈도 비교, `global_costmap.track_unknown_space` 설정과 함께 점검 | Unknown을 free로 취급하면 실제 장애물/낙차 영역 위험. `tolerance`가 크면 정확한 목표 도착이 흐려질 수 있음 | `allow_unknown: false`로 복귀, `use_astar`/`tolerance` 이전 값으로 복귀 |
| 2025-12-17 | `planner_server.ros__parameters.use_sim_time` | `True` | `False` | 실제 Go2 + Jetson 운용(실시간)에서 시뮬레이션 시간이 아닌 시스템 시간을 사용하도록 일관성 맞춤 | TF/센서 timestamp 불일치로 인한 nav2 경고/지연 감소 기대 | `/clock` 사용 여부 확인, nav2 로그에서 time 관련 warning 확인 | 시뮬 환경에서는 `/clock` 기반 노드들과 시간 불일치 가능 | 시뮬에서는 `True`로 복귀 |
| 2025-12-18 | `planner_server.ros__parameters.GridBased.use_astar` | `false` | `true` | Dijkstra 대비 평균적으로 탐색 범위를 줄여 플래닝 시간/CPU를 낮추기 위해 A*로 전환(목표 방향 우선 탐색). Jetson에서 planner 여유를 확보 | 동일 환경에서 플래닝 응답성↑, CPU 사용률↓(케이스에 따라) | 동일 start/goal 반복 시 `ComputePathToPose` 응답시간 비교, 장애물 배치가 복잡한 환경에서 실패율/응답시간 비교 | 최악 케이스에서는 성능 차이가 작을 수 있음. 경로가 격자 형태로 보일 수 있음 | `use_astar: false`로 복귀 |

**참고: Dijkstra vs A* 설명 정확성(Planner 관점)**

- **Dijkstra (NavFn `use_astar: false`)**
  - 가중치가 양수인 그래프에서 **최단 경로를 보장**합니다.
  - 격자(costmap)에서는 목표 방향을 “우선”하지 않고 **원점에서 비용이 낮은 영역부터 넓게 확장**하는 형태로 보입니다(동심원처럼 보일 수 있음).
  - 연산량은 맵이 커질수록 커지지만, 일반적으로 표현을 엄밀히 하면 **"기하급수적"이라기보다는 격자 셀 수(면적)에 비례해 증가**합니다(최악의 경우 A*도 비슷).

- **A* (NavFn `use_astar: true`)**
  - 목표까지의 휴리스틱을 사용하여 **목표 방향을 우선 탐색**하므로, 많은 케이스에서 Dijkstra보다 **확장하는 노드(셀) 수가 크게 줄어** 더 빠르게 끝납니다.
  - 다만 **최악의 경우(휴리스틱이 거의 도움이 안 되거나 장애물로 우회가 큰 경우)** A*도 Dijkstra에 가까운 연산량이 나올 수 있습니다.

**플래너 알고리즘/튜닝 후보 (미적용 가이드)**

아래 내용은 **"현재 파일에 적용된 변경 로그"가 아니라**, 경로 planning 성능/효율(특히 Jetson) 관점에서 **추가로 고려할 후보**를 정리한 가이드입니다.

| 후보 | 무엇을 바꾸나(파라미터/블록) | 기대 효과 | 검증 방법(체크리스트) | 리스크/주의 | 롤백 |
|---|---|---|---|---|---|
| NavFn에서 A* 켜기(저리스크) | `planner_server.ros__parameters.GridBased.use_astar: false -> true` | 대다수 상황에서 플래닝 시간/CPU 감소(목표 방향 우선 탐색) | 동일 start/goal 반복 시 `ComputePathToPose` 응답시간/CPU 비교, 좁은 통로/장애물 배치에서 성공률 비교 | 최악 케이스에서는 차이가 작을 수 있음. 경로가 격자 형태로 각질게 보일 수 있음 | `use_astar: false`로 복귀 |
| SmacPlanner2D 적용(고성능 후보) | `planner_server.ros__parameters.GridBased.plugin`을 `nav2_smac_planner/SmacPlanner2D`로 교체 + Smac 파라미터 추가 | NavFn 대비 성능/경로 품질 개선 가능(환경에 따라), cost 반영이 더 정교 | 플래닝 시간/CPU 비교, 경로가 벽에서 너무 붙지 않는지 확인, Unknown 통과 정책(`allow_unknown` + `global_costmap.track_unknown_space`) 일관성 확인 | 파라미터가 늘어 튜닝 포인트 증가. 패키지/플러그인 존재 여부(빌드/설치) 확인 필요 | plugin을 `nav2_navfn_planner/NavFnPlanner`로 복귀 |
| Theta*(Any-angle) 적용(경로 직선화 후보) | `planner_server...GridBased.plugin`을 Theta* 플래너로 교체(패키지/플러그인 확인 필요) | 대각선/직선 경로로 경로가 시각적으로 더 “사람처럼” 나올 수 있음 | 좁은 통로에서 벽 스침/통과 실패 여부 확인, `inflation_radius`/`cost_scaling_factor` 조합에서 안전거리 확보 확인 | 장애물 모서리를 아슬아슬하게 스치는 경향이 있을 수 있어 costmap inflation 튜닝 의존도가 큼 | 기존 플래너로 복귀 |

**(미적용) 변경 후보 스니펫 예시**

NavFn A* 전환 시 변경 포인트는 아래 1줄입니다.

```yaml
planner_server:
  ros__parameters:
    GridBased:
      plugin: "nav2_navfn_planner/NavFnPlanner"
      use_astar: true
```

SmacPlanner2D 적용 시에는 아래처럼 `GridBased` 플러그인을 교체하고(필수), 나머지는 환경/버전에 맞춰 조정합니다. 아래는 **예시**이며, Nav2 배포판/플러그인 버전에 따라 지원 파라미터가 다를 수 있으니 적용 전 파라미터 인식 여부를 확인합니다.

```yaml
planner_server:
  ros__parameters:
    expected_planner_frequency: 5.0
    use_sim_time: False
    planner_plugins: ["GridBased"]

    GridBased:
      plugin: "nav2_smac_planner/SmacPlanner2D"

      tolerance: 0.5
      downsample_costmap: false
      allow_unknown: true

      motion_model_for_search: "MOORE"
      angle_quantization_bins: 1

      cost_travel_multiplier: 1.0

      smooth_path: true
      smoother:
        max_iterations: 1000
        w_smooth: 0.3
        w_data: 0.2
        tolerance: 1e-10
```

SmacPlanner2D/Theta*로 넘어갈 경우에는 플래너 교체뿐 아니라, 환경에 따라 아래 항목들이 함께 영향을 줍니다.

- `global_costmap.*.resolution` (해상도↑: 경로 정교↑ / CPU·메모리↑)
- `global_costmap.*.inflation_layer.*` (안전거리/중앙 주행 성향)
- `planner_server.*.allow_unknown` 과 `global_costmap.*.track_unknown_space`의 짝 맞춤

**변경 기록 템플릿 (Planner Server)**

| 날짜 | 파라미터(경로) | Before | After | 변경 이유(관찰/사용자 발화) | 예상 기대결과 | 검증 방법(체크리스트) | 리스크/트레이드오프 | 롤백 |
|---|---|---|---|---|---|---|---|---|
| YYYY-MM-DD | `planner_server.ros__parameters.<param>` |  |  |  |  |  |  |  |

### Controller Server (Local Planner / DWB)

**Controller Server 튜닝 전(원본) 값 요약**

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

**변경 로그 테이블 (Controller Server)**

| 날짜 | 파라미터(경로) | Before | After | 변경 이유(관찰/사용자 발화) | 예상 기대결과 | 검증 방법(체크리스트) | 리스크/트레이드오프 | 롤백 |
|---|---|---|---|---|---|---|---|---|
| 2025-12-17 | `controller_server.ros__parameters.controller_frequency` | `3.0` | `10.0` | Jetson에서 처리가 가능한 범위에서 제어 루프를 빠르게 돌려 **실시간 장애물 회피/경로 추종 반응성** 확보 | 장애물 등장/코너링에서 반응 지연 감소, local planner 흔들림 감소 | 장애물 급출현 상황에서 회피 성공률 확인, `cmd_vel` 갱신 주기/지연 확인, CPU 사용률 확인 | 주파수 증가로 CPU 부하↑ (환경에 따라) | `controller_frequency: 3.0`로 복귀 |
| 2025-12-17 | `controller_server.ros__parameters.FollowPath.max_vel_x` / `max_speed_xy` | `3.0` / `3.0` | `0.8` / `0.8` | 실내에서 속도가 너무 높으면 위험 + slip 증가로 odom 신뢰도↓ → **특징 없는 맵에서 AMCL 튐** 악화 가능 | 안전 속도 확보, slip 감소로 odom/AMCL 안정성↑ | 동일 경로 주행 시 slip/pose jump 빈도 비교, 목표 도달 안정성 확인 | 속도가 낮아져 이동 시간이 증가, 장애물 회피가 느리게 느껴질 수 있음 | `max_vel_x: 3.0`, `max_speed_xy: 3.0`로 복귀 |
| 2025-12-17 | `controller_server.ros__parameters.FollowPath.max_vel_theta` | `3.0` | `1.0` | 빠른 회전은 스캔 변화가 급격해 **AMCL yaw delocalization** 유발 가능 + 실내에서 회전이 과격 | 회전 시 pose 안정성↑, 제자리 회전 품질↑ | 제자리 회전/코너 회전 시 `amcl_pose` 흔들림 및 회전 실패 여부 확인 | 회전이 느려져 좁은 공간에서 회피 동작이 답답할 수 있음 | `max_vel_theta: 3.0`로 복귀 |
| 2025-12-17 | `controller_server.ros__parameters.FollowPath.acc_lim_x` / `acc_lim_theta` (+ decel) | `2.5` / `3.2` | `1.5` / `2.0` | 급가속/급감속은 slip/진동을 만들고 odom/IMU 안정성을 해침 → **대칭/특징 없는 환경에서 localization 불안정** | 가감속이 부드러워져 자세/odom 안정성↑, local planner 제어 안정화 | 급정지/출발 시 미끄러짐 감소 확인, IMU/odom 노이즈 체감 및 pose jump 빈도 비교 | 너무 낮으면 가속이 답답하고 회피가 느릴 수 있음 | 기존 값으로 복귀 (`acc_lim_x: 2.5`, `acc_lim_theta: 3.2`, decel도 원복) |
| 2025-12-17 | `controller_server.ros__parameters.FollowPath.sim_time` | `1.2` | `1.5` | 가까운 미래만 보면 장애물을 늦게 인지해 진동/충돌 위험. 예측 길이를 늘려 더 일찍 회피 곡선을 생성 | 장애물 회피가 미리 시작되고 local costmap과의 충돌/진동 감소 | 좁은 통로에서 oscillation 감소 확인, 충돌 직전 급회피 빈도 감소 확인 | 예측이 길어져 보수적 회피/우회가 늘 수 있음 | `sim_time: 1.2`로 복귀 |
| 2025-12-17 | `controller_server.ros__parameters.FollowPath.xy_goal_tolerance` | `0.3` | `0.25` | goal 근처에서 과도하게 넓은 허용 반경은 “도착”이 빨라 보이지만 위치 정밀도가 떨어질 수 있음 | goal 근처 도착 정밀도↑ | goal 도착 후 실제 위치 오차 확인, oscillation 여부 확인 | 너무 낮으면 도착 직전 미세 조정이 길어질 수 있음 | `xy_goal_tolerance: 0.3`로 복귀 |

**속도 상향(미적용 가이드)**

아래는 **현재 적용된 변경 로그가 아니라**, “안전형(저속) 세팅에서 속도를 더 올리고 싶을 때” 참고하는 가이드입니다. `FollowPath (DWBLocalPlanner)`는 단순히 `max_vel_x`만 올리면 끝이 아니라, **속도/가속/예측시간/주파수**가 서로 맞아야 실제로 빨라지고(그리고 안전합니다).

| 그룹 | 무엇을 바꾸나(파라미터 경로) | 왜 같이 바꿔야 하나(팩트/메커니즘) | 변경 예시(범위) | 검증 방법(체크리스트) | 리스크/트레이드오프 | 롤백 |
|---|---|---|---|---|---|---|
| 속도 제한 | `controller_server...FollowPath.max_vel_x` / `max_vel_theta` / `max_speed_xy` (필수) | DWB는 후보 궤적을 샘플링할 때 이 속도 상한을 사용. 또한 `max_speed_xy`가 낮으면 `max_vel_x`를 올려도 **최종 속도 벡터가 `max_speed_xy`로 클램프**되어 실제 속도가 안 올라감 | `max_vel_x: 1.5~2.5` , `max_speed_xy: max_vel_x 이상`, `max_vel_theta: 2.0~3.0` | 직선 구간에서 실제 `cmd_vel`이 목표 속도까지 올라가는지 확인, 코너에서 과회전/미끄러짐 여부 확인 | 속도↑는 slip/충돌 위험↑, AMCL 튐 가능성↑ | 기존 저속값으로 복귀(`max_vel_x/max_speed_xy: 0.8`, `max_vel_theta: 1.0`) |
| 가속/감속 | `controller_server...FollowPath.acc_lim_x` / `decel_lim_x` (+ theta) | 최고속도를 올려도 가속도가 낮으면 짧은 구간에서 **목표 속도에 도달하기 전에 감속**하게 되어 체감 속도가 안 올라감. 또한 고속일수록 제동(감속) 여유가 없으면 충돌 위험↑ | `acc_lim_x: 2.0~5.0`, `decel_lim_x: -(acc_lim_x와 비슷하거나 더 큼)` , `acc_lim_theta/decel_lim_theta`도 함께 상향 고려 | 같은 경로에서 주행 시간 단축 확인, 급정지 시 미끄러짐/자세 흔들림/AMCL 튐 여부 확인 | 가속↑는 slip/IMU 노이즈↑로 localization 악화 가능, 실내에서는 위험 | 현재 값으로 복귀(`acc_lim_x: 1.5`, `decel_lim_x: -1.5`, theta도 원복) |
| 예측 시간 | `controller_server...FollowPath.sim_time` | 속도↑일수록 1초 동안 더 멀리 이동하므로, `sim_time`이 짧으면 **미래 충돌을 충분히 평가하지 못해** 늦게 피할 수 있음. 엄밀한 “공식식”이라기보다는, 최소한 `v * sim_time`이 정지/회피에 필요한 거리보다 충분히 크도록 잡는 게 실전적으로 안전 | `sim_time: 2.0~2.5` (속도/환경에 따라) | 장애물 급출현/코너에서 회피 시작 시점이 앞당겨지는지 확인, CPU 사용률 확인 | `sim_time`↑는 CPU↑ 및 보수적 회피/우회 증가 가능 | `sim_time: 1.5`로 복귀 |

**속도 상향 시 주파수/Costmap 권장 (조건부)**

속도만 올리고 `controller_frequency` / `local_costmap.update_frequency`가 낮으면, “한 번 판단/갱신 사이에 이동하는 거리”가 커져 위험해질 수 있습니다.

- 예: `1.5 m/s`에서 `controller_frequency: 10 Hz`면, 약 `0.15 m`마다 판단
- 예: `1.5 m/s`에서 `local_costmap.update_frequency: 5 Hz`면, 약 `0.30 m`마다 로컬 맵 갱신

Jetson이 버틴다면(모니터링 필요), 아래처럼 올리는 선택지가 있습니다.

- `controller_server.ros__parameters.controller_frequency: 20.0`
- `local_costmap.local_costmap.ros__parameters.update_frequency: 10.0`

단, 특히 `local_costmap`이 `voxel_layer`를 쓰는 경우 update 주파수 증가는 CPU에 민감하므로, 속도↑와 함께라면 **`vy_samples`/`vx_samples`/`vtheta_samples` 조정 등과 묶어서** 전체 부하를 같이 봐야 합니다.

**변경 기록 템플릿 (Controller Server)**

| 날짜 | 파라미터(경로) | Before | After | 변경 이유(관찰/사용자 발화) | 예상 기대결과 | 검증 방법(체크리스트) | 리스크/트레이드오프 | 롤백 |
|---|---|---|---|---|---|---|---|---|
| YYYY-MM-DD | `controller_server.ros__parameters.<param>` |  |  |  |  |  |  |  |

### Local Costmap

**Local Costmap 튜닝 전(원본) 값 요약**

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

**변경 로그 테이블 (Local Costmap)**

| 날짜 | 파라미터(경로) | Before | After | 변경 이유(관찰/사용자 발화) | 예상 기대결과 | 검증 방법(체크리스트) | 리스크/트레이드오프 | 롤백 |
|---|---|---|---|---|---|---|---|---|
| 2025-12-18 | `local_costmap.local_costmap.ros__parameters.plugins` / `static_layer` | `["static_layer", "voxel_layer", "inflation_layer"]` | `["voxel_layer", "inflation_layer"]` | “회색(Unknown) 영역도 지나가고 싶다” 요구에서, Local costmap이 map 기반(static)을 강하게 반영하면 Global path를 받았어도 local에서 주저/정지할 수 있음. 로컬은 **센서 기반(Voxel/Obstacle)만 믿고** 회피/추종하도록 단순화 | Unknown/회색 영역에서도 실제 장애물이 없으면 주행 지속, local planner의 불필요한 stop 감소 | RViz에서 local costmap 레이어 구성 확인, 회색 영역에서 로봇이 멈추는지/주저하는지 비교 | 센서가 못 보는 벽/유리/맵에만 있는 장애물은 로컬에서 반영이 약해질 수 있음(센서 품질/환경 의존) | plugins를 원래대로 복귀(`static_layer` 재추가) |
| 2025-12-18 | `local_costmap...update_frequency` / `publish_frequency` | `3.0` / `3.0` | `5.0` / `2.0` | Jetson이 감당 가능한 범위에서 local 장애물 반영을 더 자주(update) 하되, 시각화/외부 송신(publish)은 과도하게 올리지 않음 | 장애물 회피 반응성↑, CPU/네트워크 부담은 상대적으로 완화 | 장애물 급출현에서 회피 성공률 확인, local costmap 업데이트/지연 확인, CPU 사용률 확인 | update 증가로 CPU 부하↑ (환경에 따라) | `update_frequency: 3.0`, `publish_frequency: 3.0`로 복귀 |
| 2025-12-18 | `local_costmap...width` / `height` | `6` / `6` | `5` / `5` | update_frequency를 올린 만큼 연산량을 상쇄하기 위해 로컬 맵 범위를 축소(필수 근거리만) | 주기↑를 유지하면서도 CPU 부담 완화 | 동일 환경에서 CPU 사용률 비교, 회피 성능 저하 여부 확인 | 너무 작으면 멀리 있는 장애물을 늦게 인지해 급회피/진동 가능 | `width: 6`, `height: 6`으로 복귀 |
| 2025-12-18 | `local_costmap...inflation_layer.cost_scaling_factor` / `inflation_radius` | `1.0` / `0.25` | `3.0` / `0.45` | Go2 크기/실내 환경에서 너무 타이트한 inflation은 충돌 위험↑. 반면 지나치게 넓은 비용 확산은 좁은 곳 통과 실패를 부를 수 있어 tradeoff로 조정 | 장애물/벽과의 최소 안전거리 확보, 좁은 통로에서 주행 가능성 유지 | 벽/장애물 근처 주행 시 스치지 않는지 확인, 좁은 통로 통과 실패 여부 확인 | inflation이 과하면 통로를 “막힌 것”으로 판단할 수 있음 | `cost_scaling_factor: 1.0`, `inflation_radius: 0.25`로 복귀 |

**변경 기록 템플릿 (Local Costmap)**

| 날짜 | 파라미터(경로) | Before | After | 변경 이유(관찰/사용자 발화) | 예상 기대결과 | 검증 방법(체크리스트) | 리스크/트레이드오프 | 롤백 |
|---|---|---|---|---|---|---|---|---|
| YYYY-MM-DD | `local_costmap.local_costmap.ros__parameters.<param>` |  |  |  |  |  |  |  |

### Global Costmap

**Global Costmap 튜닝 전(원본) 값 요약**

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

**변경 로그 테이블 (Global Costmap)**

| 날짜 | 파라미터(경로) | Before | After | 변경 이유(관찰/사용자 발화) | 예상 기대결과 | 검증 방법(체크리스트) | 리스크/트레이드오프 | 롤백 |
|---|---|---|---|---|---|---|---|---|
| 2025-12-18 | `global_costmap.global_costmap.ros__parameters.track_unknown_space` | `False` | `True` | “회색(Unknown) 영역도 지나가고 싶다” 요구를 `planner_server.allow_unknown: true`와 짝으로 동작시키기 위해 Unknown을 costmap에서 유지 | Unknown(회색) 영역을 포함한 전역 경로 생성 가능(조건 충족 시) | RViz에서 global costmap에 Unknown이 유지되는지 확인, `allow_unknown` 켠 상태에서 Unknown 관통 경로 생성 확인 | Unknown을 free처럼 다루면 실제 장애물/낙차 등 위험(운용 환경 주의) | `track_unknown_space: False`로 복귀 |
| 2025-12-18 | `global_costmap...plugins` / `obstacle_layer` / `voxel_layer` | `voxel_layer` 사용 | `obstacle_layer` 사용(2D) + `voxel_layer` 주석 | Go2가 주로 2D `scan` 기반이면 3D voxel 처리가 Jetson에 불필요하게 무거움. 전역 경로에도 동적 장애물을 반영하기 위해 `obstacle_layer`를 사용 | Jetson 부하 감소, 맵에 없던 장애물 등장 시 전역 경로가 우회하도록 반영 | 장애물(박스/사람) 추가 시 global path가 우회하는지 확인, CPU 사용률 확인 | 2D 스캔만으로는 높이/머리 위 장애물 표현 한계. 잘못된 marking/clearing 튜닝 시 costmap 노이즈 가능 | plugins를 원래대로 복귀(`voxel_layer` 활성), `obstacle_layer` 제거 |
| 2025-12-18 | `global_costmap...resolution` | `0.2` | `0.05` | 좁은 공간/정교한 경로 필요 + 거친 격자에서는 통로가 막히거나 장애물이 뭉개져 보임 | 더 정교한 전역 경로(좁은 통로 인식/중앙 주행 유도 개선) | 같은 start/goal에서 경로 품질 비교, costmap/plan 생성 시간 확인 | 해상도↑로 메모리/CPU 증가(환경에 따라) | `resolution: 0.2`로 복귀 |
| 2025-12-18 | `global_costmap...inflation_layer.cost_scaling_factor` / `inflation_radius` | `1.0` / `0.25` | `3.0` / `0.55` | 벽에 너무 붙는 주행은 특징 없는 맵에서 라이다 매칭/자세 추정이 불리. 벽에서 떨어져 중앙 주행 유도 + 안전거리 확보 | 벽과의 거리 확보, 중앙 주행 경향 강화, 좁은 곳에서 충돌 위험 감소 | 벽 가까운 구간에서 경로가 중앙으로 이동하는지 확인, 통로 통과 실패 여부 확인 | inflation이 과하면 통로를 “막힌 것”으로 판단할 수 있음 | `cost_scaling_factor: 1.0`, `inflation_radius: 0.25`로 복귀 |

**변경 기록 템플릿 (Global Costmap)**

| 날짜 | 파라미터(경로) | Before | After | 변경 이유(관찰/사용자 발화) | 예상 기대결과 | 검증 방법(체크리스트) | 리스크/트레이드오프 | 롤백 |
|---|---|---|---|---|---|---|---|---|
| YYYY-MM-DD | `global_costmap.global_costmap.ros__parameters.<param>` |  |  |  |  |  |  |  |

### 장애물 회피/주행 이상 시 트러블슈팅 (미적용 가이드)

아래는 설정이 “틀렸다”기보다는, **실제 물리 환경/센서 특성/바닥 마찰/장애물 형태** 때문에 현장에서 자주 생기는 증상에 대한 조치 후보입니다. 적용 시에는 **한 번에 1~2개만** 바꾸고, 반드시 비교 실험(전/후)을 권장합니다.

| 증상 | 가능한 원인(팩트/메커니즘) | 우선 조치(파라미터 경로) | 권장 범위(예시) | 기대 효과 | 리스크/트레이드오프 | 롤백 |
|---|---|---|---|---|---|---|
| 장애물을 늦게 피함 / 너무 아슬아슬함 | DWB critic 가중치에서 **장애물 비용(BaseObstacle)** 보다 목표/경로 정렬 관련 비용이 상대적으로 커서, 후보 궤적 중 장애물 근접 궤적이 선택될 수 있음. 또한 `sim_time`이 짧으면 “더 먼 미래 충돌”을 평가하지 못함 | `controller_server.ros__parameters.FollowPath.BaseObstacle.scale` / `controller_server.ros__parameters.FollowPath.sim_time` | `scale: 0.05~0.1` / `sim_time: 2.0` | 장애물에 더 보수적으로 반응, 더 이른 회피 시작 | scale이 과하면 좁은 곳에서 회피가 과도해져 **oscillation/진행 불가** 가능. `sim_time`↑는 CPU 부하↑ 및 보수적 회피↑ | `scale: 0.02`, `sim_time: 1.5`로 복귀 |
| 좁은 문/틈을 못 지나가고 뱅뱅 돎 | inflation이 통로를 사실상 막아버림. 특히 `inflation_radius`가 크면 통로 전체가 고비용이 되고, `cost_scaling_factor`가 낮으면 고비용 구역이 넓게 퍼짐 | `local_costmap...inflation_layer.cost_scaling_factor` / `global_costmap...inflation_layer.cost_scaling_factor` (필요 시 `inflation_radius`) | `cost_scaling_factor: 5.0~10.0` (먼저 시도) / `inflation_radius: 0.35~0.45` (최후) | 좁은 통로에서 통과 가능성↑ (장애물 바로 옆만 높은 비용) | `cost_scaling_factor`↑는 장애물 가까이 붙어 주행할 수 있어 **안전거리↓**. `inflation_radius`↓는 충돌 리스크↑ | `cost_scaling_factor: 3.0`, `inflation_radius` 원복(local `0.45`, global `0.55`) |
| “움찔움찔” / stop-go / 경로를 자주 다시 짬 | 가속도 제한이 너무 낮으면 속도/추종이 버벅일 수 있음. 또한 costmap update가 느리면 장애물 비용이 계단식으로 갱신되어 jerk/oscillation을 유발할 수 있음. (단, 주파수 정합은 ‘필수’는 아니고 환경/센서에 따라 체감 차이) | `controller_server...FollowPath.acc_lim_x` / `local_costmap...update_frequency` | `acc_lim_x: 2.0` (소폭) / `update_frequency: 10.0` (Jetson 여유 시) | 추종이 더 부드러워지고 불필요한 멈춤 감소 | `acc_lim_x`↑는 slip/IMU 노이즈↑로 localization 악화 가능. `update_frequency`↑는 CPU 부하↑(특히 voxel_layer) | `acc_lim_x: 1.5`, `update_frequency: 5.0`로 복귀 |
| CPU 부하가 심함(Jetson 느려짐) | DWB는 샘플 수(`vx_samples`, `vtheta_samples`, `vy_samples`)×sim_time 등으로 후보 궤적 평가량이 커짐. AMCL은 `max_particles`가 커질수록 업데이트 비용↑ | `controller_server...FollowPath.vx_samples` / `vtheta_samples` (필요 시 `vy_samples`) / `amcl.ros__parameters.max_particles` | `10~15` / `max_particles: 2000` | 즉각적인 CPU 사용률 감소(체감) | 샘플 수↓는 장애물 회피 품질/부드러움 저하 가능. `max_particles`↓는 대칭/특징 없는 맵에서 튐이 다시 늘 수 있음 | `vx_samples: 20`, `vtheta_samples: 20`, `max_particles: 3000`로 복귀 |

### Localization ↔ Avoidance 트레이드오프 튜닝 가이드 (미적용)

아래는 **현재 적용된 변경 로그가 아니라**, “맵 대칭으로 인한 localization 튐”과 “동적 장애물 회피 반응성” 사이에서 균형을 잡기 위한 **조절 레버**를 정리한 가이드입니다.

**1) Localization 딜레마: 튐(jump) ↔ 드리프트(drift)**

| 상황 | 조절 레버(파라미터 경로) | 조정 방향 | 기대 효과 | 주의/리스크 | 검증 방법 |
|---|---|---|---|---|---|
| 드리프트가 누적되어 map 상 위치가 조금씩 어긋남(odom 과신) | `amcl.ros__parameters.alpha1~4` | `0.05`에서 **소폭↑**(예: `0.08~0.12`) | motion 모델에서 odom 불확실성을 키워 레이저 업데이트가 더 영향력을 갖도록 유도(드리프트 보정에 도움) | 대칭 환경에서는 다시 **corner jump**가 늘 수 있음 | 동일 루트 반복 주행 후 `amcl_pose` 누적 오차 비교, `tf(map->odom)` 드리프트 추세 확인 |
| 드리프트 보정이 잘 안 되고 라이다 매칭이 약함 | `amcl.ros__parameters.z_hit` / `z_rand` | `z_hit` **↑**, `z_rand` **↓** (가급적 소폭) | 측정 모델에서 “hit” 비중을 키워 맵-스캔 정합을 더 강하게 유도 | 이 계열 파라미터(`z_hit/z_short/z_max/z_rand`)는 **상대 가중치**라서 한 개만 크게 바꾸면 부작용이 커질 수 있음(스캔 노이즈/유리/반사에 민감) | 장애물/벽 주변에서 pose 안정성, 유리/반사 환경에서 튐 증가 여부 확인 |
| pose가 너무 “자주” 흔들리거나 jitter가 심함(정지/저속) | `amcl.ros__parameters.update_min_d` / `update_min_a` | **↑** (업데이트를 덜 자주) | 미세 움직임/노이즈에 의한 과도한 업데이트 감소 → jitter 감소 | 너무 키우면 빠른 움직임에서 보정이 늦어져 추종 품질 저하 가능 | 정지/저속 상태에서 pose 흔들림 빈도 비교, 빠른 회전에서 지연 체감 확인 |
| 대칭 환경에서 여전히 튐(소프트웨어 한계) | (환경/맵) | 맵에 비대칭 요소 추가(문 열기/박스 배치 등) | “관측 모호성” 자체를 줄여 AMCL이 헷갈릴 여지를 감소(가장 효과적) | 맵을 다시 땄을 때 다른 파라미터 재튜닝 필요 | 같은 시작점에서 corner jump 재현성 비교 |
| (최후) 튐을 수치로 눌러야 함 | `amcl.ros__parameters.max_particles` | **↑** (예: `5000~8000`) | 대칭/모호한 상황에서 particle 다양성을 키워 잘못된 모드로 급수렴하는 위험 감소 | CPU/메모리 증가(실시간성 저하 가능) | CPU 사용률/AMCL update rate 확인, 튐 빈도 감소 여부 비교 |

**2) 동적 장애물 반응성: “빨리 감지/빨리 회피” ↔ “좁은 길 통과/CPU”**

| 목표 | 조절 레버(파라미터 경로) | 조정 방향 | 기대 효과 | 주의/리스크 | 검증 방법 |
|---|---|---|---|---|---|
| 장애물을 더 멀리서 감지해서 미리 피하고 싶음 | `local_costmap.local_costmap.ros__parameters.voxel_layer.scan.obstacle_max_range` | **↑** (라이다 스펙 내) | 더 먼 거리의 장애물이 costmap에 일찍 반영 | false positive/노이즈가 늘 수 있고, marking 범위가 늘어 CPU↑ 가능 | 장애물 급출현에서 회피 시작 시점 비교, costmap 노이즈(점 잡음) 확인 |
| 장애물이 사라졌을 때 잔상이 오래 남음(늦게 clearing) | `local_costmap...voxel_layer.scan.raytrace_max_range` | `obstacle_max_range`보다 **약간↑** | clearing raytrace 범위가 충분해져 잔상 제거가 빨라짐 | 너무 크면 불필요한 raytrace로 CPU↑ 가능 | 장애물 제거 후 costmap에서 사라지는 시간 비교 |
| 장애물을 더 “일찍” 피하게 만들고 싶음(안전거리 확보) | `local_costmap...inflation_layer.cost_scaling_factor` / `inflation_radius` | `cost_scaling_factor` **↓** 또는 `inflation_radius` **↑** | 낮은 cost_scaling은 비용 감쇠가 느려져 “위험 구역”이 넓어짐(더 일찍 꺾음) / inflation_radius↑는 절대 안전거리↑ | 좁은 길이 더 쉽게 막힘. 기존 “좁은 문 통과” 목표와 충돌하는 trade-off | 넓은 공간에서 회피 시작 거리↑ 확인, 좁은 문 통과 실패 여부 확인 |
| Controller가 장애물 쪽을 너무 과감하게 파고듦 | `controller_server.ros__parameters.FollowPath.BaseObstacle.scale` | **↑** | 장애물 비용이 커져 근접 궤적이 선택될 확률↓ | 과하면 oscillation/진행 불가↑ | 같은 코너/장애물 배치에서 최소 거리/충돌 여부 비교 |
| 감지→회피의 “지연”이 큼(반응이 둔함) | `local_costmap.local_costmap.ros__parameters.update_frequency` / `controller_server.ros__parameters.controller_frequency` | **↑** (Jetson 여유 시) | costmap 갱신/제어 루프가 빨라져 반응성↑ | CPU↑. 특히 `voxel_layer`는 주파수 증가에 민감 | htop/로그로 CPU 확인, cmd_vel 갱신 지연/주기 확인 |

### BT Navigator (Behavior Tree Manager)

**BT Navigator 튜닝 전(원본) 값 요약**

```yaml
bt_navigator:
  ros__parameters:
    use_sim_time: False
```

**변경 로그 테이블 (BT Navigator)**

| 날짜 | 파라미터(경로) | Before | After | 변경 이유(관찰/사용자 발화) | 예상 기대결과 | 검증 방법(체크리스트) | 리스크/트레이드오프 | 롤백 |
|---|---|---|---|---|---|---|---|---|
| 2025-12-18 | `bt_navigator.ros__parameters.use_sim_time` | `False` | `False` | 실제 Go2 + Jetson 운용에서는 `/clock`가 없으므로 Sim Time을 켜면 BT가 시간 진행을 기다리거나 TF/time 관련 오류가 발생할 수 있음. 전체 Nav2 노드들과 시간 정책을 **일관되게 System Time(=False)** 로 맞춤. 플러그인 리스트는 Nav2 동작에 필수이므로 유지 | BT tick/Action 전개가 정상 동작, time sync 불일치로 인한 경고/지연 감소 | nav2 로그에서 time 관련 warning 확인, `/clock` 토픽 존재 여부 확인, 네비게이션 시 BT가 멈추지 않는지 확인 | 시뮬 환경에서는 `/clock` 기반 노드와 시간 불일치 가능 | 시뮬에서는 `True`로 복귀 |

**변경 기록 템플릿 (BT Navigator)**

| 날짜 | 파라미터(경로) | Before | After | 변경 이유(관찰/사용자 발화) | 예상 기대결과 | 검증 방법(체크리스트) | 리스크/트레이드오프 | 롤백 |
|---|---|---|---|---|---|---|---|---|
| YYYY-MM-DD | `bt_navigator.ros__parameters.<param>` |  |  |  |  |  |  |  |

### BT Node Parameters (NavigateToPose / NavigateThroughPoses)

**BT Node Parameters 튜닝 전(원본) 값 요약**

```yaml
bt_navigator_navigate_through_poses_rclcpp_node:
  ros__parameters:
    use_sim_time: False
bt_navigator_navigate_to_pose_rclcpp_node:
  ros__parameters:
    use_sim_time: False
```

**변경 로그 테이블 (BT Node Parameters)**

| 날짜 | 파라미터(경로) | Before | After | 변경 이유(관찰/사용자 발화) | 예상 기대결과 | 검증 방법(체크리스트) | 리스크/트레이드오프 | 롤백 |
|---|---|---|---|---|---|---|---|---|
| 2025-12-18 | `bt_navigator_navigate_through_poses_rclcpp_node.ros__parameters.use_sim_time` | `False` | `False` | BT 하위 노드도 상위(`bt_navigator`)와 동일하게 System Time으로 맞춰 time sync 문제를 예방 | NavigateThroughPoses 수행 중 시간 불일치 경고/지연 감소 | nav2 로그에서 time 관련 warning 확인, Waypoints(ThroughPoses) 동작 중 멈춤 여부 확인 | 시뮬 환경에서는 `/clock` 기반과 불일치 가능 | 시뮬에서는 `True`로 복귀 |
| 2025-12-18 | `bt_navigator_navigate_to_pose_rclcpp_node.ros__parameters.use_sim_time` | `False` | `False` | NavigateToPose 수행 시도 시 time mismatch로 액션/TF가 멈추는 상황을 예방 | 단일 목표점 주행 시 액션이 정상 진행 | 네비게이션 액션 응답/진행률 확인, time warning 확인 | 시뮬 환경에서는 `/clock` 기반과 불일치 가능 | 시뮬에서는 `True`로 복귀 |

**변경 기록 템플릿 (BT Node Parameters)**

| 날짜 | 파라미터(경로) | Before | After | 변경 이유(관찰/사용자 발화) | 예상 기대결과 | 검증 방법(체크리스트) | 리스크/트레이드오프 | 롤백 |
|---|---|---|---|---|---|---|---|---|
| YYYY-MM-DD | `bt_navigator_navigate_to_pose_rclcpp_node.ros__parameters.<param>` |  |  |  |  |  |  |  |

### Behavior Server (Recovery Behaviors)

**Behavior Server 튜닝 전(원본) 값 요약**

```yaml
behavior_server:
  ros__parameters:
    use_sim_time: True
```

**변경 로그 테이블 (Behavior Server)**

| 날짜 | 파라미터(경로) | Before | After | 변경 이유(관찰/사용자 발화) | 예상 기대결과 | 검증 방법(체크리스트) | 리스크/트레이드오프 | 롤백 |
|---|---|---|---|---|---|---|---|---|
| 2025-12-18 | `behavior_server.ros__parameters.use_sim_time` | `True` | `False` | 원본 파일에서 `behavior_server`만 Sim Time이 켜져 있었고, 다른 노드(AMCL/Controller/TF)는 System Time 기반이라 시간축이 갈라짐. 이 경우 `Spin/BackUp` 등 복구 동작이 시간 진행을 못 받아 **실제 로봇에서 복구 동작이 멈추거나 TF/time 오류**가 날 수 있어 치명적. 실로봇 운용 기준으로 False로 강제 | stuck/oscillation 등 상황에서 recovery behavior가 정상 실행, time mismatch 관련 에러 감소 | 의도적으로 경로 막힘 상황을 만들고 recovery(Spin/BackUp/Wait)가 수행되는지 확인, nav2 로그에서 TF/time 에러 여부 확인 | 시뮬 환경에서는 `/clock` 기반과 불일치 가능 | 시뮬에서는 `True`로 복귀 |

**변경 기록 템플릿 (Behavior Server)**

| 날짜 | 파라미터(경로) | Before | After | 변경 이유(관찰/사용자 발화) | 예상 기대결과 | 검증 방법(체크리스트) | 리스크/트레이드오프 | 롤백 |
|---|---|---|---|---|---|---|---|---|
| YYYY-MM-DD | `behavior_server.ros__parameters.<param>` |  |  |  |  |  |  |  |

### Robot State Publisher

**Robot State Publisher 튜닝 전(원본) 값 요약**

```yaml
robot_state_publisher:
  ros__parameters:
    use_sim_time: False
```

**변경 로그 테이블 (Robot State Publisher)**

| 날짜 | 파라미터(경로) | Before | After | 변경 이유(관찰/사용자 발화) | 예상 기대결과 | 검증 방법(체크리스트) | 리스크/트레이드오프 | 롤백 |
|---|---|---|---|---|---|---|---|---|
| 2025-12-18 | `robot_state_publisher.ros__parameters.use_sim_time` | `False` | `False` | TF 발행 노드도 System Time으로 맞춰야 TF timestamp 불일치로 인한 경고/lookup 실패를 줄일 수 있음 | TF 안정성↑, time mismatch 경고 감소 | `tf2_echo`로 TF가 끊기지 않는지 확인, nav2 로그에서 TF extrapolation/time error 확인 | 시뮬 환경에서는 `/clock` 기반과 불일치 가능 | 시뮬에서는 `True`로 복귀 |

**변경 기록 템플릿 (Robot State Publisher)**

| 날짜 | 파라미터(경로) | Before | After | 변경 이유(관찰/사용자 발화) | 예상 기대결과 | 검증 방법(체크리스트) | 리스크/트레이드오프 | 롤백 |
|---|---|---|---|---|---|---|---|---|
| YYYY-MM-DD | `robot_state_publisher.ros__parameters.<param>` |  |  |  |  |  |  |  |

### Waypoint Follower

**Waypoint Follower 튜닝 전(원본) 값 요약**

```yaml
waypoint_follower:
  ros__parameters:
    use_sim_time: False
```

**변경 로그 테이블 (Waypoint Follower)**

| 날짜 | 파라미터(경로) | Before | After | 변경 이유(관찰/사용자 발화) | 예상 기대결과 | 검증 방법(체크리스트) | 리스크/트레이드오프 | 롤백 |
|---|---|---|---|---|---|---|---|---|
| 2025-12-18 | `waypoint_follower.ros__parameters.use_sim_time` | `False` | `False` | 경유지 주행 기능도 System Time으로 맞춰 전체 Nav2와 time sync 일관성 유지 | waypoint 수행 중 시간 이슈로 멈추는 상황 예방 | waypoint 수행 시 지연/멈춤 여부 확인, time warning 확인 | 시뮬 환경에서는 `/clock` 기반과 불일치 가능 | 시뮬에서는 `True`로 복귀 |

**변경 기록 템플릿 (Waypoint Follower)**

| 날짜 | 파라미터(경로) | Before | After | 변경 이유(관찰/사용자 발화) | 예상 기대결과 | 검증 방법(체크리스트) | 리스크/트레이드오프 | 롤백 |
|---|---|---|---|---|---|---|---|---|
| YYYY-MM-DD | `waypoint_follower.ros__parameters.<param>` |  |  |  |  |  |  |  |