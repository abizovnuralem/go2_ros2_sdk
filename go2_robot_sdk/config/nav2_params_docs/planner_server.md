# Planner Server (Global Planner)

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
