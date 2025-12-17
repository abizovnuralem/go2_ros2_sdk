# Global Costmap

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
