# Local Costmap

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
