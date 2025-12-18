# Localization ↔ Avoidance 트레이드오프 튜닝 가이드 (미적용)

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
