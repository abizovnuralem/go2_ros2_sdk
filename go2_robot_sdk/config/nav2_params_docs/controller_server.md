# Controller Server (Local Planner / DWB)

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
