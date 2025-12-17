# BT Node Parameters (NavigateToPose / NavigateThroughPoses)

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
