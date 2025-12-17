# Nav2 Parameters (`go2_robot_sdk/config/nav2_params.yaml`)

`nav2_params.yaml` 변경을 **왜 했는지(문제/가설)**, **무엇을 바꿨는지(파라미터/Before/After)**, 그리고 **예상 기대결과/검증 방법/롤백 방법**까지 블록별로 기록한 문서입니다.

- **Config 파일**: [`nav2_params.yaml`](./nav2_params.yaml)
- **Reference**: [관련 문서 프롬프트](https://aistudio.google.com/app/prompts/13bQ00apmpNhlRA3CXkmPfvtdOs3NY-KP)

---

## 섹션별 문서

- **AMCL (Localization)**: [`nav2_params_docs/amcl.md`](./nav2_params_docs/amcl.md)
- **Planner Server (Global Planner)**: [`nav2_params_docs/planner_server.md`](./nav2_params_docs/planner_server.md)
- **Controller Server (Local Planner / DWB)**: [`nav2_params_docs/controller_server.md`](./nav2_params_docs/controller_server.md)
- **Local Costmap**: [`nav2_params_docs/local_costmap.md`](./nav2_params_docs/local_costmap.md)
- **Global Costmap**: [`nav2_params_docs/global_costmap.md`](./nav2_params_docs/global_costmap.md)
- **장애물 회피/주행 이상 시 트러블슈팅 (미적용 가이드)**: [`nav2_params_docs/troubleshooting.md`](./nav2_params_docs/troubleshooting.md)
- **Localization ↔ Avoidance 트레이드오프 튜닝 가이드 (미적용)**: [`nav2_params_docs/localization_avoidance_tradeoff.md`](./nav2_params_docs/localization_avoidance_tradeoff.md)
- **BT Navigator (Behavior Tree Manager)**: [`nav2_params_docs/bt_navigator.md`](./nav2_params_docs/bt_navigator.md)
- **BT Node Parameters (NavigateToPose / NavigateThroughPoses)**: [`nav2_params_docs/bt_node_parameters.md`](./nav2_params_docs/bt_node_parameters.md)
- **Behavior Server (Recovery Behaviors)**: [`nav2_params_docs/behavior_server.md`](./nav2_params_docs/behavior_server.md)
- **Robot State Publisher**: [`nav2_params_docs/robot_state_publisher.md`](./nav2_params_docs/robot_state_publisher.md)
- **Waypoint Follower**: [`nav2_params_docs/waypoint_follower.md`](./nav2_params_docs/waypoint_follower.md)
