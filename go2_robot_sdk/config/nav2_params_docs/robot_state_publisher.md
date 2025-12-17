# Robot State Publisher

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
