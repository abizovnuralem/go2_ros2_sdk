# Waypoint Follower

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
