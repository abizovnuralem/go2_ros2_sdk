# Behavior Server (Recovery Behaviors)

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
