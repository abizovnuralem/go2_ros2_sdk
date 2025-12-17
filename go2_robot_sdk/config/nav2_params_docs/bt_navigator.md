# BT Navigator (Behavior Tree Manager)

**BT Navigator 튜닝 전(원본) 값 요약**

```yaml
bt_navigator:
  ros__parameters:
    use_sim_time: False
```

**변경 로그 테이블 (BT Navigator)**

| 날짜 | 파라미터(경로) | Before | After | 변경 이유(관찰/사용자 발화) | 예상 기대결과 | 검증 방법(체크리스트) | 리스크/트레이드오프 | 롤백 |
|---|---|---|---|---|---|---|---|---|
| 2025-12-18 | `bt_navigator.ros__parameters.use_sim_time` | `False` | `False` | 실제 Go2 + Jetson 운용에서는 `/clock`가 없으므로 Sim Time을 켜면 BT가 시간 진행을 기다리거나 TF/time 관련 오류가 발생할 수 있음. 전체 Nav2 노드들과 시간 정책을 **일관되게 System Time(=False)** 로 맞춤. 플러그인 리스트는 Nav2 동작에 필수이므로 유지 | BT tick/Action 전개가 정상 동작, time sync 불일치로 인한 경고/지연 감소 | nav2 로그에서 time 관련 warning 확인, `/clock` 토픽 존재 여부 확인, 네비게이션 시 BT가 멈추지 않는지 확인 | 시뮬 환경에서는 `/clock` 기반 노드와 시간 불일치 가능 | 시뮬에서는 `True`로 복귀 |

**변경 기록 템플릿 (BT Navigator)**

| 날짜 | 파라미터(경로) | Before | After | 변경 이유(관찰/사용자 발화) | 예상 기대결과 | 검증 방법(체크리스트) | 리스크/트레이드오프 | 롤백 |
|---|---|---|---|---|---|---|---|---|
| YYYY-MM-DD | `bt_navigator.ros__parameters.<param>` |  |  |  |  |  |  |  |
