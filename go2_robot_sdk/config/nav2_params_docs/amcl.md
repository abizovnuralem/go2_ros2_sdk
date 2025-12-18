# AMCL (Localization)

**AMCL 튜닝 전(원본) 값 요약**

```yaml
amcl:
  ros__parameters:
    alpha1: 0.2
    alpha2: 0.2
    alpha3: 0.2
    alpha4: 0.2
    alpha5: 0.2
    max_particles: 2000
    update_min_a: 0.2
    update_min_d: 0.25
```

**변경 로그 테이블 (AMCL)**

| 날짜 | 파라미터(경로) | Before | After | 변경 이유(관찰/사용자 발화) | 예상 기대결과 | 검증 방법(체크리스트) | 리스크/트레이드오프 | 롤백 |
|---|---|---|---|---|---|---|---|---|
| 2025-12-17 | `amcl.ros__parameters.alpha1~4` | `0.2` | `0.05` | 특징 없는 정사각형 맵에서 corner 대칭으로 라이다 매칭이 헷갈려 **pose가 다른 corner로 점프**하는 현상 | 오도메트리를 더 신뢰해서 **대칭 환경에서 순간이동/튐 감소** | RViz에서 `amcl_pose` / `tf(map->odom)` 점프 여부 확인, 같은 루트로 반복 주행 시 재현성 확인 | 라이다 기반 보정이 약해져 장거리 누적 drift가 커질 수 있음(환경에 따라) | `alpha1~4`를 `0.2`로 복귀 |
| 2025-12-17 | `amcl.ros__parameters.max_particles` | `2000` | `3000` | 대칭 환경에서 후보 위치가 여러 곳으로 분산되므로 particle 다양성 확보 필요 | ambiguity 상황에서 **진짜 위치 particle 생존 확률 증가**, 튐 감소 | CPU 사용률/AMCL 업데이트 주기 확인, pose 튐 빈도 비교 | CPU/메모리 사용 증가(저사양에서는 latency 증가 가능) | `max_particles: 2000`으로 복귀 |
| 2025-12-17 | `amcl.ros__parameters.update_min_d`, `update_min_a` | `0.25`, `0.2` | `0.2`, `0.2` | 너무 자주 업데이트하면 노이즈가 누적되어 jitter/맵 회전처럼 보이는 현상 유발 가능 | **정지/미세 움직임에서 jitter 감소**, 안정적인 pose | 제자리/저속 이동에서 pose 흔들림 감소 확인, tracking 안정성 확인 | 반응성이 떨어져 빠른 움직임에서 pose 갱신이 늦어 보일 수 있음 | `update_min_d: 0.25`, `update_min_a: 0.2`로 복귀 |

**변경 기록 템플릿 (AMCL)**

| 날짜 | 파라미터(경로) | Before | After | 변경 이유(관찰/사용자 발화) | 예상 기대결과 | 검증 방법(체크리스트) | 리스크/트레이드오프 | 롤백 |
|---|---|---|---|---|---|---|---|---|
| YYYY-MM-DD | `amcl.ros__parameters.<param>` |  |  |  |  |  |  |  |
