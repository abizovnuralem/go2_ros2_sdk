# lidar_accelerator

`lidar_accelerator`는 **Go2 LiDAR 처리 파이프라인의 CPU 병목을 줄이기 위한 pybind11 기반 C++ 가속 모듈**입니다.

- **Phase A (완료)**: (positions/uvs → XYZI float32) 전처리 및 필터링을 C++로 가속
- **Phase B (준비 완료 / 실데이터 샘플 필요)**: `libvoxel.wasm` 기반 디코드 자체를 Python(wasmtime)에서 C++(wasmtime C API)로 이동하여 end-to-end 디코드+전처리 경로 제공

이 패키지는 ROS2 워크스페이스에서 `ament_cmake`로 빌드되며, 결과물은 Python 모듈로 설치되어 **`import lidar_accelerator`**로 사용됩니다.

---

## 목적

### 해결하려는 문제
Go2의 WebRTC LiDAR 메시지는 압축된 voxel 데이터를 포함하며, 전체 파이프라인에서 아래 구간이 CPU 병목이 되기 쉽습니다.

- Python 레벨 데이터 가공(필터링/다운샘플/포인트 제한/중복 제거)
- (Phase B 목표) Python wasmtime 기반 WASM 호출 및 메모리 복사

### 제공하는 것
- `process_u8_to_xyzi_f32(...)`:
  - 입력: `positions`(uint8), `uvs`(uint8)
  - 출력: `points` (N,4) `float32` (x,y,z,intensity)
  - 파라미터: `intensity_threshold`, `downsample_step`, `max_points`, `deduplicate`
- `decode_and_process(...)`:
  - 입력: 압축 바이트(`compressed_data`)
  - 출력: `points` (N,4) `float32`
  - 내부: WASM(libvoxel.wasm) 호출 + 즉시 전처리

---

## 구성(디렉토리 구조)

- `CMakeLists.txt`
  - `pybind11_add_module(lidar_accelerator ...)`
  - wasmtime C API 탐지 시 `GO2_WASMTIME_C_API=1` 정의
- `package.xml`
  - ROS2 패키지 메타데이터
- `include/lidar_accelerator/`
  - `processing.hpp`: Phase A 전처리
  - `packing.hpp`: (선택) bytes 패킹
  - `wasm_decode.hpp`: Phase B 디코드+전처리
- `src/`
  - `processing.cpp`: Phase A 구현
  - `pybind_module.cpp`: Python 바인딩
  - `packing.cpp`: (선택) bytes 패킹
  - `wasm_decode.cpp`: Phase B (wasmtime C API) 구현
- `scripts/`
  - `bench_go2_lidar_accel.py`: 전처리 벤치 + sweep/CSV
  - `bench_go2_lidar_packing.py`: 패킹 벤치
  - `bench_go2_lidar_decode_and_process_sample.py`: recorded sample 기반 end-to-end 벤치(Phase B)
  - `make_dummy_ulidar_array_buffer.py`: 더미 array-buffer 생성(포맷 테스트용)
- `tests/`
  - `conftest.py`: 테스트 실행을 위한 import path 설정
  - `test_*.py`: 유닛/통합 테스트

---

## 런타임 플로우(실사용 경로)

### A) Phase A: positions/uvs 기반 전처리 가속
`go2_robot_sdk`에서 LiDAR를 PointCloud2로 퍼블리시하는 worker는 다음 흐름을 탑니다.

1. WebRTC로부터 받은 데이터에서 `positions`, `uvs`, `resolution`, `origin`을 확보
2. `update_meshes_for_cloud2(... use_cpp_accel=True)`
3. 내부에서 `import lidar_accelerator` 후 `process_u8_to_xyzi_f32` 호출
4. 결과 `(N,4) float32`를 `PointCloud2.data = points.tobytes()`로 패킹하여 퍼블리시

### B) Phase B: compressed_data 기반 디코드+전처리(준비 완료)
실데이터(`ulidar array-buffer .bin`)가 있으면 다음 경로를 통해 **디코드 자체 병목**까지 C++로 이동 가능합니다.

1. WebRTC에서 `compressed_data`를 그대로 전달
2. `ROS2Publisher._lidar_worker()`가 `lidar_accelerator.decode_and_process(...)`를 우선 시도
3. 실패 시 Python(wasmtime) 디코더 경로로 자동 fallback

---

## 빌드

컨테이너(또는 Jetson)에서:

```bash
source /opt/ros/humble/setup.bash
cd /ros2_ws
colcon build --packages-select lidar_accelerator --cmake-args -DCMAKE_BUILD_TYPE=Release
source /ros2_ws/install/setup.bash
```

> Phase B를 위해서는 wasmtime C API가 필요합니다. 컨테이너 이미지에 `/opt/wasmtime-c-api`가 설치되어 있어야 합니다.

---

## 테스트

`lidar_accelerator/tests` 기준:

```bash
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash
python3 -m pytest -q -o addopts= /ros2_ws/src/third_party/go2_ros2_sdk/lidar_accelerator/tests
```

- recorded sample이 없으면 `test_lidar_offline_sample_integration.py`는 skip될 수 있습니다.

---

## 벤치마크

### 1) Phase A 전처리 벤치

```bash
python3 /ros2_ws/src/third_party/go2_ros2_sdk/lidar_accelerator/scripts/bench_go2_lidar_accel.py \
  --points 30000 --warmup 2 --iters 20 \
  --downsample 32 --max-points 3000 --deduplicate false --intensity 0.0
```

### 2) Phase A Sweep + CSV(보고용)

```bash
python3 /ros2_ws/src/third_party/go2_ros2_sdk/lidar_accelerator/scripts/bench_go2_lidar_accel.py \
  --sweep --warmup 2 --iters 20 \
  --points-list 3000,30000 \
  --downsample-list 1,32 \
  --max-points-list 0,3000 \
  --deduplicate-list false,true \
  --sweep-out /tmp/lidar_preprocess_sweep.csv
```

CSV 컬럼:
- `points, downsample, max_points, deduplicate, label, iters, mean_ms, median_ms, min_ms, max_ms, speedup_vs_python`

### 2.1) Phase A 벤치마크 결과(예시)

아래는 컨테이너 환경에서 `points=30000`, `intensity=0.0`, `iters=30`로 측정한 예시 결과입니다.

비교 기준:
- `python`: `update_meshes_for_cloud2(... use_cpp_accel=False)`
- `cpp_via_switch`: `update_meshes_for_cloud2(... use_cpp_accel=True)` (실사용 경로)

설정:
- `max_points=3000`

| downsample_step | deduplicate | python mean (ms) | cpp_via_switch mean (ms) | speedup |
|---:|:---:|---:|---:|---:|
| 2 | false | 1.673978 | 1.020360 | 1.640576x |
| 2 | true  | 2.531029 | 2.118821 | 1.194546x |
| 4 | false | 1.446396 | 0.981397 | 1.473813x |
| 4 | true  | 2.571196 | 2.121090 | 1.212205x |
| 8 | false | 1.501358 | 0.945406 | 1.588058x |
| 8 | true  | 2.528978 | 2.151944 | 1.175206x |

### 3) Phase B end-to-end 벤치(실데이터 필요)

```bash
python3 /ros2_ws/src/third_party/go2_ros2_sdk/lidar_accelerator/scripts/bench_go2_lidar_decode_and_process_sample.py \
  /ros2_ws/lidar_samples/ulidar_array_buffer_XXXX.bin \
  --warmup 3 --iters 20 --intensity 0.0 --downsample 32 --max-points 3000
```

---

## 오프라인 샘플 워크플로우(Phase B 검증용)

### 1) 샘플 덤프(로봇이 잠깐 가능할 때)
환경변수로 raw array-buffer를 파일로 저장합니다.

- `LIDAR_DUMP_DIR`: 덤프 디렉토리
- `LIDAR_DUMP_MAX`: 최대 저장 파일 개수

### 2) recorded sample 기반 통합 테스트

```bash
export LIDAR_SAMPLE_PATH=/ros2_ws/lidar_samples/ulidar_array_buffer_XXXX.bin
python3 -m pytest -q -o addopts= \
  /ros2_ws/src/third_party/go2_ros2_sdk/lidar_accelerator/tests/test_lidar_offline_sample_integration.py
```

---

## API

### `lidar_accelerator.process_u8_to_xyzi_f32(positions, uvs, res, origin, intense_limiter, deduplicate=True, downsample_step=1, max_points=0) -> np.ndarray`
- 반환: `(N,4)` `float32`

### `lidar_accelerator.decode_and_process(compressed, res, origin, intense_limiter, deduplicate=True, downsample_step=1, max_points=0) -> np.ndarray`
- 반환: `(N,4)` `float32`
- 주의: 올바른 Go2 LiDAR compressed bytes가 아니면 실패할 수 있습니다.
