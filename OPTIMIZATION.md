이 문서는 Jetson Orin Nano (8GB) 환경에서 ROS2 + Nav2 + WebRTC 스트리밍 성능을 극대화하기 위해 적용된 코드 변경 사항을 정리합니다.

## 1. WebRTC 병목 해소 (비동기 처리)
**파일:** `@go2_ros2_sdk/go2_robot_sdk/go2_robot_sdk/infrastructure/webrtc/go2_connection.py`

### 문제점
- WebRTC 데이터 수신 콜백(`on_data_channel_message`)에서 무거운 LiDAR 디코딩을 **동기(Synchronous)**로 수행.
- LiDAR 처리가 늦어지면 같은 채널로 들어오는 **Odom(로봇 위치), RobotState 데이터 처리가 함께 지연됨**.
- 결과적으로 RViz에서 로봇 움직임이 뚝뚝 끊기거나 과거 위치에 머무는 현상 발생.

### 변경 사항
- **Worker Thread 도입:** `threading`과 `queue`를 사용하여 LiDAR 데이터 처리를 별도 스레드(`_lidar_worker`)로 분리.
- **Head-Drop 전략:** 큐 크기를 `2`로 제한하고, 처리 속도가 수신 속도보다 느리면 **가장 오래된 프레임을 즉시 폐기**.
- **효과:** 메인 WebRTC 스레드가 절대 차단되지 않음 -> **Odom/TF 실시간성 확보**.

```python
# 변경된 로직 요약
def on_data_channel_message(self, message):
    if isinstance(message, bytes):
        # 큐가 꽉 차면 옛날 데이터 버리고 최신 데이터 넣기 (Head-Drop)
        if self.lidar_queue.full():
            self.lidar_queue.get_nowait()
        self.lidar_queue.put_nowait(message)
```

## 2. LiDAR 디코딩 가속 (메모리 복사 최적화)
**파일:** `@go2_ros2_sdk/go2_robot_sdk/go2_robot_sdk/infrastructure/sensors/lidar_decoder.py`

### 문제점
- WASM(WebAssembly) 메모리에 데이터를 쓸 때 Python `for` 루프로 **1바이트씩 복사** (`add_value_arr`).
- 디코딩된 데이터를 꺼낼 때도 불필요한 `bytearray` 생성 및 리스트 슬라이싱 발생.
- `update_meshes_for_cloud2`에서 모든 점을 `float32`로 변환한 뒤 중복 제거(`unique`)를 수행하여 메모리와 연산 낭비 심함.

### 변경 사항
- **`ctypes.memmove` 적용:** 루프 없이 메모리 블록을 한 번에 복사 (C++ memcpy 수준 속도).
- **Zero-Copy 추출:** WASM 메모리에서 Numpy 배열로 데이터를 **직접 복사**.
- **조기 필터링 (Early Filtering):**
    - `float32` 변환 전, **`uint8` 단계에서 필터링 및 중복 제거** 수행.
    - 데이터 크기가 1/4인 상태에서 연산하므로 속도 대폭 향상.

## 3. ROS2 퍼블리싱 가속 (직렬화 회피)
**파일:** `@go2_robot_sdk/go2_robot_sdk/infrastructure/ros2/ros2_publisher.py`

### 문제점
- `point_cloud2.create_cloud()` 함수가 수만 개의 점을 하나씩 순회하며 패킹(Serialization)함.
- CPU 사용량이 매우 높음.

### 변경 사항
- **직접 메모리 할당:** Numpy 배열의 메모리(`tobytes()`)를 `PointCloud2.data`에 직접 대입.
- 반복문이 완전히 제거되어 **CPU 부하가 거의 0에 수렴**.

```python
# 변경된 로직 요약
point_cloud.data = points.tobytes() # Zero-Copy에 가까운 방식
```

## 4. 하드웨어/시스템 권장 설정 (User Action Required)

코드 최적화와 별개로 Jetson Orin Nano 8GB의 성능을 위해 다음 설정이 필수입니다.

1.  **Swap 메모리 확장 (필수):**
    ```bash
    # ZRAM 끄고 디스크 기반 Swap 8GB 생성
    sudo systemctl disable nvzramconfig
    sudo fallocate -l 8G /swapfile && sudo mkswap /swapfile && sudo swapon /swapfile
    ```
2.  **전력 모드:** `sudo nvpmodel -m 0` (15W MAX 성능)
3.  **Jetson Clocks:** `sudo jetson_clocks` (최대 클럭 고정)
