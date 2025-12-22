# Go2 ROS2 SDK Style Guide

## 🚨 Core Instruction
**Review & Chat Language: Korean (한국어)**
이 프로젝트는 외부 SDK이지만, 팀 내 소통을 위해 리뷰는 한국어로 진행합니다.

---

## 1. Architecture: 'Clean Architecture'
This SDK follows strict Clean Architecture principles.
*   **Presentation Layer:** ROS 2 Nodes (`go2_driver_node.py`). Handles User/ROS interactions.
*   **Domain Layer:** Core logic & entities (`robot_commands.py`). Must be independent of Frameworks.
*   **Infrastructure Layer:** External interfaces (`webrtc_adapter`, `cyclonedds`).

**Rule:** Dependencies points **inwards**.
*   `Infrastructure` depends on `Domain`.
*   `Presentation` depends on `Domain`.
*   `Domain` depends on **Nothing**.

## 2. Concurrency Model
*   **Hybrid Approach:** Uses both `asyncio` (Main Loop) and `threading` (ROS 2 Spin).
*   **Rule:** 
    *   IO-bound operations (WebRTC, Network) -> Use `async/await`.
    *   CPU-bound or blocking legacy calls -> Use `run_in_executor` or separate threads.
    *   **Main Loop:** `asyncio.run(main_async())` handles the orchestration.

## 3. Python Conventions
*   **Type Hinting:** Required.
*   **Docstrings:** Google Style.
*   **Error Handling:** Catch specific exceptions (`KeyboardInterrupt`, `ConnectionError`).

## 4. Stability
This acts as a Driver/HAL for the main BattleBang system.
*   **Robustness:** It must NOT crash even if the robot disconnects. It should attempt reconnect.
*   **Logging:** Use `node.get_logger()` for all logs.
