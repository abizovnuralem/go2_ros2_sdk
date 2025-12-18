#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

namespace lidar_accelator {

// (Phase B) WASM decode 대체/래퍼 구현 시 추가 예정

std::vector<float> decode_and_process(
    const uint8_t* compressed,
    std::size_t compressed_len,
    float res,
    const float origin[3],
    float intense_limiter,
    bool deduplicate,
    int downsample_step,
    int max_points,
    std::size_t* out_points);

}  // namespace lidar_accelator
