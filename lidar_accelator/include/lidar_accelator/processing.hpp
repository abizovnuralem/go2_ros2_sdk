#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

namespace lidar_accelator {

std::vector<float> process_u8_to_xyzi_f32(
    const uint8_t* positions_u8,
    std::size_t positions_len,
    const uint8_t* uvs_u8,
    std::size_t uvs_len,
    float res,
    const float origin[3],
    float intense_limiter,
    bool deduplicate,
    int downsample_step,
    int max_points,
    std::size_t* out_points);

}  // namespace lidar_accelator
