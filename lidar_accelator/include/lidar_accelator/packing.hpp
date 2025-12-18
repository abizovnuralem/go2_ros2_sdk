#pragma once

#include <cstddef>
#include <string>

namespace lidar_accelator {

// (optional) Phase A에서 PointCloud2 패킹까지 C++로 옮길 때 추가 예정

std::string pack_xyzi_f32_to_bytes(const float* xyzi_f32, std::size_t count_points);

}  // namespace lidar_accelator
