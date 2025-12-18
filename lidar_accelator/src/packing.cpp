#include "lidar_accelator/packing.hpp"

#include <cstddef>
#include <cstdint>
#include <string>

namespace lidar_accelator {

std::string pack_xyzi_f32_to_bytes(const float* xyzi_f32, std::size_t count_points) {
  if (!xyzi_f32 || count_points == 0) {
    return {};
  }

  const std::size_t n_floats = count_points * 4;
  const std::size_t n_bytes = n_floats * sizeof(float);

  return std::string(reinterpret_cast<const char*>(xyzi_f32), n_bytes);
}

}  // namespace lidar_accelator
