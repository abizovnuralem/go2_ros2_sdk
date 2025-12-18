#include "lidar_accelator/processing.hpp"

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <algorithm>
#include <vector>

namespace lidar_accelator {

namespace {

inline bool row_less_lex(const std::array<float, 4>& a, const std::array<float, 4>& b) {
  if (a[0] != b[0]) return a[0] < b[0];
  if (a[1] != b[1]) return a[1] < b[1];
  if (a[2] != b[2]) return a[2] < b[2];
  return a[3] < b[3];
}

}  // namespace

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
    std::size_t* out_points) {
  if (out_points) {
    *out_points = 0;
  }

  if (!positions_u8 || !uvs_u8) {
    return {};
  }
  if (positions_len % 3 != 0) {
    return {};
  }
  if (uvs_len % 2 != 0) {
    return {};
  }

  const std::size_t n_pos = positions_len / 3;
  const std::size_t n_uv = uvs_len / 2;
  const std::size_t n = (n_pos < n_uv) ? n_pos : n_uv;

  if (n == 0) {
    return {};
  }

  if (downsample_step <= 0) {
    downsample_step = 1;
  }

  std::vector<std::array<float, 4>> filtered;
  filtered.reserve(n);

  for (std::size_t i = 0; i < n; ++i) {
    const float u = static_cast<float>(uvs_u8[i * 2 + 0]);
    const float v = static_cast<float>(uvs_u8[i * 2 + 1]);
    const float intensity = (u < v) ? u : v;

    if (intensity <= intense_limiter) {
      continue;
    }

    const float x = static_cast<float>(positions_u8[i * 3 + 0]) * res + origin[0];
    const float y = static_cast<float>(positions_u8[i * 3 + 1]) * res + origin[1];
    const float z = static_cast<float>(positions_u8[i * 3 + 2]) * res + origin[2];

    filtered.push_back({x, y, z, intensity});
  }

  if (filtered.empty()) {
    return {};
  }

  // Python semantics: downsample AFTER filtering with [::downsample_step]
  if (downsample_step > 1) {
    std::vector<std::array<float, 4>> down;
    down.reserve((filtered.size() + static_cast<std::size_t>(downsample_step) - 1) /
                 static_cast<std::size_t>(downsample_step));
    for (std::size_t k = 0; k < filtered.size(); k += static_cast<std::size_t>(downsample_step)) {
      down.push_back(filtered[k]);
    }
    filtered.swap(down);
  }

  if (max_points > 0 && static_cast<std::size_t>(max_points) < filtered.size()) {
    std::vector<std::array<float, 4>> limited;
    limited.reserve(static_cast<std::size_t>(max_points));

    const std::size_t total = filtered.size();
    const std::size_t m = static_cast<std::size_t>(max_points);
    if (m == 1) {
      limited.push_back(filtered.front());
    } else {
      for (std::size_t k = 0; k < m; ++k) {
        // Python semantics: idx = np.linspace(0, total-1, num=m, dtype=int64)
        // Numpy casts float->int by truncation toward zero (floor for non-negative).
        const double t = static_cast<double>(k) / static_cast<double>(m - 1);
        const std::size_t idx = static_cast<std::size_t>(t * static_cast<double>(total - 1));
        limited.push_back(filtered[idx]);
      }
    }
    filtered.swap(limited);
  }

  if (deduplicate) {
    // Python semantics: np.unique(..., axis=0) returns sorted unique rows.
    std::sort(filtered.begin(), filtered.end(), row_less_lex);
    auto last = std::unique(filtered.begin(), filtered.end());
    filtered.erase(last, filtered.end());
  }

  std::vector<float> out;
  out.resize(filtered.size() * 4);

  for (std::size_t i = 0; i < filtered.size(); ++i) {
    out[i * 4 + 0] = filtered[i][0];
    out[i * 4 + 1] = filtered[i][1];
    out[i * 4 + 2] = filtered[i][2];
    out[i * 4 + 3] = filtered[i][3];
  }

  if (out_points) {
    *out_points = filtered.size();
  }

  return out;
}

}  // namespace lidar_accelator
