#pragma once

#include <cstdint>
#include <string>
#include <string_view>
#include <vector>

namespace foxglove {

std::string base64Encode(const std::string_view& input);

std::vector<unsigned char> base64Decode(const std::string& input);

}  // namespace foxglove
