#pragma once

#include <algorithm>
#include <regex>
#include <string>
#include <vector>

namespace foxglove {

inline bool isWhitelisted(const std::string& name, const std::vector<std::regex>& regexPatterns) {
  return std::find_if(regexPatterns.begin(), regexPatterns.end(), [name](const auto& regex) {
           return std::regex_match(name, regex);
         }) != regexPatterns.end();
}

}  // namespace foxglove
