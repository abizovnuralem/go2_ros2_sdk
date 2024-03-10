#pragma once

#include <stdint.h>

#include <nlohmann/json.hpp>

#include "common.hpp"
#include "parameter.hpp"

namespace foxglove {

inline void WriteUint64LE(uint8_t* buf, uint64_t val) {
#ifdef ARCH_IS_BIG_ENDIAN
  buf[0] = val & 0xff;
  buf[1] = (val >> 8) & 0xff;
  buf[2] = (val >> 16) & 0xff;
  buf[3] = (val >> 24) & 0xff;
  buf[4] = (val >> 32) & 0xff;
  buf[5] = (val >> 40) & 0xff;
  buf[6] = (val >> 48) & 0xff;
  buf[7] = (val >> 56) & 0xff;
#else
  reinterpret_cast<uint64_t*>(buf)[0] = val;
#endif
}

inline void WriteUint32LE(uint8_t* buf, uint32_t val) {
#ifdef ARCH_IS_BIG_ENDIAN
  buf[0] = val & 0xff;
  buf[1] = (val >> 8) & 0xff;
  buf[2] = (val >> 16) & 0xff;
  buf[3] = (val >> 24) & 0xff;
#else
  reinterpret_cast<uint32_t*>(buf)[0] = val;
#endif
}

inline uint32_t ReadUint32LE(const uint8_t* buf) {
#ifdef ARCH_IS_BIG_ENDIAN
  uint32_t val = (bytes[0] << 24) + (bytes[1] << 16) + (bytes[2] << 8) + bytes[3];
  return val;
#else
  return reinterpret_cast<const uint32_t*>(buf)[0];
#endif
}

void to_json(nlohmann::json& j, const Channel& c);
void from_json(const nlohmann::json& j, Channel& c);
void to_json(nlohmann::json& j, const ParameterValue& p);
void from_json(const nlohmann::json& j, ParameterValue& p);
void to_json(nlohmann::json& j, const Parameter& p);
void from_json(const nlohmann::json& j, Parameter& p);
void to_json(nlohmann::json& j, const Service& p);
void from_json(const nlohmann::json& j, Service& p);

}  // namespace foxglove
