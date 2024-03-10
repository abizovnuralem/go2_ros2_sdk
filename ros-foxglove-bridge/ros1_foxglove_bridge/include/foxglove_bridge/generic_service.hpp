#pragma once

#include <string>
#include <vector>

#include <ros/serialization.h>
#include <ros/service_traits.h>

namespace foxglove_bridge {

struct GenericService {
  std::string type;
  std::string md5sum;
  std::vector<uint8_t> data;

  template <typename Stream>
  inline void write(Stream& stream) const {
    std::memcpy(stream.getData(), data.data(), data.size());
  }

  template <typename Stream>
  inline void read(Stream& stream) {
    data.resize(stream.getLength());
    std::memcpy(data.data(), stream.getData(), stream.getLength());
  }
};

}  // namespace foxglove_bridge

namespace ros::service_traits {
template <>
struct MD5Sum<foxglove_bridge::GenericService> {
  static const char* value(const foxglove_bridge::GenericService& m) {
    return m.md5sum.c_str();
  }

  static const char* value() {
    return "*";
  }
};

template <>
struct DataType<foxglove_bridge::GenericService> {
  static const char* value(const foxglove_bridge::GenericService& m) {
    return m.type.c_str();
  }

  static const char* value() {
    return "*";
  }
};
}  // namespace ros::service_traits

namespace ros::serialization {

template <>
struct Serializer<foxglove_bridge::GenericService> {
  template <typename Stream>
  inline static void write(Stream& stream, const foxglove_bridge::GenericService& m) {
    m.write(stream);
  }

  template <typename Stream>
  inline static void read(Stream& stream, foxglove_bridge::GenericService& m) {
    m.read(stream);
  }

  inline static uint32_t serializedLength(const foxglove_bridge::GenericService& m) {
    return m.data.size();
  }
};
}  // namespace ros::serialization
