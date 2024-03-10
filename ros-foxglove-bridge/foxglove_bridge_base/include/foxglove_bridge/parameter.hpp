#pragma once

#include <any>
#include <stdint.h>
#include <string>
#include <unordered_map>
#include <vector>

namespace foxglove {

enum class ParameterSubscriptionOperation {
  SUBSCRIBE,
  UNSUBSCRIBE,
};

enum class ParameterType {
  PARAMETER_NOT_SET,
  PARAMETER_BOOL,
  PARAMETER_INTEGER,
  PARAMETER_DOUBLE,
  PARAMETER_STRING,
  PARAMETER_ARRAY,
  PARAMETER_STRUCT,      // ROS 1 only
  PARAMETER_BYTE_ARRAY,  // ROS 2 only
};

class ParameterValue {
public:
  ParameterValue();
  ParameterValue(bool value);
  ParameterValue(int value);
  ParameterValue(int64_t value);
  ParameterValue(double value);
  ParameterValue(const std::string& value);
  ParameterValue(const char* value);
  ParameterValue(const std::vector<unsigned char>& value);
  ParameterValue(const std::vector<ParameterValue>& value);
  ParameterValue(const std::unordered_map<std::string, ParameterValue>& value);

  inline ParameterType getType() const {
    return _type;
  }

  template <typename T>
  inline const T& getValue() const {
    return std::any_cast<const T&>(_value);
  }

private:
  ParameterType _type;
  std::any _value;
};

class Parameter {
public:
  Parameter();
  Parameter(const std::string& name);
  Parameter(const std::string& name, const ParameterValue& value);

  inline const std::string& getName() const {
    return _name;
  }

  inline ParameterType getType() const {
    return _value.getType();
  }

  inline const ParameterValue& getValue() const {
    return _value;
  }

private:
  std::string _name;
  ParameterValue _value;
};

}  // namespace foxglove
