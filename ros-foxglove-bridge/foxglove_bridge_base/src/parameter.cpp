#include <foxglove_bridge/parameter.hpp>

namespace foxglove {

ParameterValue::ParameterValue()
    : _type(ParameterType::PARAMETER_NOT_SET) {}
ParameterValue::ParameterValue(bool value)
    : _type(ParameterType::PARAMETER_BOOL)
    , _value(value) {}
ParameterValue::ParameterValue(int value)
    : _type(ParameterType::PARAMETER_INTEGER)
    , _value(static_cast<int64_t>(value)) {}
ParameterValue::ParameterValue(int64_t value)
    : _type(ParameterType::PARAMETER_INTEGER)
    , _value(value) {}
ParameterValue::ParameterValue(double value)
    : _type(ParameterType::PARAMETER_DOUBLE)
    , _value(value) {}
ParameterValue::ParameterValue(const std::string& value)
    : _type(ParameterType::PARAMETER_STRING)
    , _value(value) {}
ParameterValue::ParameterValue(const char* value)
    : _type(ParameterType::PARAMETER_STRING)
    , _value(std::string(value)) {}
ParameterValue::ParameterValue(const std::vector<unsigned char>& value)
    : _type(ParameterType::PARAMETER_BYTE_ARRAY)
    , _value(value) {}
ParameterValue::ParameterValue(const std::vector<ParameterValue>& value)
    : _type(ParameterType::PARAMETER_ARRAY)
    , _value(value) {}
ParameterValue::ParameterValue(const std::unordered_map<std::string, ParameterValue>& value)
    : _type(ParameterType::PARAMETER_STRUCT)
    , _value(value) {}

Parameter::Parameter() {}
Parameter::Parameter(const std::string& name)
    : _name(name)
    , _value(ParameterValue()) {}
Parameter::Parameter(const std::string& name, const ParameterValue& value)
    : _name(name)
    , _value(value) {}

}  // namespace foxglove
