#pragma once

#include <set>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>

namespace foxglove {

// Taken from
// https://github.com/ros2/rosidl/blob/a57baea5/rosidl_parser/rosidl_parser/definition.py
constexpr char SERVICE_REQUEST_MESSAGE_SUFFIX[] = "_Request";
constexpr char SERVICE_RESPONSE_MESSAGE_SUFFIX[] = "_Response";
constexpr char ACTION_GOAL_SERVICE_SUFFIX[] = "_SendGoal";
constexpr char ACTION_RESULT_SERVICE_SUFFIX[] = "_GetResult";
constexpr char ACTION_FEEDBACK_MESSAGE_SUFFIX[] = "_FeedbackMessage";

enum struct MessageDefinitionFormat {
  IDL,
  MSG,
};

struct MessageSpec {
  MessageSpec(MessageDefinitionFormat format, std::string text, const std::string& package_context);
  const std::set<std::string> dependencies;
  const std::string text;
  MessageDefinitionFormat format;
};

struct DefinitionIdentifier {
  MessageDefinitionFormat format;
  std::string package_resource_name;

  bool operator==(const DefinitionIdentifier& di) const {
    return (format == di.format) && (package_resource_name == di.package_resource_name);
  }
};

class DefinitionNotFoundError : public std::exception {
private:
  std::string name_;

public:
  explicit DefinitionNotFoundError(std::string name)
      : name_(std::move(name)) {}

  const char* what() const noexcept override {
    return name_.c_str();
  }
};

class MessageDefinitionCache final {
public:
  /**
   * Concatenate the message definition with its dependencies into a self-contained schema.
   * The format is different for MSG and IDL definitions, and is described fully here:
   * [MSG](https://mcap.dev/specification/appendix.html#ros2msg-data-format)
   * [IDL](https://mcap.dev/specification/appendix.html#ros2idl-data-format)
   * Throws DefinitionNotFoundError if one or more definition files are missing for the given
   * package resource name.
   */
  std::pair<MessageDefinitionFormat, std::string> get_full_text(
    const std::string& package_resource_name);

private:
  struct DefinitionIdentifierHash {
    std::size_t operator()(const DefinitionIdentifier& di) const {
      std::size_t h1 = std::hash<MessageDefinitionFormat>()(di.format);
      std::size_t h2 = std::hash<std::string>()(di.package_resource_name);
      return h1 ^ h2;
    }
  };
  /**
   * Load and parse the message file referenced by the given datatype, or return it from
   * msg_specs_by_datatype
   */
  const MessageSpec& load_message_spec(const DefinitionIdentifier& definition_identifier);

  std::unordered_map<DefinitionIdentifier, MessageSpec, DefinitionIdentifierHash>
    msg_specs_by_definition_identifier_;
};

std::set<std::string> parse_dependencies(MessageDefinitionFormat format, const std::string& text,
                                         const std::string& package_context);

}  // namespace foxglove
