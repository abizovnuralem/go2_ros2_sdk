#include "foxglove_bridge/message_definition_cache.hpp"

#include <filesystem>
#include <fstream>
#include <optional>
#include <regex>
#include <set>
#include <string>
#include <unordered_set>
#include <utility>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <ament_index_cpp/get_resource.hpp>
#include <ament_index_cpp/get_resources.hpp>
#include <rcutils/logging_macros.h>

namespace foxglove {

// Match datatype names (foo_msgs/Bar or foo_msgs/msg/Bar or foo_msgs/srv/Bar)
static const std::regex PACKAGE_TYPENAME_REGEX{
  R"(^([a-zA-Z0-9_]+)/(?:(msg|srv|action)/)?([a-zA-Z0-9_]+)$)"};

// Match field types from .msg definitions ("foo_msgs/Bar" in "foo_msgs/Bar[] bar")
static const std::regex MSG_FIELD_TYPE_REGEX{R"((?:^|\n)\s*([a-zA-Z0-9_/]+)(?:\[[^\]]*\])?\s+)"};

// match field types from `.idl` definitions ("foo_msgs/msg/bar" in #include <foo_msgs/msg/Bar.idl>)
static const std::regex IDL_FIELD_TYPE_REGEX{
  R"((?:^|\n)#include\s+(?:"|<)([a-zA-Z0-9_/]+)\.idl(?:"|>))"};

static const std::unordered_set<std::string> PRIMITIVE_TYPES{
  "bool",  "byte",   "char",  "float32", "float64", "int8",   "uint8",
  "int16", "uint16", "int32", "uint32",  "int64",   "uint64", "string"};

static std::set<std::string> parse_msg_dependencies(const std::string& text,
                                                    const std::string& package_context) {
  std::set<std::string> dependencies;

  for (std::sregex_iterator iter(text.begin(), text.end(), MSG_FIELD_TYPE_REGEX);
       iter != std::sregex_iterator(); ++iter) {
    std::string type = (*iter)[1];
    if (PRIMITIVE_TYPES.find(type) != PRIMITIVE_TYPES.end()) {
      continue;
    }
    if (type.find('/') == std::string::npos) {
      dependencies.insert(package_context + '/' + std::move(type));
    } else {
      dependencies.insert(std::move(type));
    }
  }
  return dependencies;
}

static std::set<std::string> parse_idl_dependencies(const std::string& text) {
  std::set<std::string> dependencies;

  for (std::sregex_iterator iter(text.begin(), text.end(), IDL_FIELD_TYPE_REGEX);
       iter != std::sregex_iterator(); ++iter) {
    dependencies.insert((*iter)[1]);
  }
  return dependencies;
}

std::set<std::string> parse_dependencies(MessageDefinitionFormat format, const std::string& text,
                                         const std::string& package_context) {
  switch (format) {
    case MessageDefinitionFormat::MSG:
      return parse_msg_dependencies(text, package_context);
    case MessageDefinitionFormat::IDL:
      return parse_idl_dependencies(text);
    default:
      throw std::runtime_error("switch is not exhaustive");
  }
}

static const char* extension_for_format(MessageDefinitionFormat format) {
  switch (format) {
    case MessageDefinitionFormat::MSG:
      return ".msg";
    case MessageDefinitionFormat::IDL:
      return ".idl";
    default:
      throw std::runtime_error("switch is not exhaustive");
  }
}

static std::string delimiter(const DefinitionIdentifier& definition_identifier) {
  std::string result =
    "================================================================================\n";
  switch (definition_identifier.format) {
    case MessageDefinitionFormat::MSG:
      result += "MSG: ";
      break;
    case MessageDefinitionFormat::IDL:
      result += "IDL: ";
      break;
    default:
      throw std::runtime_error("switch is not exhaustive");
  }
  result += definition_identifier.package_resource_name;
  result += "\n";
  return result;
}

static std::vector<std::string> split_string(const std::string& str,
                                             const std::string& delimiter = "\n") {
  std::vector<std::string> strings;
  std::string::size_type pos = 0;
  std::string::size_type prev = 0;

  while ((pos = str.find(delimiter, prev)) != std::string::npos) {
    strings.push_back(str.substr(prev, pos - prev));
    prev = pos + delimiter.size();
  }

  // Get the last substring (or only, if delimiter is not found)
  strings.push_back(str.substr(prev));

  return strings;
}

/// @brief Split an action definition into individual goal, result and feedback definitions.
/// @param action_definition The full action definition as read from a .action file
/// @return A tuple holding goal, result and feedback definitions
static std::tuple<std::string, std::string, std::string> split_action_msg_definition(
  const std::string& action_definition) {
  constexpr char SEP[] = "\n---\n";

  const auto definitions = split_string(action_definition, SEP);
  if (definitions.size() != 3) {
    throw std::invalid_argument("Invalid action definition:\n" + action_definition);
  }

  return {definitions[0], definitions[1], definitions[2]};
}

inline bool ends_with(const std::string& str, const std::string& suffix) {
  return str.size() >= suffix.size() &&
         0 == str.compare(str.size() - suffix.size(), suffix.size(), suffix);
}

std::string remove_action_subtype(const std::string action_type) {
  const auto action_subtype_suffixes = {
    std::string(ACTION_FEEDBACK_MESSAGE_SUFFIX),
    std::string(ACTION_RESULT_SERVICE_SUFFIX) + SERVICE_REQUEST_MESSAGE_SUFFIX,
    std::string(ACTION_RESULT_SERVICE_SUFFIX) + SERVICE_RESPONSE_MESSAGE_SUFFIX,
    std::string(ACTION_GOAL_SERVICE_SUFFIX) + SERVICE_REQUEST_MESSAGE_SUFFIX,
    std::string(ACTION_GOAL_SERVICE_SUFFIX) + SERVICE_RESPONSE_MESSAGE_SUFFIX,
  };

  for (const auto& suffix : action_subtype_suffixes) {
    if (ends_with(action_type, suffix)) {
      return action_type.substr(0, action_type.length() - suffix.length());
    }
  }

  return action_type;
}

MessageSpec::MessageSpec(MessageDefinitionFormat format, std::string text,
                         const std::string& package_context)
    : dependencies(parse_dependencies(format, text, package_context))
    , text(std::move(text))
    , format(format) {}

const MessageSpec& MessageDefinitionCache::load_message_spec(
  const DefinitionIdentifier& definition_identifier) {
  if (auto it = msg_specs_by_definition_identifier_.find(definition_identifier);
      it != msg_specs_by_definition_identifier_.end()) {
    return it->second;
  }
  std::smatch match;
  if (!std::regex_match(definition_identifier.package_resource_name, match,
                        PACKAGE_TYPENAME_REGEX)) {
    throw std::invalid_argument("Invalid package resource name: " +
                                definition_identifier.package_resource_name);
  }
  const std::string package = match[1].str();
  const std::string subfolder = match[2].str();
  const std::string type_name = match[3].str();
  const bool is_action_type = subfolder == "action";

  // The action type name includes the subtype which we have to remove to get the action name.
  // Type name: Fibonacci_FeedbackMessage -> Action name: Fibonacci
  const std::string action_name = is_action_type ? remove_action_subtype(type_name) : "";
  const std::string filename = is_action_type
                                 ? action_name + ".action"
                                 : type_name + extension_for_format(definition_identifier.format);

  // Get the package share directory, or throw a PackageNotFoundError
  const std::string share_dir = ament_index_cpp::get_package_share_directory(package);

  // Get the rosidl_interfaces index contents for this package
  std::string index_contents;
  if (!ament_index_cpp::get_resource("rosidl_interfaces", package, index_contents)) {
    throw DefinitionNotFoundError(definition_identifier.package_resource_name);
  }

  // Find the first line that ends with the filename we're looking for
  const auto lines = split_string(index_contents);
  const auto it = std::find_if(lines.begin(), lines.end(), [&filename](const std::string& line) {
    std::filesystem::path filePath(line);
    return filePath.filename() == filename;
  });
  if (it == lines.end()) {
    throw DefinitionNotFoundError(definition_identifier.package_resource_name);
  }

  // Read the file
  const std::string full_path = share_dir + std::filesystem::path::preferred_separator + *it;
  std::ifstream file{full_path};
  if (!file.good()) {
    throw DefinitionNotFoundError(definition_identifier.package_resource_name);
  }
  const std::string contents{std::istreambuf_iterator(file), {}};

  if (is_action_type) {
    if (definition_identifier.format == MessageDefinitionFormat::MSG) {
      const auto [goalDef, resultDef, feedbackDef] = split_action_msg_definition(contents);

      // Define type definitions for each action subtype.
      // These type definitions may include additional fields such as the goal_id.
      // See also https://design.ros2.org/articles/actions.html
      const std::map<std::string, std::string> action_type_definitions = {
        {ACTION_FEEDBACK_MESSAGE_SUFFIX, "unique_identifier_msgs/UUID goal_id\n" + feedbackDef},
        {std::string(ACTION_RESULT_SERVICE_SUFFIX) + SERVICE_REQUEST_MESSAGE_SUFFIX,
         "unique_identifier_msgs/UUID goal_id\n"},
        {std::string(ACTION_RESULT_SERVICE_SUFFIX) + SERVICE_RESPONSE_MESSAGE_SUFFIX,
         "int8 status\n" + resultDef},
        {std::string(ACTION_GOAL_SERVICE_SUFFIX) + SERVICE_REQUEST_MESSAGE_SUFFIX,
         "unique_identifier_msgs/UUID goal_id\n" + goalDef},
        {std::string(ACTION_GOAL_SERVICE_SUFFIX) + SERVICE_RESPONSE_MESSAGE_SUFFIX,
         "bool accepted\nbuiltin_interfaces/msg/Time stamp"}};

      // Create a MessageSpec instance for every action subtype and add it to the cache.
      for (const auto& [action_suffix, definition] : action_type_definitions) {
        DefinitionIdentifier definition_id;
        definition_id.format = definition_identifier.format;
        definition_id.package_resource_name = package + "/action/" + action_name + action_suffix;
        msg_specs_by_definition_identifier_.emplace(
          definition_id, MessageSpec(definition_id.format, definition, package));
      }

      // Find the the subtype that was originally requested and return it.
      const auto it = msg_specs_by_definition_identifier_.find(definition_identifier);
      if (it == msg_specs_by_definition_identifier_.end()) {
        throw DefinitionNotFoundError(definition_identifier.package_resource_name);
      }
      return it->second;
    } else {
      RCUTILS_LOG_ERROR_NAMED("foxglove_bridge",
                              "Action IDL definitions are currently not supported");
      throw DefinitionNotFoundError(definition_identifier.package_resource_name);
    }
  } else {
    // Normal message type.
    const MessageSpec& spec =
      msg_specs_by_definition_identifier_
        .emplace(definition_identifier,
                 MessageSpec(definition_identifier.format, std::move(contents), package))
        .first->second;

    // "References and pointers to data stored in the container are only invalidated by erasing that
    // element, even when the corresponding iterator is invalidated."
    return spec;
  }
}

std::pair<MessageDefinitionFormat, std::string> MessageDefinitionCache::get_full_text(
  const std::string& root_package_resource_name) {
  std::unordered_set<DefinitionIdentifier, DefinitionIdentifierHash> seen_deps;

  std::function<std::string(const DefinitionIdentifier&)> append_recursive =
    [&](const DefinitionIdentifier& definition_identifier) {
      const MessageSpec& spec = load_message_spec(definition_identifier);
      std::string result = spec.text;
      for (const auto& dep_name : spec.dependencies) {
        DefinitionIdentifier dep{definition_identifier.format, dep_name};
        bool inserted = seen_deps.insert(dep).second;
        if (inserted) {
          result += "\n";
          result += delimiter(dep);
          result += append_recursive(dep);
        }
      }
      return result;
    };

  std::string result;
  auto format = MessageDefinitionFormat::MSG;
  try {
    result = append_recursive(DefinitionIdentifier{format, root_package_resource_name});
  } catch (const DefinitionNotFoundError& err) {
    // log that we've fallen back
    RCUTILS_LOG_WARN_NAMED("foxglove_bridge", "no .msg definition for %s, falling back to IDL",
                           err.what());
    format = MessageDefinitionFormat::IDL;
    DefinitionIdentifier root_definition_identifier{format, root_package_resource_name};
    result = delimiter(root_definition_identifier) + append_recursive(root_definition_identifier);
  }
  return std::make_pair(format, result);
}

}  // namespace foxglove
