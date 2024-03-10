

#include <foxglove_bridge/common.hpp>
#include <foxglove_bridge/param_utils.hpp>

namespace foxglove_bridge {

void declareParameters(rclcpp::Node* node) {
  auto portDescription = rcl_interfaces::msg::ParameterDescriptor{};
  portDescription.name = PARAM_PORT;
  portDescription.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
  portDescription.description = "The TCP port to bind the WebSocket server to";
  portDescription.read_only = true;
  portDescription.additional_constraints =
    "Must be a valid TCP port number, or 0 to use a random port";
  portDescription.integer_range.resize(1);
  portDescription.integer_range[0].from_value = 0;
  portDescription.integer_range[0].to_value = 65535;
  portDescription.integer_range[0].step = 1;
  node->declare_parameter(PARAM_PORT, DEFAULT_PORT, portDescription);

  auto addressDescription = rcl_interfaces::msg::ParameterDescriptor{};
  addressDescription.name = PARAM_ADDRESS;
  addressDescription.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
  addressDescription.description = "The host address to bind the WebSocket server to";
  addressDescription.read_only = true;
  node->declare_parameter(PARAM_ADDRESS, DEFAULT_ADDRESS, addressDescription);

  auto sendBufferLimitDescription = rcl_interfaces::msg::ParameterDescriptor{};
  sendBufferLimitDescription.name = PARAM_SEND_BUFFER_LIMIT;
  sendBufferLimitDescription.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
  sendBufferLimitDescription.description =
    "Connection send buffer limit in bytes. Messages will be dropped when a connection's send "
    "buffer reaches this limit to avoid a queue of outdated messages building up.";
  sendBufferLimitDescription.integer_range.resize(1);
  sendBufferLimitDescription.integer_range[0].from_value = 0;
  sendBufferLimitDescription.integer_range[0].to_value = std::numeric_limits<int64_t>::max();
  sendBufferLimitDescription.read_only = true;
  node->declare_parameter(PARAM_SEND_BUFFER_LIMIT, DEFAULT_SEND_BUFFER_LIMIT,
                          sendBufferLimitDescription);

  auto useTlsDescription = rcl_interfaces::msg::ParameterDescriptor{};
  useTlsDescription.name = PARAM_USETLS;
  useTlsDescription.type = rcl_interfaces::msg::ParameterType::PARAMETER_BOOL;
  useTlsDescription.description = "Use Transport Layer Security for encrypted communication";
  useTlsDescription.read_only = true;
  node->declare_parameter(PARAM_USETLS, false, useTlsDescription);

  auto certfileDescription = rcl_interfaces::msg::ParameterDescriptor{};
  certfileDescription.name = PARAM_CERTFILE;
  certfileDescription.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
  certfileDescription.description = "Path to the certificate to use for TLS";
  certfileDescription.read_only = true;
  node->declare_parameter(PARAM_CERTFILE, "", certfileDescription);

  auto keyfileDescription = rcl_interfaces::msg::ParameterDescriptor{};
  keyfileDescription.name = PARAM_KEYFILE;
  keyfileDescription.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
  keyfileDescription.description = "Path to the private key to use for TLS";
  keyfileDescription.read_only = true;
  node->declare_parameter(PARAM_KEYFILE, "", keyfileDescription);

  auto minQosDepthDescription = rcl_interfaces::msg::ParameterDescriptor{};
  minQosDepthDescription.name = PARAM_MIN_QOS_DEPTH;
  minQosDepthDescription.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
  minQosDepthDescription.description = "Minimum depth used for the QoS profile of subscriptions.";
  minQosDepthDescription.read_only = true;
  minQosDepthDescription.additional_constraints = "Must be a non-negative integer";
  minQosDepthDescription.integer_range.resize(1);
  minQosDepthDescription.integer_range[0].from_value = 0;
  minQosDepthDescription.integer_range[0].to_value = INT32_MAX;
  minQosDepthDescription.integer_range[0].step = 1;
  node->declare_parameter(PARAM_MIN_QOS_DEPTH, DEFAULT_MIN_QOS_DEPTH, minQosDepthDescription);

  auto maxQosDepthDescription = rcl_interfaces::msg::ParameterDescriptor{};
  maxQosDepthDescription.name = PARAM_MAX_QOS_DEPTH;
  maxQosDepthDescription.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
  maxQosDepthDescription.description = "Maximum depth used for the QoS profile of subscriptions.";
  maxQosDepthDescription.read_only = true;
  maxQosDepthDescription.additional_constraints = "Must be a non-negative integer";
  maxQosDepthDescription.integer_range.resize(1);
  maxQosDepthDescription.integer_range[0].from_value = 0;
  maxQosDepthDescription.integer_range[0].to_value = INT32_MAX;
  maxQosDepthDescription.integer_range[0].step = 1;
  node->declare_parameter(PARAM_MAX_QOS_DEPTH, DEFAULT_MAX_QOS_DEPTH, maxQosDepthDescription);

  auto topicWhiteListDescription = rcl_interfaces::msg::ParameterDescriptor{};
  topicWhiteListDescription.name = PARAM_TOPIC_WHITELIST;
  topicWhiteListDescription.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING_ARRAY;
  topicWhiteListDescription.description =
    "List of regular expressions (ECMAScript) of whitelisted topic names.";
  topicWhiteListDescription.read_only = true;
  node->declare_parameter(PARAM_TOPIC_WHITELIST, std::vector<std::string>({".*"}),
                          topicWhiteListDescription);

  auto serviceWhiteListDescription = rcl_interfaces::msg::ParameterDescriptor{};
  serviceWhiteListDescription.name = PARAM_SERVICE_WHITELIST;
  serviceWhiteListDescription.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING_ARRAY;
  serviceWhiteListDescription.description =
    "List of regular expressions (ECMAScript) of whitelisted service names.";
  serviceWhiteListDescription.read_only = true;
  node->declare_parameter(PARAM_SERVICE_WHITELIST, std::vector<std::string>({".*"}),
                          serviceWhiteListDescription);

  auto paramWhiteListDescription = rcl_interfaces::msg::ParameterDescriptor{};
  paramWhiteListDescription.name = PARAM_PARAMETER_WHITELIST;
  paramWhiteListDescription.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING_ARRAY;
  paramWhiteListDescription.description =
    "List of regular expressions (ECMAScript) of whitelisted parameter names.";
  paramWhiteListDescription.read_only = true;
  node->declare_parameter(PARAM_PARAMETER_WHITELIST, std::vector<std::string>({".*"}),
                          paramWhiteListDescription);

  auto useCompressionDescription = rcl_interfaces::msg::ParameterDescriptor{};
  useCompressionDescription.name = PARAM_USE_COMPRESSION;
  useCompressionDescription.type = rcl_interfaces::msg::ParameterType::PARAMETER_BOOL;
  useCompressionDescription.description =
    "Use websocket compression (permessage-deflate). Suited for connections with smaller bandwith, "
    "at the cost of additional CPU load.";
  useCompressionDescription.read_only = true;
  node->declare_parameter(PARAM_USE_COMPRESSION, false, useCompressionDescription);

  auto paramCapabilities = rcl_interfaces::msg::ParameterDescriptor{};
  paramCapabilities.name = PARAM_CAPABILITIES;
  paramCapabilities.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING_ARRAY;
  paramCapabilities.description = "Server capabilities";
  paramCapabilities.read_only = true;
  node->declare_parameter(
    PARAM_CAPABILITIES,
    std::vector<std::string>(std::vector<std::string>(foxglove::DEFAULT_CAPABILITIES.begin(),
                                                      foxglove::DEFAULT_CAPABILITIES.end())),
    paramCapabilities);

  auto clientTopicWhiteListDescription = rcl_interfaces::msg::ParameterDescriptor{};
  clientTopicWhiteListDescription.name = PARAM_CLIENT_TOPIC_WHITELIST;
  clientTopicWhiteListDescription.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING_ARRAY;
  clientTopicWhiteListDescription.description =
    "List of regular expressions (ECMAScript) of whitelisted client-published topic names.";
  clientTopicWhiteListDescription.read_only = true;
  node->declare_parameter(PARAM_CLIENT_TOPIC_WHITELIST, std::vector<std::string>({".*"}),
                          paramWhiteListDescription);

  auto includeHiddenDescription = rcl_interfaces::msg::ParameterDescriptor{};
  includeHiddenDescription.name = PARAM_INCLUDE_HIDDEN;
  includeHiddenDescription.type = rcl_interfaces::msg::ParameterType::PARAMETER_BOOL;
  includeHiddenDescription.description = "Include hidden topics and services";
  includeHiddenDescription.read_only = true;
  node->declare_parameter(PARAM_INCLUDE_HIDDEN, false, includeHiddenDescription);

  auto assetUriAllowlistDescription = rcl_interfaces::msg::ParameterDescriptor{};
  assetUriAllowlistDescription.name = PARAM_ASSET_URI_ALLOWLIST;
  assetUriAllowlistDescription.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING_ARRAY;
  assetUriAllowlistDescription.description =
    "List of regular expressions (ECMAScript) of whitelisted asset URIs.";
  assetUriAllowlistDescription.read_only = true;
  node->declare_parameter(
    PARAM_ASSET_URI_ALLOWLIST,
    std::vector<std::string>(
      {"^package://(?:\\w+/"
       ")*\\w+\\.(?:dae|fbx|glb|gltf|jpeg|jpg|mtl|obj|png|stl|tif|tiff|urdf|webp|xacro)$"}),
    paramWhiteListDescription);
}

std::vector<std::regex> parseRegexStrings(rclcpp::Node* node,
                                          const std::vector<std::string>& strings) {
  std::vector<std::regex> regexVector;
  regexVector.reserve(strings.size());

  for (const auto& pattern : strings) {
    try {
      regexVector.push_back(
        std::regex(pattern, std::regex_constants::ECMAScript | std::regex_constants::icase));
    } catch (const std::exception& ex) {
      RCLCPP_ERROR(node->get_logger(), "Ignoring invalid regular expression '%s': %s",
                   pattern.c_str(), ex.what());
    }
  }

  return regexVector;
}

}  // namespace foxglove_bridge
