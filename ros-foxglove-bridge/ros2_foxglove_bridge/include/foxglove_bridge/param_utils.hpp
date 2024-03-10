#pragma once

#include <regex>
#include <string>
#include <vector>

#include <rclcpp/node.hpp>

namespace foxglove_bridge {

constexpr char PARAM_PORT[] = "port";
constexpr char PARAM_ADDRESS[] = "address";
constexpr char PARAM_SEND_BUFFER_LIMIT[] = "send_buffer_limit";
constexpr char PARAM_USETLS[] = "tls";
constexpr char PARAM_CERTFILE[] = "certfile";
constexpr char PARAM_KEYFILE[] = "keyfile";
constexpr char PARAM_MIN_QOS_DEPTH[] = "min_qos_depth";
constexpr char PARAM_MAX_QOS_DEPTH[] = "max_qos_depth";
constexpr char PARAM_TOPIC_WHITELIST[] = "topic_whitelist";
constexpr char PARAM_SERVICE_WHITELIST[] = "service_whitelist";
constexpr char PARAM_PARAMETER_WHITELIST[] = "param_whitelist";
constexpr char PARAM_USE_COMPRESSION[] = "use_compression";
constexpr char PARAM_CAPABILITIES[] = "capabilities";
constexpr char PARAM_CLIENT_TOPIC_WHITELIST[] = "client_topic_whitelist";
constexpr char PARAM_INCLUDE_HIDDEN[] = "include_hidden";
constexpr char PARAM_ASSET_URI_ALLOWLIST[] = "asset_uri_allowlist";

constexpr int64_t DEFAULT_PORT = 8765;
constexpr char DEFAULT_ADDRESS[] = "0.0.0.0";
constexpr int64_t DEFAULT_SEND_BUFFER_LIMIT = 10000000;
constexpr int64_t DEFAULT_MIN_QOS_DEPTH = 1;
constexpr int64_t DEFAULT_MAX_QOS_DEPTH = 25;

void declareParameters(rclcpp::Node* node);

std::vector<std::regex> parseRegexStrings(rclcpp::Node* node,
                                          const std::vector<std::string>& strings);

}  // namespace foxglove_bridge
