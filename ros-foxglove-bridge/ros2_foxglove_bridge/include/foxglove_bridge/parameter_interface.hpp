#pragma once

#include <chrono>
#include <functional>
#include <mutex>
#include <regex>
#include <string>
#include <unordered_set>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <foxglove_bridge/parameter.hpp>

namespace foxglove_bridge {

using ParameterList = std::vector<foxglove::Parameter>;
using ParamUpdateFunc = std::function<void(const ParameterList&)>;

class ParameterInterface {
public:
  ParameterInterface(rclcpp::Node* node, std::vector<std::regex> paramWhitelistPatterns);

  ParameterList getParams(const std::vector<std::string>& paramNames,
                          const std::chrono::duration<double>& timeout);
  void setParams(const ParameterList& params, const std::chrono::duration<double>& timeout);
  void subscribeParams(const std::vector<std::string>& paramNames);
  void unsubscribeParams(const std::vector<std::string>& paramNames);
  void setParamUpdateCallback(ParamUpdateFunc paramUpdateFunc);

private:
  rclcpp::Node* _node;
  std::vector<std::regex> _paramWhitelistPatterns;
  rclcpp::CallbackGroup::SharedPtr _callbackGroup;
  std::mutex _mutex;
  std::unordered_map<std::string, rclcpp::AsyncParametersClient::SharedPtr> _paramClientsByNode;
  std::unordered_map<std::string, std::unordered_set<std::string>> _subscribedParamsByNode;
  std::unordered_map<std::string, rclcpp::SubscriptionBase::SharedPtr> _paramSubscriptionsByNode;
  ParamUpdateFunc _paramUpdateFunc;

  ParameterList getNodeParameters(rclcpp::AsyncParametersClient::SharedPtr paramClient,
                                  const std::string& nodeName,
                                  const std::vector<std::string>& paramNames,
                                  const std::chrono::duration<double>& timeout);
  void setNodeParameters(rclcpp::AsyncParametersClient::SharedPtr paramClient,
                         const std::string& nodeName, const std::vector<rclcpp::Parameter>& params,
                         const std::chrono::duration<double>& timeout);
  bool isWhitelistedParam(const std::string& paramName);
};

}  // namespace foxglove_bridge
