#include "foxglove_bridge/parameter_interface.hpp"

#include <nlohmann/json.hpp>

#include <foxglove_bridge/regex_utils.hpp>
#include <foxglove_bridge/utils.hpp>

namespace {

constexpr char PARAM_SEP = '.';

static std::pair<std::string, std::string> getNodeAndParamName(
  const std::string& nodeNameAndParamName) {
  return {nodeNameAndParamName.substr(0UL, nodeNameAndParamName.find(PARAM_SEP)),
          nodeNameAndParamName.substr(nodeNameAndParamName.find(PARAM_SEP) + 1UL)};
}

static std::string prependNodeNameToParamName(const std::string& paramName,
                                              const std::string& nodeName) {
  return nodeName + PARAM_SEP + paramName;
}

static rclcpp::Parameter toRosParam(const foxglove::Parameter& p) {
  using foxglove::Parameter;
  using foxglove::ParameterType;

  const auto paramType = p.getType();
  const auto value = p.getValue();

  if (paramType == ParameterType::PARAMETER_BOOL) {
    return rclcpp::Parameter(p.getName(), value.getValue<bool>());
  } else if (paramType == ParameterType::PARAMETER_INTEGER) {
    return rclcpp::Parameter(p.getName(), value.getValue<int64_t>());
  } else if (paramType == ParameterType::PARAMETER_DOUBLE) {
    return rclcpp::Parameter(p.getName(), value.getValue<double>());
  } else if (paramType == ParameterType::PARAMETER_STRING) {
    return rclcpp::Parameter(p.getName(), value.getValue<std::string>());
  } else if (paramType == ParameterType::PARAMETER_BYTE_ARRAY) {
    return rclcpp::Parameter(p.getName(), value.getValue<std::vector<unsigned char>>());
  } else if (paramType == ParameterType::PARAMETER_ARRAY) {
    const auto paramVec = value.getValue<std::vector<foxglove::ParameterValue>>();

    const auto elementType = paramVec.front().getType();
    if (elementType == ParameterType::PARAMETER_BOOL) {
      std::vector<bool> vec;
      for (const auto& paramValue : paramVec) {
        vec.push_back(paramValue.getValue<bool>());
      }
      return rclcpp::Parameter(p.getName(), vec);
    } else if (elementType == ParameterType::PARAMETER_INTEGER) {
      std::vector<int64_t> vec;
      for (const auto& paramValue : paramVec) {
        vec.push_back(paramValue.getValue<int64_t>());
      }
      return rclcpp::Parameter(p.getName(), vec);
    } else if (elementType == ParameterType::PARAMETER_DOUBLE) {
      std::vector<double> vec;
      for (const auto& paramValue : paramVec) {
        vec.push_back(paramValue.getValue<double>());
      }
      return rclcpp::Parameter(p.getName(), vec);
    } else if (elementType == ParameterType::PARAMETER_STRING) {
      std::vector<std::string> vec;
      for (const auto& paramValue : paramVec) {
        vec.push_back(paramValue.getValue<std::string>());
      }
      return rclcpp::Parameter(p.getName(), vec);
    }
    throw std::runtime_error("Unsupported parameter type");
  } else if (paramType == ParameterType::PARAMETER_NOT_SET) {
    return rclcpp::Parameter(p.getName());
  } else {
    throw std::runtime_error("Unsupported parameter type");
  }

  return rclcpp::Parameter();
}

static foxglove::Parameter fromRosParam(const rclcpp::Parameter& p) {
  const auto type = p.get_type();

  if (type == rclcpp::ParameterType::PARAMETER_NOT_SET) {
    return foxglove::Parameter(p.get_name(), foxglove::ParameterValue());
  } else if (type == rclcpp::ParameterType::PARAMETER_BOOL) {
    return foxglove::Parameter(p.get_name(), p.as_bool());
  } else if (type == rclcpp::ParameterType::PARAMETER_INTEGER) {
    return foxglove::Parameter(p.get_name(), p.as_int());
  } else if (type == rclcpp::ParameterType::PARAMETER_DOUBLE) {
    return foxglove::Parameter(p.get_name(), p.as_double());
  } else if (type == rclcpp::ParameterType::PARAMETER_STRING) {
    return foxglove::Parameter(p.get_name(), p.as_string());
  } else if (type == rclcpp::ParameterType::PARAMETER_BYTE_ARRAY) {
    return foxglove::Parameter(p.get_name(), p.as_byte_array());
  } else if (type == rclcpp::ParameterType::PARAMETER_BOOL_ARRAY) {
    std::vector<foxglove::ParameterValue> paramVec;
    for (const auto value : p.as_bool_array()) {
      paramVec.push_back(foxglove::ParameterValue(value));
    }
    return foxglove::Parameter(p.get_name(), paramVec);
  } else if (type == rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY) {
    std::vector<foxglove::ParameterValue> paramVec;
    for (const auto value : p.as_integer_array()) {
      paramVec.push_back(value);
    }
    return foxglove::Parameter(p.get_name(), paramVec);
  } else if (type == rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY) {
    std::vector<foxglove::ParameterValue> paramVec;
    for (const auto value : p.as_double_array()) {
      paramVec.push_back(value);
    }
    return foxglove::Parameter(p.get_name(), paramVec);
  } else if (type == rclcpp::ParameterType::PARAMETER_STRING_ARRAY) {
    std::vector<foxglove::ParameterValue> paramVec;
    for (const auto& value : p.as_string_array()) {
      paramVec.push_back(value);
    }
    return foxglove::Parameter(p.get_name(), paramVec);
  } else {
    throw std::runtime_error("Unsupported parameter type");
  }
}

}  // namespace

namespace foxglove_bridge {

using foxglove::isWhitelisted;

ParameterInterface::ParameterInterface(rclcpp::Node* node,
                                       std::vector<std::regex> paramWhitelistPatterns)
    : _node(node)
    , _paramWhitelistPatterns(paramWhitelistPatterns)
    , _callbackGroup(node->create_callback_group(rclcpp::CallbackGroupType::Reentrant)) {}

ParameterList ParameterInterface::getParams(const std::vector<std::string>& paramNames,
                                            const std::chrono::duration<double>& timeout) {
  std::lock_guard<std::mutex> lock(_mutex);

  std::unordered_map<std::string, std::vector<std::string>> paramNamesByNodeName;
  const auto thisNode = _node->get_fully_qualified_name();

  if (!paramNames.empty()) {
    // Break apart fully qualified {node_name}.{param_name} strings and build a
    // mape of node names to the list of parameters for each node
    for (const auto& fullParamName : paramNames) {
      const auto& [nodeName, paramName] = getNodeAndParamName(fullParamName);
      paramNamesByNodeName[nodeName].push_back(paramName);
    }

    RCLCPP_DEBUG(_node->get_logger(), "Getting %zu parameters from %zu nodes...", paramNames.size(),
                 paramNamesByNodeName.size());
  } else {
    // Make a map of node names to empty parameter lists
    // Only consider nodes that offer services to list & get parameters.
    for (const auto& fqnNodeName : _node->get_node_names()) {
      if (fqnNodeName == thisNode) {
        continue;
      }
      const auto [nodeNs, nodeName] = getNodeAndNodeNamespace(fqnNodeName);
      const auto serviceNamesAndTypes =
        _node->get_service_names_and_types_by_node(nodeName, nodeNs);

      bool listParamsSrvFound = false, getParamsSrvFound = false;
      for (const auto& [serviceName, serviceTypes] : serviceNamesAndTypes) {
        constexpr char GET_PARAMS_SERVICE_TYPE[] = "rcl_interfaces/srv/GetParameters";
        constexpr char LIST_PARAMS_SERVICE_TYPE[] = "rcl_interfaces/srv/ListParameters";

        if (!getParamsSrvFound) {
          getParamsSrvFound = std::find(serviceTypes.begin(), serviceTypes.end(),
                                        GET_PARAMS_SERVICE_TYPE) != serviceTypes.end();
        }
        if (!listParamsSrvFound) {
          listParamsSrvFound = std::find(serviceTypes.begin(), serviceTypes.end(),
                                         LIST_PARAMS_SERVICE_TYPE) != serviceTypes.end();
        }
      }

      if (listParamsSrvFound && getParamsSrvFound) {
        paramNamesByNodeName.insert({fqnNodeName, {}});
      }
    }

    if (!paramNamesByNodeName.empty()) {
      RCLCPP_DEBUG(_node->get_logger(), "Getting all parameters from %zu nodes...",
                   paramNamesByNodeName.size());
    }
  }

  std::vector<std::future<ParameterList>> getParametersFuture;
  for (const auto& [nodeName, nodeParamNames] : paramNamesByNodeName) {
    if (nodeName == thisNode) {
      continue;
    }

    auto paramClientIt = _paramClientsByNode.find(nodeName);
    if (paramClientIt == _paramClientsByNode.end()) {
      const auto insertedPair = _paramClientsByNode.emplace(
        nodeName, rclcpp::AsyncParametersClient::make_shared(
                    _node, nodeName, rmw_qos_profile_parameters, _callbackGroup));
      paramClientIt = insertedPair.first;
    }

    getParametersFuture.emplace_back(
      std::async(std::launch::async, &ParameterInterface::getNodeParameters, this,
                 paramClientIt->second, nodeName, nodeParamNames, timeout));
  }

  ParameterList result;
  for (auto& future : getParametersFuture) {
    try {
      const auto params = future.get();
      result.insert(result.begin(), params.begin(), params.end());
    } catch (const std::exception& e) {
      RCLCPP_ERROR(_node->get_logger(), "Exception when getting parameters: %s", e.what());
    }
  }

  return result;
}

void ParameterInterface::setParams(const ParameterList& parameters,
                                   const std::chrono::duration<double>& timeout) {
  std::lock_guard<std::mutex> lock(_mutex);

  rclcpp::ParameterMap paramsByNode;
  for (const auto& param : parameters) {
    if (!isWhitelisted(param.getName(), _paramWhitelistPatterns)) {
      return;
    }

    const auto rosParam = toRosParam(param);
    const auto& [nodeName, paramName] = getNodeAndParamName(rosParam.get_name());
    paramsByNode[nodeName].emplace_back(paramName, rosParam.get_parameter_value());
  }

  std::vector<std::future<void>> setParametersFuture;
  for (const auto& [nodeName, params] : paramsByNode) {
    auto paramClientIt = _paramClientsByNode.find(nodeName);
    if (paramClientIt == _paramClientsByNode.end()) {
      const auto insertedPair = _paramClientsByNode.emplace(
        nodeName, rclcpp::AsyncParametersClient::make_shared(
                    _node, nodeName, rmw_qos_profile_parameters, _callbackGroup));
      paramClientIt = insertedPair.first;
    }

    setParametersFuture.emplace_back(std::async(std::launch::async,
                                                &ParameterInterface::setNodeParameters, this,
                                                paramClientIt->second, nodeName, params, timeout));
  }

  for (auto& future : setParametersFuture) {
    try {
      future.get();
    } catch (const std::exception& e) {
      RCLCPP_ERROR(_node->get_logger(), "Exception when setting parameters: %s", e.what());
    }
  }
}

void ParameterInterface::subscribeParams(const std::vector<std::string>& paramNames) {
  std::lock_guard<std::mutex> lock(_mutex);

  std::unordered_set<std::string> nodesToSubscribe;
  for (const auto& paramName : paramNames) {
    if (!isWhitelisted(paramName, _paramWhitelistPatterns)) {
      return;
    }

    const auto& [nodeName, paramN] = getNodeAndParamName(paramName);
    auto [subscribedParamsit, wasNewlyCreated] = _subscribedParamsByNode.try_emplace(nodeName);

    auto& subscribedNodeParams = subscribedParamsit->second;
    subscribedNodeParams.insert(paramN);

    if (wasNewlyCreated) {
      nodesToSubscribe.insert(nodeName);
    }
  }

  for (const auto& nodeName : nodesToSubscribe) {
    auto paramClientIt = _paramClientsByNode.find(nodeName);
    if (paramClientIt == _paramClientsByNode.end()) {
      const auto insertedPair = _paramClientsByNode.emplace(
        nodeName, rclcpp::AsyncParametersClient::make_shared(
                    _node, nodeName, rmw_qos_profile_parameters, _callbackGroup));
      paramClientIt = insertedPair.first;
    }

    auto& paramClient = paramClientIt->second;

    _paramSubscriptionsByNode[nodeName] = paramClient->on_parameter_event(
      [this, nodeName](rcl_interfaces::msg::ParameterEvent::ConstSharedPtr msg) {
        RCLCPP_DEBUG(_node->get_logger(), "Retrieved param update for node %s: %zu params changed",
                     nodeName.c_str(), msg->changed_parameters.size());

        ParameterList result;
        const auto& subscribedNodeParams = _subscribedParamsByNode[nodeName];
        for (const auto& param : msg->changed_parameters) {
          if (subscribedNodeParams.find(param.name) != subscribedNodeParams.end()) {
            result.push_back(fromRosParam(
              rclcpp::Parameter(prependNodeNameToParamName(param.name, nodeName), param.value)));
          }
        }

        if (!result.empty() && _paramUpdateFunc) {
          _paramUpdateFunc(result);
        }
      });
  }
}

void ParameterInterface::unsubscribeParams(const std::vector<std::string>& paramNames) {
  std::lock_guard<std::mutex> lock(_mutex);

  for (const auto& paramName : paramNames) {
    const auto& [nodeName, paramN] = getNodeAndParamName(paramName);

    const auto subscribedNodeParamsIt = _subscribedParamsByNode.find(nodeName);
    if (subscribedNodeParamsIt != _subscribedParamsByNode.end()) {
      subscribedNodeParamsIt->second.erase(subscribedNodeParamsIt->second.find(paramN));

      if (subscribedNodeParamsIt->second.empty()) {
        _subscribedParamsByNode.erase(subscribedNodeParamsIt);
        _paramSubscriptionsByNode.erase(_paramSubscriptionsByNode.find(nodeName));
      }
    }
  }
}

void ParameterInterface::setParamUpdateCallback(ParamUpdateFunc paramUpdateFunc) {
  std::lock_guard<std::mutex> lock(_mutex);
  _paramUpdateFunc = paramUpdateFunc;
}

ParameterList ParameterInterface::getNodeParameters(
  const rclcpp::AsyncParametersClient::SharedPtr paramClient, const std::string& nodeName,
  const std::vector<std::string>& paramNames, const std::chrono::duration<double>& timeout) {
  if (!paramClient->service_is_ready()) {
    throw std::runtime_error("Parameter service for node '" + nodeName + "' is not ready");
  }

  auto paramsToRequest = paramNames;
  if (paramsToRequest.empty()) {
    // `paramNames` is empty, list all parameter names for this node
    auto future = paramClient->list_parameters({}, 0UL);
    if (std::future_status::ready != future.wait_for(timeout)) {
      throw std::runtime_error("Failed to retrieve parameter names for node '" + nodeName + "'");
    }
    paramsToRequest = future.get().names;
  }

  // Start parameter fetches and wait for them to complete
  auto getParamsFuture = paramClient->get_parameters(paramsToRequest);
  if (std::future_status::ready != getParamsFuture.wait_for(timeout)) {
    throw std::runtime_error("Timed out waiting for " + std::to_string(paramsToRequest.size()) +
                             " parameter(s) from node '" + nodeName + "'");
  }
  const auto params = getParamsFuture.get();

  ParameterList result;
  for (const auto& param : params) {
    const auto fullParamName = prependNodeNameToParamName(param.get_name(), nodeName);
    if (isWhitelisted(fullParamName, _paramWhitelistPatterns)) {
      result.push_back(fromRosParam(rclcpp::Parameter(fullParamName, param.get_parameter_value())));
    }
  }
  return result;
}

void ParameterInterface::setNodeParameters(rclcpp::AsyncParametersClient::SharedPtr paramClient,
                                           const std::string& nodeName,
                                           const std::vector<rclcpp::Parameter>& params,
                                           const std::chrono::duration<double>& timeout) {
  if (!paramClient->service_is_ready()) {
    throw std::runtime_error("Parameter service for node '" + nodeName + "' is not ready");
  }

  auto future = paramClient->set_parameters(params);

  std::vector<std::string> paramsToDelete;
  for (const auto& p : params) {
    if (p.get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET) {
      paramsToDelete.push_back(p.get_name());
    }
  }

  if (!paramsToDelete.empty()) {
    auto deleteFuture = paramClient->delete_parameters(paramsToDelete);
    if (std::future_status::ready != deleteFuture.wait_for(timeout)) {
      RCLCPP_WARN(
        _node->get_logger(),
        "Param client failed to delete %zu parameter(s) for node '%s' within the given timeout",
        paramsToDelete.size(), nodeName.c_str());
    }
  }

  if (std::future_status::ready != future.wait_for(timeout)) {
    throw std::runtime_error("Param client failed to set " + std::to_string(params.size()) +
                             " parameter(s) for node '" + nodeName + "' within the given timeout");
  }

  const auto setParamResults = future.get();
  for (auto& result : setParamResults) {
    if (!result.successful) {
      RCLCPP_WARN(_node->get_logger(), "Failed to set one or more parameters for node '%s': %s",
                  nodeName.c_str(), result.reason.c_str());
    }
  }
}

}  // namespace foxglove_bridge
