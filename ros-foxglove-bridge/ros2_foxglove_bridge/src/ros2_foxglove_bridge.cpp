#include <unordered_set>

#include <resource_retriever/retriever.hpp>

#include <foxglove_bridge/ros2_foxglove_bridge.hpp>

namespace foxglove_bridge {
namespace {
inline bool isHiddenTopicOrService(const std::string& name) {
  if (name.empty()) {
    throw std::invalid_argument("Topic or service name can't be empty");
  }
  return name.front() == '_' || name.find("/_") != std::string::npos;
}
}  // namespace

using namespace std::chrono_literals;
using namespace std::placeholders;
using foxglove::isWhitelisted;

FoxgloveBridge::FoxgloveBridge(const rclcpp::NodeOptions& options)
    : Node("foxglove_bridge", options) {
  const char* rosDistro = std::getenv("ROS_DISTRO");
  RCLCPP_INFO(this->get_logger(), "Starting foxglove_bridge (%s, %s@%s) with %s", rosDistro,
              foxglove::FOXGLOVE_BRIDGE_VERSION, foxglove::FOXGLOVE_BRIDGE_GIT_HASH,
              foxglove::WebSocketUserAgent());

  declareParameters(this);

  const auto port = static_cast<uint16_t>(this->get_parameter(PARAM_PORT).as_int());
  const auto address = this->get_parameter(PARAM_ADDRESS).as_string();
  const auto send_buffer_limit =
    static_cast<size_t>(this->get_parameter(PARAM_SEND_BUFFER_LIMIT).as_int());
  const auto useTLS = this->get_parameter(PARAM_USETLS).as_bool();
  const auto certfile = this->get_parameter(PARAM_CERTFILE).as_string();
  const auto keyfile = this->get_parameter(PARAM_KEYFILE).as_string();
  _minQosDepth = static_cast<size_t>(this->get_parameter(PARAM_MIN_QOS_DEPTH).as_int());
  _maxQosDepth = static_cast<size_t>(this->get_parameter(PARAM_MAX_QOS_DEPTH).as_int());
  const auto topicWhiteList = this->get_parameter(PARAM_TOPIC_WHITELIST).as_string_array();
  _topicWhitelistPatterns = parseRegexStrings(this, topicWhiteList);
  const auto serviceWhiteList = this->get_parameter(PARAM_SERVICE_WHITELIST).as_string_array();
  _serviceWhitelistPatterns = parseRegexStrings(this, serviceWhiteList);
  const auto paramWhiteList = this->get_parameter(PARAM_PARAMETER_WHITELIST).as_string_array();
  const auto paramWhitelistPatterns = parseRegexStrings(this, paramWhiteList);
  const auto useCompression = this->get_parameter(PARAM_USE_COMPRESSION).as_bool();
  _useSimTime = this->get_parameter("use_sim_time").as_bool();
  _capabilities = this->get_parameter(PARAM_CAPABILITIES).as_string_array();
  const auto clientTopicWhiteList =
    this->get_parameter(PARAM_CLIENT_TOPIC_WHITELIST).as_string_array();
  const auto clientTopicWhiteListPatterns = parseRegexStrings(this, clientTopicWhiteList);
  _includeHidden = this->get_parameter(PARAM_INCLUDE_HIDDEN).as_bool();
  const auto assetUriAllowlist = this->get_parameter(PARAM_ASSET_URI_ALLOWLIST).as_string_array();
  _assetUriAllowlistPatterns = parseRegexStrings(this, assetUriAllowlist);

  const auto logHandler = std::bind(&FoxgloveBridge::logHandler, this, _1, _2);
  // Fetching of assets may be blocking, hence we fetch them in a separate thread.
  _fetchAssetQueue = std::make_unique<foxglove::CallbackQueue>(logHandler, 1 /* num_threads */);

  foxglove::ServerOptions serverOptions;
  serverOptions.capabilities = _capabilities;
  if (_useSimTime) {
    serverOptions.capabilities.push_back(foxglove::CAPABILITY_TIME);
  }
  serverOptions.supportedEncodings = {"cdr"};
  serverOptions.metadata = {{"ROS_DISTRO", rosDistro}};
  serverOptions.sendBufferLimitBytes = send_buffer_limit;
  serverOptions.sessionId = std::to_string(std::time(nullptr));
  serverOptions.useCompression = useCompression;
  serverOptions.useTls = useTLS;
  serverOptions.certfile = certfile;
  serverOptions.keyfile = keyfile;
  serverOptions.clientTopicWhitelistPatterns = clientTopicWhiteListPatterns;

  _server = foxglove::ServerFactory::createServer<ConnectionHandle>("foxglove_bridge", logHandler,
                                                                    serverOptions);

  foxglove::ServerHandlers<ConnectionHandle> hdlrs;
  hdlrs.subscribeHandler = std::bind(&FoxgloveBridge::subscribe, this, _1, _2);
  hdlrs.unsubscribeHandler = std::bind(&FoxgloveBridge::unsubscribe, this, _1, _2);
  hdlrs.clientAdvertiseHandler = std::bind(&FoxgloveBridge::clientAdvertise, this, _1, _2);
  hdlrs.clientUnadvertiseHandler = std::bind(&FoxgloveBridge::clientUnadvertise, this, _1, _2);
  hdlrs.clientMessageHandler = std::bind(&FoxgloveBridge::clientMessage, this, _1, _2);
  hdlrs.serviceRequestHandler = std::bind(&FoxgloveBridge::serviceRequest, this, _1, _2);
  hdlrs.subscribeConnectionGraphHandler =
    std::bind(&FoxgloveBridge::subscribeConnectionGraph, this, _1);

  if (hasCapability(foxglove::CAPABILITY_PARAMETERS) ||
      hasCapability(foxglove::CAPABILITY_PARAMETERS_SUBSCRIBE)) {
    hdlrs.parameterRequestHandler = std::bind(&FoxgloveBridge::getParameters, this, _1, _2, _3);
    hdlrs.parameterChangeHandler = std::bind(&FoxgloveBridge::setParameters, this, _1, _2, _3);
    hdlrs.parameterSubscriptionHandler =
      std::bind(&FoxgloveBridge::subscribeParameters, this, _1, _2, _3);

    _paramInterface = std::make_shared<ParameterInterface>(this, paramWhitelistPatterns);
    _paramInterface->setParamUpdateCallback(std::bind(&FoxgloveBridge::parameterUpdates, this, _1));
  }

  if (hasCapability(foxglove::CAPABILITY_ASSETS)) {
    hdlrs.fetchAssetHandler = [this](const std::string& uri, uint32_t requestId,
                                     ConnectionHandle hdl) {
      _fetchAssetQueue->addCallback(
        std::bind(&FoxgloveBridge::fetchAsset, this, uri, requestId, hdl));
    };
  }

  _server->setHandlers(std::move(hdlrs));
  _server->start(address, port);

  // Get the actual port we bound to
  uint16_t listeningPort = _server->getPort();
  if (port != listeningPort) {
    RCLCPP_DEBUG(this->get_logger(), "Reassigning \"port\" parameter from %d to %d", port,
                 listeningPort);
    this->set_parameter(rclcpp::Parameter{PARAM_PORT, listeningPort});
  }

  // Start the thread polling for rosgraph changes
  _rosgraphPollThread =
    std::make_unique<std::thread>(std::bind(&FoxgloveBridge::rosgraphPollThread, this));

  _subscriptionCallbackGroup = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  _clientPublishCallbackGroup =
    this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  _servicesCallbackGroup = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  if (_useSimTime) {
    _clockSubscription = this->create_subscription<rosgraph_msgs::msg::Clock>(
      "/clock", rclcpp::QoS{rclcpp::KeepLast(1)}.best_effort(),
      [&](std::shared_ptr<rosgraph_msgs::msg::Clock> msg) {
        const auto timestamp = rclcpp::Time{msg->clock}.nanoseconds();
        assert(timestamp >= 0 && "Timestamp is negative");
        _server->broadcastTime(static_cast<uint64_t>(timestamp));
      });
  }
}

FoxgloveBridge::~FoxgloveBridge() {
  RCLCPP_INFO(this->get_logger(), "Shutting down %s", this->get_name());
  if (_rosgraphPollThread) {
    _rosgraphPollThread->join();
  }
  _server->stop();
  RCLCPP_INFO(this->get_logger(), "Shutdown complete");
}

void FoxgloveBridge::rosgraphPollThread() {
  updateAdvertisedTopics(get_topic_names_and_types());
  updateAdvertisedServices();

  auto graphEvent = this->get_graph_event();
  while (rclcpp::ok()) {
    try {
      this->wait_for_graph_change(graphEvent, 200ms);
      bool triggered = graphEvent->check_and_clear();
      if (triggered) {
        RCLCPP_DEBUG(this->get_logger(), "rosgraph change detected");
        const auto topicNamesAndTypes = get_topic_names_and_types();
        updateAdvertisedTopics(topicNamesAndTypes);
        updateAdvertisedServices();
        if (_subscribeGraphUpdates) {
          updateConnectionGraph(topicNamesAndTypes);
        }
        // Graph changes tend to come in batches, so wait a bit before checking again
        std::this_thread::sleep_for(500ms);
      }
    } catch (const std::exception& ex) {
      RCLCPP_ERROR(this->get_logger(), "Exception thrown in rosgraphPollThread: %s", ex.what());
    }
  }

  RCLCPP_DEBUG(this->get_logger(), "rosgraph polling thread exiting");
}

void FoxgloveBridge::updateAdvertisedTopics(
  const std::map<std::string, std::vector<std::string>>& topicNamesAndTypes) {
  if (!rclcpp::ok()) {
    return;
  }

  std::unordered_set<TopicAndDatatype, PairHash> latestTopics;
  latestTopics.reserve(topicNamesAndTypes.size());
  for (const auto& topicNamesAndType : topicNamesAndTypes) {
    const auto& topicName = topicNamesAndType.first;
    const auto& datatypes = topicNamesAndType.second;

    // Ignore hidden topics if not explicitly included
    if (!_includeHidden && isHiddenTopicOrService(topicName)) {
      continue;
    }

    // Ignore the topic if it is not on the topic whitelist
    if (isWhitelisted(topicName, _topicWhitelistPatterns)) {
      for (const auto& datatype : datatypes) {
        latestTopics.emplace(topicName, datatype);
      }
    }
  }

  if (const auto numIgnoredTopics = topicNamesAndTypes.size() - latestTopics.size()) {
    RCLCPP_DEBUG(
      this->get_logger(),
      "%zu topics have been ignored as they do not match any pattern on the topic whitelist",
      numIgnoredTopics);
  }

  std::lock_guard<std::mutex> lock(_subscriptionsMutex);

  // Remove channels for which the topic does not exist anymore
  std::vector<foxglove::ChannelId> channelIdsToRemove;
  for (auto channelIt = _advertisedTopics.begin(); channelIt != _advertisedTopics.end();) {
    const TopicAndDatatype topicAndDatatype = {channelIt->second.topic,
                                               channelIt->second.schemaName};
    if (latestTopics.find(topicAndDatatype) == latestTopics.end()) {
      const auto channelId = channelIt->first;
      channelIdsToRemove.push_back(channelId);
      _subscriptions.erase(channelId);
      RCLCPP_INFO(this->get_logger(), "Removed channel %d for topic \"%s\" (%s)", channelId,
                  topicAndDatatype.first.c_str(), topicAndDatatype.second.c_str());
      channelIt = _advertisedTopics.erase(channelIt);
    } else {
      channelIt++;
    }
  }
  _server->removeChannels(channelIdsToRemove);

  // Add new channels for new topics
  std::vector<foxglove::ChannelWithoutId> channelsToAdd;
  for (const auto& topicAndDatatype : latestTopics) {
    if (std::find_if(_advertisedTopics.begin(), _advertisedTopics.end(),
                     [topicAndDatatype](const auto& channelIdAndChannel) {
                       const auto& channel = channelIdAndChannel.second;
                       return channel.topic == topicAndDatatype.first &&
                              channel.schemaName == topicAndDatatype.second;
                     }) != _advertisedTopics.end()) {
      continue;  // Topic already advertised
    }

    foxglove::ChannelWithoutId newChannel{};
    newChannel.topic = topicAndDatatype.first;
    newChannel.schemaName = topicAndDatatype.second;

    try {
      auto [format, schema] = _messageDefinitionCache.get_full_text(topicAndDatatype.second);
      switch (format) {
        case foxglove::MessageDefinitionFormat::MSG:
          newChannel.encoding = "cdr";
          newChannel.schema = schema;
          newChannel.schemaEncoding = "ros2msg";
          break;
        case foxglove::MessageDefinitionFormat::IDL:
          newChannel.encoding = "cdr";
          newChannel.schema = schema;
          newChannel.schemaEncoding = "ros2idl";
          break;
      }

    } catch (const foxglove::DefinitionNotFoundError& err) {
      RCLCPP_WARN(this->get_logger(), "Could not find definition for type %s: %s",
                  topicAndDatatype.second.c_str(), err.what());
      // We still advertise the channel, but with an emtpy schema
      newChannel.schema = "";
    } catch (const std::exception& err) {
      RCLCPP_WARN(this->get_logger(), "Failed to add channel for topic \"%s\" (%s): %s",
                  topicAndDatatype.first.c_str(), topicAndDatatype.second.c_str(), err.what());
      continue;
    }

    channelsToAdd.push_back(newChannel);
  }

  const auto channelIds = _server->addChannels(channelsToAdd);
  for (size_t i = 0; i < channelsToAdd.size(); ++i) {
    const auto channelId = channelIds[i];
    const auto& channel = channelsToAdd[i];
    _advertisedTopics.emplace(channelId, channel);
    RCLCPP_DEBUG(this->get_logger(), "Advertising channel %d for topic \"%s\" (%s)", channelId,
                 channel.topic.c_str(), channel.schemaName.c_str());
  }
}

void FoxgloveBridge::updateAdvertisedServices() {
  if (!rclcpp::ok()) {
    return;
  } else if (!hasCapability(foxglove::CAPABILITY_SERVICES)) {
    return;
  }

  // Get the current list of visible services and datatypes from the ROS graph
  const auto serviceNamesAndTypes = this->get_node_graph_interface()->get_service_names_and_types();

  std::lock_guard<std::mutex> lock(_servicesMutex);

  // Remove advertisements for services that have been removed
  std::vector<foxglove::ServiceId> servicesToRemove;
  for (const auto& service : _advertisedServices) {
    const auto it = std::find_if(serviceNamesAndTypes.begin(), serviceNamesAndTypes.end(),
                                 [service](const auto& serviceNameAndTypes) {
                                   return serviceNameAndTypes.first == service.second.name;
                                 });
    if (it == serviceNamesAndTypes.end()) {
      servicesToRemove.push_back(service.first);
    }
  }
  for (auto serviceId : servicesToRemove) {
    _advertisedServices.erase(serviceId);
  }
  _server->removeServices(servicesToRemove);

  // Advertise new services
  std::vector<foxglove::ServiceWithoutId> newServices;
  for (const auto& serviceNamesAndType : serviceNamesAndTypes) {
    const auto& serviceName = serviceNamesAndType.first;
    const auto& datatypes = serviceNamesAndType.second;

    // Ignore the service if it's already advertised
    if (std::find_if(_advertisedServices.begin(), _advertisedServices.end(),
                     [serviceName](const auto& idWithService) {
                       return idWithService.second.name == serviceName;
                     }) != _advertisedServices.end()) {
      continue;
    }

    // Ignore hidden services if not explicitly included
    if (!_includeHidden && isHiddenTopicOrService(serviceName)) {
      continue;
    }

    // Ignore the service if it is not on the service whitelist
    if (!isWhitelisted(serviceName, _serviceWhitelistPatterns)) {
      continue;
    }

    foxglove::ServiceWithoutId service;
    service.name = serviceName;
    service.type = datatypes.front();

    try {
      const auto requestTypeName = service.type + foxglove::SERVICE_REQUEST_MESSAGE_SUFFIX;
      const auto responseTypeName = service.type + foxglove::SERVICE_RESPONSE_MESSAGE_SUFFIX;
      const auto [format, reqSchema] = _messageDefinitionCache.get_full_text(requestTypeName);
      const auto resSchema = _messageDefinitionCache.get_full_text(responseTypeName).second;
      switch (format) {
        case foxglove::MessageDefinitionFormat::MSG:
          service.requestSchema = reqSchema;
          service.responseSchema = resSchema;
          break;
        case foxglove::MessageDefinitionFormat::IDL:
          RCLCPP_WARN(this->get_logger(),
                      "IDL message definition format cannot be communicated over ws-protocol. "
                      "Service \"%s\" (%s) may not decode correctly in clients",
                      service.name.c_str(), service.type.c_str());
          service.requestSchema = reqSchema;
          service.responseSchema = resSchema;
          break;
      }
    } catch (const foxglove::DefinitionNotFoundError& err) {
      RCLCPP_WARN(this->get_logger(), "Could not find definition for type %s: %s",
                  service.type.c_str(), err.what());
      // We still advertise the service, but with an emtpy schema
      service.requestSchema = "";
      service.responseSchema = "";
    } catch (const std::exception& err) {
      RCLCPP_WARN(this->get_logger(), "Failed to add service \"%s\" (%s): %s", service.name.c_str(),
                  service.type.c_str(), err.what());
      continue;
    }

    newServices.push_back(service);
  }

  const auto serviceIds = _server->addServices(newServices);
  for (size_t i = 0; i < serviceIds.size(); ++i) {
    _advertisedServices.emplace(serviceIds[i], newServices[i]);
  }
}

void FoxgloveBridge::updateConnectionGraph(
  const std::map<std::string, std::vector<std::string>>& topicNamesAndTypes) {
  foxglove::MapOfSets publishers, subscribers;

  for (const auto& topicNameAndType : topicNamesAndTypes) {
    const auto& topicName = topicNameAndType.first;
    if (!isWhitelisted(topicName, _topicWhitelistPatterns)) {
      continue;
    }

    const auto publishersInfo = get_publishers_info_by_topic(topicName);
    const auto subscribersInfo = get_subscriptions_info_by_topic(topicName);
    std::unordered_set<std::string> publisherIds, subscriberIds;
    for (const auto& publisher : publishersInfo) {
      const auto& ns = publisher.node_namespace();
      const auto sep = (!ns.empty() && ns.back() == '/') ? "" : "/";
      publisherIds.insert(ns + sep + publisher.node_name());
    }
    for (const auto& subscriber : subscribersInfo) {
      const auto& ns = subscriber.node_namespace();
      const auto sep = (!ns.empty() && ns.back() == '/') ? "" : "/";
      subscriberIds.insert(ns + sep + subscriber.node_name());
    }
    publishers.emplace(topicName, publisherIds);
    subscribers.emplace(topicName, subscriberIds);
  }

  foxglove::MapOfSets services;
  for (const auto& fqnNodeName : get_node_names()) {
    const auto [nodeNs, nodeName] = getNodeAndNodeNamespace(fqnNodeName);
    const auto serviceNamesAndTypes = get_service_names_and_types_by_node(nodeName, nodeNs);

    for (const auto& [serviceName, serviceTypes] : serviceNamesAndTypes) {
      (void)serviceTypes;
      if (isWhitelisted(serviceName, _serviceWhitelistPatterns)) {
        services[serviceName].insert(fqnNodeName);
      }
    }
  }

  _server->updateConnectionGraph(publishers, subscribers, services);
}

void FoxgloveBridge::subscribeConnectionGraph(bool subscribe) {
  if ((_subscribeGraphUpdates = subscribe)) {
    updateConnectionGraph(get_topic_names_and_types());
  };
}

void FoxgloveBridge::subscribe(foxglove::ChannelId channelId, ConnectionHandle clientHandle) {
  std::lock_guard<std::mutex> lock(_subscriptionsMutex);
  auto it = _advertisedTopics.find(channelId);
  if (it == _advertisedTopics.end()) {
    throw foxglove::ChannelError(
      channelId, "Received subscribe request for unknown channel " + std::to_string(channelId));
  }

  const auto& channel = it->second;
  const auto& topic = channel.topic;
  const auto& datatype = channel.schemaName;

  // Get client subscriptions for this channel or insert an empty map.
  auto [subscriptionsIt, firstSubscription] =
    _subscriptions.emplace(channelId, SubscriptionsByClient());
  auto& subscriptionsByClient = subscriptionsIt->second;

  if (!firstSubscription &&
      subscriptionsByClient.find(clientHandle) != subscriptionsByClient.end()) {
    throw foxglove::ChannelError(
      channelId, "Client is already subscribed to channel " + std::to_string(channelId));
  }

  rclcpp::SubscriptionEventCallbacks eventCallbacks;
  eventCallbacks.incompatible_qos_callback = [&](const rclcpp::QOSRequestedIncompatibleQoSInfo&) {
    RCLCPP_ERROR(this->get_logger(), "Incompatible subscriber QoS settings for topic \"%s\" (%s)",
                 topic.c_str(), datatype.c_str());
  };

  rclcpp::SubscriptionOptions subscriptionOptions;
  subscriptionOptions.event_callbacks = eventCallbacks;
  subscriptionOptions.callback_group = _subscriptionCallbackGroup;

  // Select an appropriate subscription QOS profile. This is similar to how ros2 topic echo
  // does it:
  // https://github.com/ros2/ros2cli/blob/619b3d1c9/ros2topic/ros2topic/verb/echo.py#L137-L194
  size_t depth = 0;
  size_t reliabilityReliableEndpointsCount = 0;
  size_t durabilityTransientLocalEndpointsCount = 0;

  const auto publisherInfo = this->get_publishers_info_by_topic(topic);
  for (const auto& publisher : publisherInfo) {
    const auto& qos = publisher.qos_profile();
    if (qos.reliability() == rclcpp::ReliabilityPolicy::Reliable) {
      ++reliabilityReliableEndpointsCount;
    }
    if (qos.durability() == rclcpp::DurabilityPolicy::TransientLocal) {
      ++durabilityTransientLocalEndpointsCount;
    }
    // Some RMWs do not retrieve history information of the publisher endpoint in which case the
    // history depth is 0. We use a lower limit of 1 here, so that the history depth is at least
    // equal to the number of publishers. This covers the case where there are multiple
    // transient_local publishers with a depth of 1 (e.g. multiple tf_static transform
    // broadcasters). See also
    // https://github.com/foxglove/ros-foxglove-bridge/issues/238 and
    // https://github.com/foxglove/ros-foxglove-bridge/issues/208
    const size_t publisherHistoryDepth = std::max(1ul, qos.depth());
    depth = depth + publisherHistoryDepth;
  }

  depth = std::max(depth, _minQosDepth);
  if (depth > _maxQosDepth) {
    RCLCPP_WARN(this->get_logger(),
                "Limiting history depth for topic '%s' to %zu (was %zu). You may want to increase "
                "the max_qos_depth parameter value.",
                topic.c_str(), _maxQosDepth, depth);
    depth = _maxQosDepth;
  }

  rclcpp::QoS qos{rclcpp::KeepLast(depth)};

  // If all endpoints are reliable, ask for reliable
  if (!publisherInfo.empty() && reliabilityReliableEndpointsCount == publisherInfo.size()) {
    qos.reliable();
  } else {
    if (reliabilityReliableEndpointsCount > 0) {
      RCLCPP_WARN(
        this->get_logger(),
        "Some, but not all, publishers on topic '%s' are offering QoSReliabilityPolicy.RELIABLE. "
        "Falling back to QoSReliabilityPolicy.BEST_EFFORT as it will connect to all publishers",
        topic.c_str());
    }
    qos.best_effort();
  }

  // If all endpoints are transient_local, ask for transient_local
  if (!publisherInfo.empty() && durabilityTransientLocalEndpointsCount == publisherInfo.size()) {
    qos.transient_local();
  } else {
    if (durabilityTransientLocalEndpointsCount > 0) {
      RCLCPP_WARN(this->get_logger(),
                  "Some, but not all, publishers on topic '%s' are offering "
                  "QoSDurabilityPolicy.TRANSIENT_LOCAL. Falling back to "
                  "QoSDurabilityPolicy.VOLATILE as it will connect to all publishers",
                  topic.c_str());
    }
    qos.durability_volatile();
  }

  if (firstSubscription) {
    RCLCPP_INFO(this->get_logger(), "Subscribing to topic \"%s\" (%s) on channel %d", topic.c_str(),
                datatype.c_str(), channelId);

  } else {
    RCLCPP_INFO(this->get_logger(), "Adding subscriber #%zu to topic \"%s\" (%s) on channel %d",
                subscriptionsByClient.size(), topic.c_str(), datatype.c_str(), channelId);
  }

  try {
    auto subscriber = this->create_generic_subscription(
      topic, datatype, qos,
      [this, channelId, clientHandle](std::shared_ptr<rclcpp::SerializedMessage> msg) {
        this->rosMessageHandler(channelId, clientHandle, msg);
      },
      subscriptionOptions);
    subscriptionsByClient.emplace(clientHandle, std::move(subscriber));
  } catch (const std::exception& ex) {
    throw foxglove::ChannelError(
      channelId, "Failed to subscribe to topic " + topic + " (" + datatype + "): " + ex.what());
  }
}

void FoxgloveBridge::unsubscribe(foxglove::ChannelId channelId, ConnectionHandle clientHandle) {
  std::lock_guard<std::mutex> lock(_subscriptionsMutex);

  const auto channelIt = _advertisedTopics.find(channelId);
  if (channelIt == _advertisedTopics.end()) {
    throw foxglove::ChannelError(
      channelId, "Received unsubscribe request for unknown channel " + std::to_string(channelId));
  }
  const auto& channel = channelIt->second;

  auto subscriptionsIt = _subscriptions.find(channelId);
  if (subscriptionsIt == _subscriptions.end()) {
    throw foxglove::ChannelError(channelId, "Received unsubscribe request for channel " +
                                              std::to_string(channelId) +
                                              " that was not subscribed to");
  }

  auto& subscriptionsByClient = subscriptionsIt->second;
  const auto clientSubscription = subscriptionsByClient.find(clientHandle);
  if (clientSubscription == subscriptionsByClient.end()) {
    throw foxglove::ChannelError(
      channelId, "Received unsubscribe request for channel " + std::to_string(channelId) +
                   "from a client that was not subscribed to this channel");
  }

  subscriptionsByClient.erase(clientSubscription);
  if (subscriptionsByClient.empty()) {
    RCLCPP_INFO(this->get_logger(), "Unsubscribing from topic \"%s\" (%s) on channel %d",
                channel.topic.c_str(), channel.schemaName.c_str(), channelId);
    _subscriptions.erase(subscriptionsIt);
  } else {
    RCLCPP_INFO(this->get_logger(),
                "Removed one subscription from channel %d (%zu subscription(s) left)", channelId,
                subscriptionsByClient.size());
  }
}

void FoxgloveBridge::clientAdvertise(const foxglove::ClientAdvertisement& advertisement,
                                     ConnectionHandle hdl) {
  std::lock_guard<std::mutex> lock(_clientAdvertisementsMutex);

  // Get client publications or insert an empty map.
  auto [clientPublicationsIt, isFirstPublication] =
    _clientAdvertisedTopics.emplace(hdl, ClientPublications());

  auto& clientPublications = clientPublicationsIt->second;

  if (!isFirstPublication &&
      clientPublications.find(advertisement.channelId) != clientPublications.end()) {
    throw foxglove::ClientChannelError(
      advertisement.channelId,
      "Received client advertisement from " + _server->remoteEndpointString(hdl) + " for channel " +
        std::to_string(advertisement.channelId) + " it had already advertised");
  }

  try {
    // Create a new topic advertisement
    const auto& topicName = advertisement.topic;
    const auto& topicType = advertisement.schemaName;

    // Lookup if there are publishers from other nodes for that topic. If that's the case, we use
    // a matching QoS profile.
    const auto otherPublishers = get_publishers_info_by_topic(topicName);
    const auto otherPublisherIt =
      std::find_if(otherPublishers.begin(), otherPublishers.end(),
                   [this](const rclcpp::TopicEndpointInfo& endpoint) {
                     return endpoint.node_name() != this->get_name() ||
                            endpoint.node_namespace() != this->get_namespace();
                   });
    rclcpp::QoS qos = otherPublisherIt == otherPublishers.end() ? rclcpp::SystemDefaultsQoS()
                                                                : otherPublisherIt->qos_profile();

    // When the QoS profile is copied from another existing publisher, it can happen that the
    // history policy is Unknown, leading to an error when subsequently trying to create a publisher
    // with that QoS profile. As a fix, we explicitly set the history policy to the system default.
    if (qos.history() == rclcpp::HistoryPolicy::Unknown) {
      qos.history(rclcpp::HistoryPolicy::SystemDefault);
    }
    rclcpp::PublisherOptions publisherOptions{};
    publisherOptions.callback_group = _clientPublishCallbackGroup;
    auto publisher = this->create_generic_publisher(topicName, topicType, qos, publisherOptions);

    RCLCPP_INFO(this->get_logger(), "Client %s is advertising \"%s\" (%s) on channel %d",
                _server->remoteEndpointString(hdl).c_str(), topicName.c_str(), topicType.c_str(),
                advertisement.channelId);

    // Store the new topic advertisement
    clientPublications.emplace(advertisement.channelId, std::move(publisher));
  } catch (const std::exception& ex) {
    throw foxglove::ClientChannelError(advertisement.channelId,
                                       std::string("Failed to create publisher: ") + ex.what());
  }
}

void FoxgloveBridge::clientUnadvertise(foxglove::ChannelId channelId, ConnectionHandle hdl) {
  std::lock_guard<std::mutex> lock(_clientAdvertisementsMutex);

  auto it = _clientAdvertisedTopics.find(hdl);
  if (it == _clientAdvertisedTopics.end()) {
    throw foxglove::ClientChannelError(
      channelId, "Ignoring client unadvertisement from " + _server->remoteEndpointString(hdl) +
                   " for unknown channel " + std::to_string(channelId) +
                   ", client has no advertised topics");
  }

  auto& clientPublications = it->second;
  auto it2 = clientPublications.find(channelId);
  if (it2 == clientPublications.end()) {
    throw foxglove::ClientChannelError(
      channelId, "Ignoring client unadvertisement from " + _server->remoteEndpointString(hdl) +
                   " for unknown channel " + std::to_string(channelId) + ", client has " +
                   std::to_string(clientPublications.size()) + " advertised topic(s)");
  }

  const auto& publisher = it2->second;
  RCLCPP_INFO(this->get_logger(),
              "Client %s is no longer advertising %s (%zu subscribers) on channel %d",
              _server->remoteEndpointString(hdl).c_str(), publisher->get_topic_name(),
              publisher->get_subscription_count(), channelId);

  clientPublications.erase(it2);
  if (clientPublications.empty()) {
    _clientAdvertisedTopics.erase(it);
  }

  // Create a timer that immedeately goes out of scope (so it never fires) which will trigger
  // the previously destroyed publisher to be cleaned up. This is a workaround for
  // https://github.com/ros2/rclcpp/issues/2146
  this->create_wall_timer(1s, []() {});
}

void FoxgloveBridge::clientMessage(const foxglove::ClientMessage& message, ConnectionHandle hdl) {
  // Get the publisher
  rclcpp::GenericPublisher::SharedPtr publisher;
  {
    const auto channelId = message.advertisement.channelId;
    std::lock_guard<std::mutex> lock(_clientAdvertisementsMutex);

    auto it = _clientAdvertisedTopics.find(hdl);
    if (it == _clientAdvertisedTopics.end()) {
      throw foxglove::ClientChannelError(
        channelId, "Dropping client message from " + _server->remoteEndpointString(hdl) +
                     " for unknown channel " + std::to_string(channelId) +
                     ", client has no advertised topics");
    }

    auto& clientPublications = it->second;
    auto it2 = clientPublications.find(channelId);
    if (it2 == clientPublications.end()) {
      throw foxglove::ClientChannelError(
        channelId, "Dropping client message from " + _server->remoteEndpointString(hdl) +
                     " for unknown channel " + std::to_string(channelId) + ", client has " +
                     std::to_string(clientPublications.size()) + " advertised topic(s)");
    }
    publisher = it2->second;
  }

  // Copy the message payload into a SerializedMessage object
  rclcpp::SerializedMessage serializedMessage{message.getLength()};
  auto& rclSerializedMsg = serializedMessage.get_rcl_serialized_message();
  std::memcpy(rclSerializedMsg.buffer, message.getData(), message.getLength());
  rclSerializedMsg.buffer_length = message.getLength();

  // Publish the message
  publisher->publish(serializedMessage);
}

void FoxgloveBridge::setParameters(const std::vector<foxglove::Parameter>& parameters,
                                   const std::optional<std::string>& requestId,
                                   ConnectionHandle hdl) {
  _paramInterface->setParams(parameters, std::chrono::seconds(5));

  // If a request Id was given, send potentially updated parameters back to client
  if (requestId) {
    std::vector<std::string> parameterNames(parameters.size());
    for (size_t i = 0; i < parameters.size(); ++i) {
      parameterNames[i] = parameters[i].getName();
    }
    getParameters(parameterNames, requestId, hdl);
  }
}

void FoxgloveBridge::getParameters(const std::vector<std::string>& parameters,
                                   const std::optional<std::string>& requestId,
                                   ConnectionHandle hdl) {
  const auto params = _paramInterface->getParams(parameters, std::chrono::seconds(5));
  _server->publishParameterValues(hdl, params, requestId);
}

void FoxgloveBridge::subscribeParameters(const std::vector<std::string>& parameters,
                                         foxglove::ParameterSubscriptionOperation op,
                                         ConnectionHandle) {
  if (op == foxglove::ParameterSubscriptionOperation::SUBSCRIBE) {
    _paramInterface->subscribeParams(parameters);
  } else {
    _paramInterface->unsubscribeParams(parameters);
  }
}

void FoxgloveBridge::parameterUpdates(const std::vector<foxglove::Parameter>& parameters) {
  _server->updateParameterValues(parameters);
}

void FoxgloveBridge::logHandler(LogLevel level, char const* msg) {
  switch (level) {
    case LogLevel::Debug:
      RCLCPP_DEBUG(this->get_logger(), "[WS] %s", msg);
      break;
    case LogLevel::Info:
      RCLCPP_INFO(this->get_logger(), "[WS] %s", msg);
      break;
    case LogLevel::Warn:
      RCLCPP_WARN(this->get_logger(), "[WS] %s", msg);
      break;
    case LogLevel::Error:
      RCLCPP_ERROR(this->get_logger(), "[WS] %s", msg);
      break;
    case LogLevel::Critical:
      RCLCPP_FATAL(this->get_logger(), "[WS] %s", msg);
      break;
  }
}

void FoxgloveBridge::rosMessageHandler(const foxglove::ChannelId& channelId,
                                       ConnectionHandle clientHandle,
                                       std::shared_ptr<rclcpp::SerializedMessage> msg) {
  // NOTE: Do not call any RCLCPP_* logging functions from this function. Otherwise, subscribing
  // to `/rosout` will cause a feedback loop
  const auto timestamp = this->now().nanoseconds();
  assert(timestamp >= 0 && "Timestamp is negative");
  const auto rclSerializedMsg = msg->get_rcl_serialized_message();
  _server->sendMessage(clientHandle, channelId, static_cast<uint64_t>(timestamp),
                       rclSerializedMsg.buffer, rclSerializedMsg.buffer_length);
}

void FoxgloveBridge::serviceRequest(const foxglove::ServiceRequest& request,
                                    ConnectionHandle clientHandle) {
  RCLCPP_DEBUG(this->get_logger(), "Received a request for service %d", request.serviceId);

  std::lock_guard<std::mutex> lock(_servicesMutex);
  const auto serviceIt = _advertisedServices.find(request.serviceId);
  if (serviceIt == _advertisedServices.end()) {
    throw foxglove::ServiceError(
      request.serviceId,
      "Service with id " + std::to_string(request.serviceId) + " does not exist");
  }

  auto clientIt = _serviceClients.find(request.serviceId);
  if (clientIt == _serviceClients.end()) {
    try {
      auto clientOptions = rcl_client_get_default_options();
      auto genClient = GenericClient::make_shared(
        this->get_node_base_interface().get(), this->get_node_graph_interface(),
        serviceIt->second.name, serviceIt->second.type, clientOptions);
      clientIt = _serviceClients.emplace(request.serviceId, std::move(genClient)).first;
      this->get_node_services_interface()->add_client(clientIt->second, _servicesCallbackGroup);
    } catch (const std::exception& ex) {
      throw foxglove::ServiceError(
        request.serviceId,
        "Failed to create service client for service " + serviceIt->second.name + ": " + ex.what());
    }
  }

  auto client = clientIt->second;
  if (!client->wait_for_service(1s)) {
    throw foxglove::ServiceError(request.serviceId,
                                 "Service " + serviceIt->second.name + " is not available");
  }

  auto reqMessage = std::make_shared<rclcpp::SerializedMessage>(request.data.size());
  auto& rclSerializedMsg = reqMessage->get_rcl_serialized_message();
  std::memcpy(rclSerializedMsg.buffer, request.data.data(), request.data.size());
  rclSerializedMsg.buffer_length = request.data.size();

  auto responseReceivedCallback = [this, request,
                                   clientHandle](GenericClient::SharedFuture future) {
    const auto serializedResponseMsg = future.get()->get_rcl_serialized_message();
    foxglove::ServiceRequest response{request.serviceId, request.callId, request.encoding,
                                      std::vector<uint8_t>(serializedResponseMsg.buffer_length)};
    std::memcpy(response.data.data(), serializedResponseMsg.buffer,
                serializedResponseMsg.buffer_length);
    _server->sendServiceResponse(clientHandle, response);
  };
  client->async_send_request(reqMessage, responseReceivedCallback);
}

void FoxgloveBridge::fetchAsset(const std::string& uri, uint32_t requestId,
                                ConnectionHandle clientHandle) {
  foxglove::FetchAssetResponse response;
  response.requestId = requestId;

  try {
    // We reject URIs that are not on the allowlist or that contain two consecutive dots. The latter
    // can be utilized to construct URIs for retrieving confidential files that should not be
    // accessible over the WebSocket connection. Example:
    // `package://<pkg_name>/../../../secret.txt`. This is an extra security measure and should not
    // be necessary if the allowlist is strict enough.
    if (uri.find("..") != std::string::npos || !isWhitelisted(uri, _assetUriAllowlistPatterns)) {
      throw std::runtime_error("Asset URI not allowed: " + uri);
    }

    resource_retriever::Retriever resource_retriever;
    const resource_retriever::MemoryResource memoryResource = resource_retriever.get(uri);
    response.status = foxglove::FetchAssetStatus::Success;
    response.errorMessage = "";
    response.data.resize(memoryResource.size);
    std::memcpy(response.data.data(), memoryResource.data.get(), memoryResource.size);
  } catch (const std::exception& ex) {
    RCLCPP_WARN(this->get_logger(), "Failed to retrieve asset '%s': %s", uri.c_str(), ex.what());
    response.status = foxglove::FetchAssetStatus::Error;
    response.errorMessage = "Failed to retrieve asset " + uri;
  }

  if (_server) {
    _server->sendFetchAssetResponse(clientHandle, response);
  }
}

bool FoxgloveBridge::hasCapability(const std::string& capability) {
  return std::find(_capabilities.begin(), _capabilities.end(), capability) != _capabilities.end();
}

}  // namespace foxglove_bridge

#include <rclcpp_components/register_node_macro.hpp>

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(foxglove_bridge::FoxgloveBridge)
