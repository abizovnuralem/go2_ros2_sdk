#include <atomic>
#include <functional>
#include <memory>
#include <mutex>
#include <regex>
#include <shared_mutex>
#include <string>
#include <unordered_set>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <resource_retriever/retriever.h>
#include <ros/message_event.h>
#include <ros/ros.h>
#include <ros/xmlrpc_manager.h>
#include <ros_babel_fish/babel_fish_message.h>
#include <ros_babel_fish/generation/providers/integrated_description_provider.h>
#include <rosgraph_msgs/Clock.h>
#include <websocketpp/common/connection_hdl.hpp>

#include <foxglove_bridge/foxglove_bridge.hpp>
#include <foxglove_bridge/generic_service.hpp>
#include <foxglove_bridge/param_utils.hpp>
#include <foxglove_bridge/regex_utils.hpp>
#include <foxglove_bridge/server_factory.hpp>
#include <foxglove_bridge/service_utils.hpp>
#include <foxglove_bridge/websocket_server.hpp>

namespace {

inline std::unordered_set<std::string> rpcValueToStringSet(const XmlRpc::XmlRpcValue& v) {
  std::unordered_set<std::string> set;
  for (int i = 0; i < v.size(); ++i) {
    set.insert(v[i]);
  }
  return set;
}

}  // namespace

namespace foxglove_bridge {

constexpr int DEFAULT_PORT = 8765;
constexpr char DEFAULT_ADDRESS[] = "0.0.0.0";
constexpr int DEFAULT_MAX_UPDATE_MS = 5000;
constexpr char ROS1_CHANNEL_ENCODING[] = "ros1";
constexpr uint32_t SUBSCRIPTION_QUEUE_LENGTH = 10;
constexpr double MIN_UPDATE_PERIOD_MS = 100.0;
constexpr uint32_t PUBLICATION_QUEUE_LENGTH = 10;
constexpr int DEFAULT_SERVICE_TYPE_RETRIEVAL_TIMEOUT_MS = 250;

using ConnectionHandle = websocketpp::connection_hdl;
using TopicAndDatatype = std::pair<std::string, std::string>;
using SubscriptionsByClient = std::map<ConnectionHandle, ros::Subscriber, std::owner_less<>>;
using ClientPublications = std::unordered_map<foxglove::ClientChannelId, ros::Publisher>;
using PublicationsByClient = std::map<ConnectionHandle, ClientPublications, std::owner_less<>>;
using foxglove::isWhitelisted;

class FoxgloveBridge : public nodelet::Nodelet {
public:
  FoxgloveBridge() = default;
  virtual void onInit() {
    auto& nhp = getPrivateNodeHandle();
    const auto address = nhp.param<std::string>("address", DEFAULT_ADDRESS);
    const int port = nhp.param<int>("port", DEFAULT_PORT);
    const auto send_buffer_limit = static_cast<size_t>(
      nhp.param<int>("send_buffer_limit", foxglove::DEFAULT_SEND_BUFFER_LIMIT_BYTES));
    const auto useTLS = nhp.param<bool>("tls", false);
    const auto certfile = nhp.param<std::string>("certfile", "");
    const auto keyfile = nhp.param<std::string>("keyfile", "");
    _maxUpdateMs = static_cast<size_t>(nhp.param<int>("max_update_ms", DEFAULT_MAX_UPDATE_MS));
    const auto useCompression = nhp.param<bool>("use_compression", false);
    _useSimTime = nhp.param<bool>("/use_sim_time", false);
    const auto sessionId = nhp.param<std::string>("/run_id", std::to_string(std::time(nullptr)));
    _capabilities = nhp.param<std::vector<std::string>>(
      "capabilities", std::vector<std::string>(foxglove::DEFAULT_CAPABILITIES.begin(),
                                               foxglove::DEFAULT_CAPABILITIES.end()));
    _serviceRetrievalTimeoutMs = nhp.param<int>("service_type_retrieval_timeout_ms",
                                                DEFAULT_SERVICE_TYPE_RETRIEVAL_TIMEOUT_MS);

    const auto topicWhitelistPatterns =
      nhp.param<std::vector<std::string>>("topic_whitelist", {".*"});
    _topicWhitelistPatterns = parseRegexPatterns(topicWhitelistPatterns);
    if (topicWhitelistPatterns.size() != _topicWhitelistPatterns.size()) {
      ROS_ERROR("Failed to parse one or more topic whitelist patterns");
    }
    const auto paramWhitelist = nhp.param<std::vector<std::string>>("param_whitelist", {".*"});
    _paramWhitelistPatterns = parseRegexPatterns(paramWhitelist);
    if (paramWhitelist.size() != _paramWhitelistPatterns.size()) {
      ROS_ERROR("Failed to parse one or more param whitelist patterns");
    }

    const auto serviceWhitelist = nhp.param<std::vector<std::string>>("service_whitelist", {".*"});
    _serviceWhitelistPatterns = parseRegexPatterns(serviceWhitelist);
    if (serviceWhitelist.size() != _serviceWhitelistPatterns.size()) {
      ROS_ERROR("Failed to parse one or more service whitelist patterns");
    }

    const auto clientTopicWhitelist =
      nhp.param<std::vector<std::string>>("client_topic_whitelist", {".*"});
    const auto clientTopicWhitelistPatterns = parseRegexPatterns(clientTopicWhitelist);
    if (clientTopicWhitelist.size() != clientTopicWhitelistPatterns.size()) {
      ROS_ERROR("Failed to parse one or more service whitelist patterns");
    }

    const auto assetUriAllowlist = nhp.param<std::vector<std::string>>(
      "asset_uri_allowlist",
      {"^package://(?:\\w+/"
       ")*\\w+\\.(?:dae|fbx|glb|gltf|jpeg|jpg|mtl|obj|png|stl|tif|tiff|urdf|webp|xacro)$"});
    _assetUriAllowlistPatterns = parseRegexPatterns(assetUriAllowlist);
    if (assetUriAllowlist.size() != _assetUriAllowlistPatterns.size()) {
      ROS_ERROR("Failed to parse one or more asset URI whitelist patterns");
    }

    const char* rosDistro = std::getenv("ROS_DISTRO");
    ROS_INFO("Starting foxglove_bridge (%s, %s@%s) with %s", rosDistro,
             foxglove::FOXGLOVE_BRIDGE_VERSION, foxglove::FOXGLOVE_BRIDGE_GIT_HASH,
             foxglove::WebSocketUserAgent());

    try {
      foxglove::ServerOptions serverOptions;
      serverOptions.capabilities = _capabilities;
      if (_useSimTime) {
        serverOptions.capabilities.push_back(foxglove::CAPABILITY_TIME);
      }
      serverOptions.supportedEncodings = {ROS1_CHANNEL_ENCODING};
      serverOptions.metadata = {{"ROS_DISTRO", rosDistro}};
      serverOptions.sendBufferLimitBytes = send_buffer_limit;
      serverOptions.sessionId = sessionId;
      serverOptions.useTls = useTLS;
      serverOptions.certfile = certfile;
      serverOptions.keyfile = keyfile;
      serverOptions.useCompression = useCompression;
      serverOptions.clientTopicWhitelistPatterns = clientTopicWhitelistPatterns;

      const auto logHandler =
        std::bind(&FoxgloveBridge::logHandler, this, std::placeholders::_1, std::placeholders::_2);

      // Fetching of assets may be blocking, hence we fetch them in a separate thread.
      _fetchAssetQueue = std::make_unique<foxglove::CallbackQueue>(logHandler, 1 /* num_threads */);

      _server = foxglove::ServerFactory::createServer<ConnectionHandle>("foxglove_bridge",
                                                                        logHandler, serverOptions);
      foxglove::ServerHandlers<ConnectionHandle> hdlrs;
      hdlrs.subscribeHandler =
        std::bind(&FoxgloveBridge::subscribe, this, std::placeholders::_1, std::placeholders::_2);
      hdlrs.unsubscribeHandler =
        std::bind(&FoxgloveBridge::unsubscribe, this, std::placeholders::_1, std::placeholders::_2);
      hdlrs.clientAdvertiseHandler = std::bind(&FoxgloveBridge::clientAdvertise, this,
                                               std::placeholders::_1, std::placeholders::_2);
      hdlrs.clientUnadvertiseHandler = std::bind(&FoxgloveBridge::clientUnadvertise, this,
                                                 std::placeholders::_1, std::placeholders::_2);
      hdlrs.clientMessageHandler = std::bind(&FoxgloveBridge::clientMessage, this,
                                             std::placeholders::_1, std::placeholders::_2);
      hdlrs.parameterRequestHandler =
        std::bind(&FoxgloveBridge::getParameters, this, std::placeholders::_1,
                  std::placeholders::_2, std::placeholders::_3);
      hdlrs.parameterChangeHandler =
        std::bind(&FoxgloveBridge::setParameters, this, std::placeholders::_1,
                  std::placeholders::_2, std::placeholders::_3);
      hdlrs.parameterSubscriptionHandler =
        std::bind(&FoxgloveBridge::subscribeParameters, this, std::placeholders::_1,
                  std::placeholders::_2, std::placeholders::_3);
      hdlrs.serviceRequestHandler = std::bind(&FoxgloveBridge::serviceRequest, this,
                                              std::placeholders::_1, std::placeholders::_2);
      hdlrs.subscribeConnectionGraphHandler = [this](bool subscribe) {
        _subscribeGraphUpdates = subscribe;
      };

      if (hasCapability(foxglove::CAPABILITY_ASSETS)) {
        hdlrs.fetchAssetHandler = [this](const std::string& uri, uint32_t requestId,
                                         foxglove::ConnHandle hdl) {
          _fetchAssetQueue->addCallback(
            std::bind(&FoxgloveBridge::fetchAsset, this, uri, requestId, hdl));
        };
      }

      _server->setHandlers(std::move(hdlrs));

      _server->start(address, static_cast<uint16_t>(port));

      xmlrpcServer.bind("paramUpdate", std::bind(&FoxgloveBridge::parameterUpdates, this,
                                                 std::placeholders::_1, std::placeholders::_2));
      xmlrpcServer.start();

      updateAdvertisedTopicsAndServices(ros::TimerEvent());

      if (_useSimTime) {
        _clockSubscription = getMTNodeHandle().subscribe<rosgraph_msgs::Clock>(
          "/clock", 10, [&](const rosgraph_msgs::Clock::ConstPtr msg) {
            _server->broadcastTime(msg->clock.toNSec());
          });
      }
    } catch (const std::exception& err) {
      ROS_ERROR("Failed to start websocket server: %s", err.what());
      // Rethrow exception such that the nodelet is unloaded.
      throw err;
    }
  };
  virtual ~FoxgloveBridge() {
    xmlrpcServer.shutdown();
    if (_server) {
      _server->stop();
    }
  }

private:
  struct PairHash {
    template <class T1, class T2>
    std::size_t operator()(const std::pair<T1, T2>& pair) const {
      return std::hash<T1>()(pair.first) ^ std::hash<T2>()(pair.second);
    }
  };

  void subscribe(foxglove::ChannelId channelId, ConnectionHandle clientHandle) {
    std::lock_guard<std::mutex> lock(_subscriptionsMutex);

    auto it = _advertisedTopics.find(channelId);
    if (it == _advertisedTopics.end()) {
      const std::string errMsg =
        "Received subscribe request for unknown channel " + std::to_string(channelId);
      ROS_WARN_STREAM(errMsg);
      throw foxglove::ChannelError(channelId, errMsg);
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
      const std::string errMsg =
        "Client is already subscribed to channel " + std::to_string(channelId);
      ROS_WARN_STREAM(errMsg);
      throw foxglove::ChannelError(channelId, errMsg);
    }

    try {
      subscriptionsByClient.emplace(
        clientHandle, getMTNodeHandle().subscribe<ros_babel_fish::BabelFishMessage>(
                        topic, SUBSCRIPTION_QUEUE_LENGTH,
                        std::bind(&FoxgloveBridge::rosMessageHandler, this, channelId, clientHandle,
                                  std::placeholders::_1)));
      if (firstSubscription) {
        ROS_INFO("Subscribed to topic \"%s\" (%s) on channel %d", topic.c_str(), datatype.c_str(),
                 channelId);

      } else {
        ROS_INFO("Added subscriber #%zu to topic \"%s\" (%s) on channel %d",
                 subscriptionsByClient.size(), topic.c_str(), datatype.c_str(), channelId);
      }
    } catch (const std::exception& ex) {
      const std::string errMsg =
        "Failed to subscribe to topic '" + topic + "' (" + datatype + "): " + ex.what();
      ROS_ERROR_STREAM(errMsg);
      throw foxglove::ChannelError(channelId, errMsg);
    }
  }

  void unsubscribe(foxglove::ChannelId channelId, ConnectionHandle clientHandle) {
    std::lock_guard<std::mutex> lock(_subscriptionsMutex);

    const auto channelIt = _advertisedTopics.find(channelId);
    if (channelIt == _advertisedTopics.end()) {
      const std::string errMsg =
        "Received unsubscribe request for unknown channel " + std::to_string(channelId);
      ROS_WARN_STREAM(errMsg);
      throw foxglove::ChannelError(channelId, errMsg);
    }
    const auto& channel = channelIt->second;

    auto subscriptionsIt = _subscriptions.find(channelId);
    if (subscriptionsIt == _subscriptions.end()) {
      throw foxglove::ChannelError(channelId, "Received unsubscribe request for channel " +
                                                std::to_string(channelId) +
                                                " that was not subscribed to ");
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
      ROS_INFO("Unsubscribing from topic \"%s\" (%s) on channel %d", channel.topic.c_str(),
               channel.schemaName.c_str(), channelId);
      _subscriptions.erase(subscriptionsIt);
    } else {
      ROS_INFO("Removed one subscription from channel %d (%zu subscription(s) left)", channelId,
               subscriptionsByClient.size());
    }
  }

  void clientAdvertise(const foxglove::ClientAdvertisement& channel,
                       ConnectionHandle clientHandle) {
    if (channel.encoding != ROS1_CHANNEL_ENCODING) {
      throw foxglove::ClientChannelError(
        channel.channelId, "Unsupported encoding. Only '" + std::string(ROS1_CHANNEL_ENCODING) +
                             "' encoding is supported at the moment.");
    }

    std::unique_lock<std::shared_mutex> lock(_publicationsMutex);

    // Get client publications or insert an empty map.
    auto [clientPublicationsIt, isFirstPublication] =
      _clientAdvertisedTopics.emplace(clientHandle, ClientPublications());

    auto& clientPublications = clientPublicationsIt->second;
    if (!isFirstPublication &&
        clientPublications.find(channel.channelId) != clientPublications.end()) {
      throw foxglove::ClientChannelError(
        channel.channelId, "Received client advertisement from " +
                             _server->remoteEndpointString(clientHandle) + " for channel " +
                             std::to_string(channel.channelId) + " it had already advertised");
    }

    const auto msgDescription = _rosTypeInfoProvider.getMessageDescription(channel.schemaName);
    if (!msgDescription) {
      throw foxglove::ClientChannelError(
        channel.channelId, "Failed to retrieve type information of data type '" +
                             channel.schemaName + "'. Unable to advertise topic " + channel.topic);
    }

    ros::AdvertiseOptions advertiseOptions;
    advertiseOptions.datatype = channel.schemaName;
    advertiseOptions.has_header = false;  // TODO
    advertiseOptions.latch = false;
    advertiseOptions.md5sum = msgDescription->md5;
    advertiseOptions.message_definition = msgDescription->message_definition;
    advertiseOptions.queue_size = PUBLICATION_QUEUE_LENGTH;
    advertiseOptions.topic = channel.topic;
    auto publisher = getMTNodeHandle().advertise(advertiseOptions);

    if (publisher) {
      clientPublications.insert({channel.channelId, std::move(publisher)});
      ROS_INFO("Client %s is advertising \"%s\" (%s) on channel %d",
               _server->remoteEndpointString(clientHandle).c_str(), channel.topic.c_str(),
               channel.schemaName.c_str(), channel.channelId);
      // Trigger topic discovery so other clients are immediately informed about this new topic.
      updateAdvertisedTopics();
    } else {
      const auto errMsg =
        "Failed to create publisher for topic " + channel.topic + "(" + channel.schemaName + ")";
      ROS_ERROR_STREAM(errMsg);
      throw foxglove::ClientChannelError(channel.channelId, errMsg);
    }
  }

  void clientUnadvertise(foxglove::ClientChannelId channelId, ConnectionHandle clientHandle) {
    std::unique_lock<std::shared_mutex> lock(_publicationsMutex);

    auto clientPublicationsIt = _clientAdvertisedTopics.find(clientHandle);
    if (clientPublicationsIt == _clientAdvertisedTopics.end()) {
      throw foxglove::ClientChannelError(
        channelId, "Ignoring client unadvertisement from " +
                     _server->remoteEndpointString(clientHandle) + " for unknown channel " +
                     std::to_string(channelId) + ", client has no advertised topics");
    }

    auto& clientPublications = clientPublicationsIt->second;

    auto channelPublicationIt = clientPublications.find(channelId);
    if (channelPublicationIt == clientPublications.end()) {
      throw foxglove::ClientChannelError(
        channelId, "Ignoring client unadvertisement from " +
                     _server->remoteEndpointString(clientHandle) + " for unknown channel " +
                     std::to_string(channelId) + ", client has " +
                     std::to_string(clientPublications.size()) + " advertised topic(s)");
    }

    const auto& publisher = channelPublicationIt->second;
    ROS_INFO("Client %s is no longer advertising %s (%d subscribers) on channel %d",
             _server->remoteEndpointString(clientHandle).c_str(), publisher.getTopic().c_str(),
             publisher.getNumSubscribers(), channelId);
    clientPublications.erase(channelPublicationIt);

    if (clientPublications.empty()) {
      _clientAdvertisedTopics.erase(clientPublicationsIt);
    }
  }

  void clientMessage(const foxglove::ClientMessage& clientMsg, ConnectionHandle clientHandle) {
    ros_babel_fish::BabelFishMessage::Ptr msg(new ros_babel_fish::BabelFishMessage);
    msg->read(clientMsg);

    const auto channelId = clientMsg.advertisement.channelId;
    std::shared_lock<std::shared_mutex> lock(_publicationsMutex);

    auto clientPublicationsIt = _clientAdvertisedTopics.find(clientHandle);
    if (clientPublicationsIt == _clientAdvertisedTopics.end()) {
      throw foxglove::ClientChannelError(
        channelId, "Dropping client message from " + _server->remoteEndpointString(clientHandle) +
                     " for unknown channel " + std::to_string(channelId) +
                     ", client has no advertised topics");
    }

    auto& clientPublications = clientPublicationsIt->second;

    auto channelPublicationIt = clientPublications.find(clientMsg.advertisement.channelId);
    if (channelPublicationIt == clientPublications.end()) {
      throw foxglove::ClientChannelError(
        channelId, "Dropping client message from " + _server->remoteEndpointString(clientHandle) +
                     " for unknown channel " + std::to_string(channelId) + ", client has " +
                     std::to_string(clientPublications.size()) + " advertised topic(s)");
    }

    try {
      channelPublicationIt->second.publish(msg);
    } catch (const std::exception& ex) {
      throw foxglove::ClientChannelError(channelId, "Failed to publish message on topic '" +
                                                      channelPublicationIt->second.getTopic() +
                                                      "': " + ex.what());
    }
  }

  void updateAdvertisedTopicsAndServices(const ros::TimerEvent&) {
    _updateTimer.stop();
    if (!ros::ok()) {
      return;
    }

    const bool servicesEnabled = hasCapability(foxglove::CAPABILITY_SERVICES);
    const bool querySystemState = servicesEnabled || _subscribeGraphUpdates;

    std::vector<std::string> serviceNames;
    foxglove::MapOfSets publishers, subscribers, services;

    // Retrieve system state from ROS master.
    if (querySystemState) {
      XmlRpc::XmlRpcValue params, result, payload;
      params[0] = this->getName();
      if (ros::master::execute("getSystemState", params, result, payload, false) &&
          static_cast<int>(result[0]) == 1) {
        const auto& systemState = result[2];
        const auto& publishersXmlRpc = systemState[0];
        const auto& subscribersXmlRpc = systemState[1];
        const auto& servicesXmlRpc = systemState[2];

        for (int i = 0; i < servicesXmlRpc.size(); ++i) {
          const std::string& name = servicesXmlRpc[i][0];
          if (isWhitelisted(name, _serviceWhitelistPatterns)) {
            serviceNames.push_back(name);
            services.emplace(name, rpcValueToStringSet(servicesXmlRpc[i][1]));
          }
        }
        for (int i = 0; i < publishersXmlRpc.size(); ++i) {
          const std::string& name = publishersXmlRpc[i][0];
          if (isWhitelisted(name, _topicWhitelistPatterns)) {
            publishers.emplace(name, rpcValueToStringSet(publishersXmlRpc[i][1]));
          }
        }
        for (int i = 0; i < subscribersXmlRpc.size(); ++i) {
          const std::string& name = subscribersXmlRpc[i][0];
          if (isWhitelisted(name, _topicWhitelistPatterns)) {
            subscribers.emplace(name, rpcValueToStringSet(subscribersXmlRpc[i][1]));
          }
        }
      } else {
        ROS_WARN("Failed to call getSystemState: %s", result.toXml().c_str());
      }
    }

    updateAdvertisedTopics();
    if (servicesEnabled) {
      updateAdvertisedServices(serviceNames);
    }
    if (_subscribeGraphUpdates) {
      _server->updateConnectionGraph(publishers, subscribers, services);
    }

    // Schedule the next update using truncated exponential backoff, between `MIN_UPDATE_PERIOD_MS`
    // and `_maxUpdateMs`
    _updateCount++;
    const auto nextUpdateMs = std::max(
      MIN_UPDATE_PERIOD_MS, static_cast<double>(std::min(size_t(1) << _updateCount, _maxUpdateMs)));
    _updateTimer = getMTNodeHandle().createTimer(
      ros::Duration(nextUpdateMs / 1e3), &FoxgloveBridge::updateAdvertisedTopicsAndServices, this);
  }

  void updateAdvertisedTopics() {
    // Get the current list of visible topics and datatypes from the ROS graph
    std::vector<ros::master::TopicInfo> topicNamesAndTypes;
    if (!ros::master::getTopics(topicNamesAndTypes)) {
      ROS_WARN("Failed to retrieve published topics from ROS master.");
      return;
    }

    std::unordered_set<TopicAndDatatype, PairHash> latestTopics;
    latestTopics.reserve(topicNamesAndTypes.size());
    for (const auto& topicNameAndType : topicNamesAndTypes) {
      const auto& topicName = topicNameAndType.name;
      const auto& datatype = topicNameAndType.datatype;

      // Ignore the topic if it is not on the topic whitelist
      if (isWhitelisted(topicName, _topicWhitelistPatterns)) {
        latestTopics.emplace(topicName, datatype);
      }
    }

    if (const auto numIgnoredTopics = topicNamesAndTypes.size() - latestTopics.size()) {
      ROS_DEBUG(
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
        ROS_DEBUG("Removed channel %d for topic \"%s\" (%s)", channelId,
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
      newChannel.encoding = ROS1_CHANNEL_ENCODING;

      try {
        const auto msgDescription =
          _rosTypeInfoProvider.getMessageDescription(topicAndDatatype.second);
        if (msgDescription) {
          newChannel.schema = msgDescription->message_definition;
        } else {
          ROS_WARN("Could not find definition for type %s", topicAndDatatype.second.c_str());

          // We still advertise the channel, but with an emtpy schema
          newChannel.schema = "";
        }
      } catch (const std::exception& err) {
        ROS_WARN("Failed to add channel for topic \"%s\" (%s): %s", topicAndDatatype.first.c_str(),
                 topicAndDatatype.second.c_str(), err.what());
        continue;
      }

      channelsToAdd.push_back(newChannel);
    }

    const auto channelIds = _server->addChannels(channelsToAdd);
    for (size_t i = 0; i < channelsToAdd.size(); ++i) {
      const auto channelId = channelIds[i];
      const auto& channel = channelsToAdd[i];
      _advertisedTopics.emplace(channelId, channel);
      ROS_DEBUG("Advertising channel %d for topic \"%s\" (%s)", channelId, channel.topic.c_str(),
                channel.schemaName.c_str());
    }
  }

  void updateAdvertisedServices(const std::vector<std::string>& serviceNames) {
    std::unique_lock<std::shared_mutex> lock(_servicesMutex);

    // Remove advertisements for services that have been removed
    std::vector<foxglove::ServiceId> servicesToRemove;
    for (const auto& service : _advertisedServices) {
      const auto it =
        std::find_if(serviceNames.begin(), serviceNames.end(), [service](const auto& serviceName) {
          return serviceName == service.second.name;
        });
      if (it == serviceNames.end()) {
        servicesToRemove.push_back(service.first);
      }
    }
    for (auto serviceId : servicesToRemove) {
      _advertisedServices.erase(serviceId);
    }
    _server->removeServices(servicesToRemove);

    // Advertise new services
    std::vector<foxglove::ServiceWithoutId> newServices;
    for (const auto& serviceName : serviceNames) {
      if (std::find_if(_advertisedServices.begin(), _advertisedServices.end(),
                       [&serviceName](const auto& idWithService) {
                         return idWithService.second.name == serviceName;
                       }) != _advertisedServices.end()) {
        continue;  // Already advertised
      }

      try {
        const auto serviceType =
          retrieveServiceType(serviceName, std::chrono::milliseconds(_serviceRetrievalTimeoutMs));
        const auto srvDescription = _rosTypeInfoProvider.getServiceDescription(serviceType);

        foxglove::ServiceWithoutId service;
        service.name = serviceName;
        service.type = serviceType;

        if (srvDescription) {
          service.requestSchema = srvDescription->request->message_definition;
          service.responseSchema = srvDescription->response->message_definition;
        } else {
          ROS_ERROR("Failed to retrieve type information for service '%s' of type '%s'",
                    serviceName.c_str(), serviceType.c_str());

          // We still advertise the channel, but with empty schema.
          service.requestSchema = "";
          service.responseSchema = "";
        }
        newServices.push_back(service);
      } catch (const std::exception& e) {
        ROS_ERROR("Failed to retrieve service type or service description of service %s: %s",
                  serviceName.c_str(), e.what());
        continue;
      }
    }

    const auto serviceIds = _server->addServices(newServices);
    for (size_t i = 0; i < serviceIds.size(); ++i) {
      _advertisedServices.emplace(serviceIds[i], newServices[i]);
    }
  }

  void getParameters(const std::vector<std::string>& parameters,
                     const std::optional<std::string>& requestId, ConnectionHandle hdl) {
    const bool allParametersRequested = parameters.empty();
    std::vector<std::string> parameterNames = parameters;
    if (allParametersRequested) {
      if (!getMTNodeHandle().getParamNames(parameterNames)) {
        const auto errMsg = "Failed to retrieve parameter names";
        ROS_ERROR_STREAM(errMsg);
        throw std::runtime_error(errMsg);
      }
    }

    bool success = true;
    std::vector<foxglove::Parameter> params;
    for (const auto& paramName : parameterNames) {
      if (!isWhitelisted(paramName, _paramWhitelistPatterns)) {
        if (allParametersRequested) {
          continue;
        } else {
          ROS_ERROR("Parameter '%s' is not on the allowlist", paramName.c_str());
          success = false;
        }
      }

      try {
        XmlRpc::XmlRpcValue value;
        getMTNodeHandle().getParam(paramName, value);
        params.push_back(fromRosParam(paramName, value));
      } catch (const std::exception& ex) {
        ROS_ERROR("Invalid parameter '%s': %s", paramName.c_str(), ex.what());
        success = false;
      } catch (const XmlRpc::XmlRpcException& ex) {
        ROS_ERROR("Invalid parameter '%s': %s", paramName.c_str(), ex.getMessage().c_str());
        success = false;
      } catch (...) {
        ROS_ERROR("Invalid parameter '%s'", paramName.c_str());
        success = false;
      }
    }

    _server->publishParameterValues(hdl, params, requestId);

    if (!success) {
      throw std::runtime_error("Failed to retrieve one or multiple parameters");
    }
  }

  void setParameters(const std::vector<foxglove::Parameter>& parameters,
                     const std::optional<std::string>& requestId, ConnectionHandle hdl) {
    using foxglove::ParameterType;
    auto nh = this->getMTNodeHandle();

    bool success = true;
    for (const auto& param : parameters) {
      const auto paramName = param.getName();
      if (!isWhitelisted(paramName, _paramWhitelistPatterns)) {
        ROS_ERROR("Parameter '%s' is not on the allowlist", paramName.c_str());
        success = false;
        continue;
      }

      try {
        const auto paramType = param.getType();
        const auto paramValue = param.getValue();
        if (paramType == ParameterType::PARAMETER_NOT_SET) {
          nh.deleteParam(paramName);
        } else {
          nh.setParam(paramName, toRosParam(paramValue));
        }
      } catch (const std::exception& ex) {
        ROS_ERROR("Failed to set parameter '%s': %s", paramName.c_str(), ex.what());
        success = false;
      } catch (const XmlRpc::XmlRpcException& ex) {
        ROS_ERROR("Failed to set parameter '%s': %s", paramName.c_str(), ex.getMessage().c_str());
        success = false;
      } catch (...) {
        ROS_ERROR("Failed to set parameter '%s'", paramName.c_str());
        success = false;
      }
    }

    // If a request Id was given, send potentially updated parameters back to client
    if (requestId) {
      std::vector<std::string> parameterNames(parameters.size());
      for (size_t i = 0; i < parameters.size(); ++i) {
        parameterNames[i] = parameters[i].getName();
      }
      getParameters(parameterNames, requestId, hdl);
    }

    if (!success) {
      throw std::runtime_error("Failed to set one or multiple parameters");
    }
  }

  void subscribeParameters(const std::vector<std::string>& parameters,
                           foxglove::ParameterSubscriptionOperation op, ConnectionHandle) {
    const auto opVerb =
      (op == foxglove::ParameterSubscriptionOperation::SUBSCRIBE) ? "subscribe" : "unsubscribe";
    bool success = true;
    for (const auto& paramName : parameters) {
      if (!isWhitelisted(paramName, _paramWhitelistPatterns)) {
        ROS_ERROR("Parameter '%s' is not allowlist", paramName.c_str());
        continue;
      }

      XmlRpc::XmlRpcValue params, result, payload;
      params[0] = getName() + "2";
      params[1] = xmlrpcServer.getServerURI();
      params[2] = ros::names::resolve(paramName);

      const std::string opName = std::string(opVerb) + "Param";
      if (ros::master::execute(opName, params, result, payload, false)) {
        ROS_DEBUG("%s '%s'", opName.c_str(), paramName.c_str());
      } else {
        ROS_WARN("Failed to %s '%s': %s", opVerb, paramName.c_str(), result.toXml().c_str());
        success = false;
      }
    }

    if (!success) {
      throw std::runtime_error("Failed to " + std::string(opVerb) + " one or multiple parameters.");
    }
  }

  void parameterUpdates(XmlRpc::XmlRpcValue& params, XmlRpc::XmlRpcValue& result) {
    result[0] = 1;
    result[1] = std::string("");
    result[2] = 0;

    if (params.size() != 3) {
      ROS_ERROR("Parameter update called with invalid parameter size: %d", params.size());
      return;
    }

    try {
      const std::string paramName = ros::names::clean(params[1]);
      const XmlRpc::XmlRpcValue paramValue = params[2];
      const auto param = fromRosParam(paramName, paramValue);
      _server->updateParameterValues({param});
    } catch (const std::exception& ex) {
      ROS_ERROR("Failed to update parameter: %s", ex.what());
    } catch (const XmlRpc::XmlRpcException& ex) {
      ROS_ERROR("Failed to update parameter: %s", ex.getMessage().c_str());
    } catch (...) {
      ROS_ERROR("Failed to update parameter");
    }
  }

  void logHandler(foxglove::WebSocketLogLevel level, char const* msg) {
    switch (level) {
      case foxglove::WebSocketLogLevel::Debug:
        ROS_DEBUG("[WS] %s", msg);
        break;
      case foxglove::WebSocketLogLevel::Info:
        ROS_INFO("[WS] %s", msg);
        break;
      case foxglove::WebSocketLogLevel::Warn:
        ROS_WARN("[WS] %s", msg);
        break;
      case foxglove::WebSocketLogLevel::Error:
        ROS_ERROR("[WS] %s", msg);
        break;
      case foxglove::WebSocketLogLevel::Critical:
        ROS_FATAL("[WS] %s", msg);
        break;
    }
  }

  void rosMessageHandler(
    const foxglove::ChannelId channelId, ConnectionHandle clientHandle,
    const ros::MessageEvent<ros_babel_fish::BabelFishMessage const>& msgEvent) {
    const auto& msg = msgEvent.getConstMessage();
    const auto receiptTimeNs = msgEvent.getReceiptTime().toNSec();
    _server->sendMessage(clientHandle, channelId, receiptTimeNs, msg->buffer(), msg->size());
  }

  void serviceRequest(const foxglove::ServiceRequest& request, ConnectionHandle clientHandle) {
    std::shared_lock<std::shared_mutex> lock(_servicesMutex);
    const auto serviceIt = _advertisedServices.find(request.serviceId);
    if (serviceIt == _advertisedServices.end()) {
      const auto errMsg =
        "Service with id " + std::to_string(request.serviceId) + " does not exist";
      ROS_ERROR_STREAM(errMsg);
      throw foxglove::ServiceError(request.serviceId, errMsg);
    }
    const auto& serviceName = serviceIt->second.name;
    const auto& serviceType = serviceIt->second.type;
    ROS_DEBUG("Received a service request for service %s (%s)", serviceName.c_str(),
              serviceType.c_str());

    if (!ros::service::exists(serviceName, false)) {
      throw foxglove::ServiceError(request.serviceId,
                                   "Service '" + serviceName + "' does not exist");
    }

    const auto srvDescription = _rosTypeInfoProvider.getServiceDescription(serviceType);
    if (!srvDescription) {
      const auto errMsg =
        "Failed to retrieve type information for service " + serviceName + "(" + serviceType + ")";
      ROS_ERROR_STREAM(errMsg);
      throw foxglove::ServiceError(request.serviceId, errMsg);
    }

    GenericService genReq, genRes;
    genReq.type = genRes.type = serviceType;
    genReq.md5sum = genRes.md5sum = srvDescription->md5;
    genReq.data = request.data;

    if (ros::service::call(serviceName, genReq, genRes)) {
      foxglove::ServiceResponse res;
      res.serviceId = request.serviceId;
      res.callId = request.callId;
      res.encoding = request.encoding;
      res.data = genRes.data;
      _server->sendServiceResponse(clientHandle, res);
    } else {
      throw foxglove::ServiceError(
        request.serviceId, "Failed to call service " + serviceName + "(" + serviceType + ")");
    }
  }

  void fetchAsset(const std::string& uri, uint32_t requestId, ConnectionHandle clientHandle) {
    foxglove::FetchAssetResponse response;
    response.requestId = requestId;

    try {
      // We reject URIs that are not on the allowlist or that contain two consecutive dots. The
      // latter can be utilized to construct URIs for retrieving confidential files that should not
      // be accessible over the WebSocket connection. Example:
      // `package://<pkg_name>/../../../secret.txt`. This is an extra security measure and should
      // not be necessary if the allowlist is strict enough.
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
      ROS_WARN("Failed to retrieve asset '%s': %s", uri.c_str(), ex.what());
      response.status = foxglove::FetchAssetStatus::Error;
      response.errorMessage = "Failed to retrieve asset " + uri;
    }

    if (_server) {
      _server->sendFetchAssetResponse(clientHandle, response);
    }
  }

  bool hasCapability(const std::string& capability) {
    return std::find(_capabilities.begin(), _capabilities.end(), capability) != _capabilities.end();
  }

  std::unique_ptr<foxglove::ServerInterface<ConnectionHandle>> _server;
  ros_babel_fish::IntegratedDescriptionProvider _rosTypeInfoProvider;
  std::vector<std::regex> _topicWhitelistPatterns;
  std::vector<std::regex> _paramWhitelistPatterns;
  std::vector<std::regex> _serviceWhitelistPatterns;
  std::vector<std::regex> _assetUriAllowlistPatterns;
  ros::XMLRPCManager xmlrpcServer;
  std::unordered_map<foxglove::ChannelId, foxglove::ChannelWithoutId> _advertisedTopics;
  std::unordered_map<foxglove::ChannelId, SubscriptionsByClient> _subscriptions;
  std::unordered_map<foxglove::ServiceId, foxglove::ServiceWithoutId> _advertisedServices;
  PublicationsByClient _clientAdvertisedTopics;
  std::mutex _subscriptionsMutex;
  std::shared_mutex _publicationsMutex;
  std::shared_mutex _servicesMutex;
  ros::Timer _updateTimer;
  size_t _maxUpdateMs = size_t(DEFAULT_MAX_UPDATE_MS);
  size_t _updateCount = 0;
  ros::Subscriber _clockSubscription;
  bool _useSimTime = false;
  std::vector<std::string> _capabilities;
  int _serviceRetrievalTimeoutMs = DEFAULT_SERVICE_TYPE_RETRIEVAL_TIMEOUT_MS;
  std::atomic<bool> _subscribeGraphUpdates = false;
  std::unique_ptr<foxglove::CallbackQueue> _fetchAssetQueue;
};

}  // namespace foxglove_bridge

PLUGINLIB_EXPORT_CLASS(foxglove_bridge::FoxgloveBridge, nodelet::Nodelet)
