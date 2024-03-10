#pragma once

#include <atomic>
#include <chrono>
#include <memory>
#include <regex>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <websocketpp/common/connection_hdl.hpp>

#include <foxglove_bridge/callback_queue.hpp>
#include <foxglove_bridge/foxglove_bridge.hpp>
#include <foxglove_bridge/generic_client.hpp>
#include <foxglove_bridge/message_definition_cache.hpp>
#include <foxglove_bridge/param_utils.hpp>
#include <foxglove_bridge/parameter_interface.hpp>
#include <foxglove_bridge/regex_utils.hpp>
#include <foxglove_bridge/server_factory.hpp>
#include <foxglove_bridge/utils.hpp>

namespace foxglove_bridge {

using ConnectionHandle = websocketpp::connection_hdl;
using LogLevel = foxglove::WebSocketLogLevel;
using Subscription = rclcpp::GenericSubscription::SharedPtr;
using SubscriptionsByClient = std::map<ConnectionHandle, Subscription, std::owner_less<>>;
using Publication = rclcpp::GenericPublisher::SharedPtr;
using ClientPublications = std::unordered_map<foxglove::ClientChannelId, Publication>;
using PublicationsByClient = std::map<ConnectionHandle, ClientPublications, std::owner_less<>>;

class FoxgloveBridge : public rclcpp::Node {
public:
  using TopicAndDatatype = std::pair<std::string, std::string>;

  FoxgloveBridge(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

  ~FoxgloveBridge();

  void rosgraphPollThread();

  void updateAdvertisedTopics(
    const std::map<std::string, std::vector<std::string>>& topicNamesAndTypes);

  void updateAdvertisedServices();

  void updateConnectionGraph(
    const std::map<std::string, std::vector<std::string>>& topicNamesAndTypes);

private:
  struct PairHash {
    template <class T1, class T2>
    std::size_t operator()(const std::pair<T1, T2>& pair) const {
      return std::hash<T1>()(pair.first) ^ std::hash<T2>()(pair.second);
    }
  };

  std::unique_ptr<foxglove::ServerInterface<ConnectionHandle>> _server;
  foxglove::MessageDefinitionCache _messageDefinitionCache;
  std::vector<std::regex> _topicWhitelistPatterns;
  std::vector<std::regex> _serviceWhitelistPatterns;
  std::vector<std::regex> _assetUriAllowlistPatterns;
  std::shared_ptr<ParameterInterface> _paramInterface;
  std::unordered_map<foxglove::ChannelId, foxglove::ChannelWithoutId> _advertisedTopics;
  std::unordered_map<foxglove::ServiceId, foxglove::ServiceWithoutId> _advertisedServices;
  std::unordered_map<foxglove::ChannelId, SubscriptionsByClient> _subscriptions;
  PublicationsByClient _clientAdvertisedTopics;
  std::unordered_map<foxglove::ServiceId, GenericClient::SharedPtr> _serviceClients;
  rclcpp::CallbackGroup::SharedPtr _subscriptionCallbackGroup;
  rclcpp::CallbackGroup::SharedPtr _clientPublishCallbackGroup;
  rclcpp::CallbackGroup::SharedPtr _servicesCallbackGroup;
  std::mutex _subscriptionsMutex;
  std::mutex _clientAdvertisementsMutex;
  std::mutex _servicesMutex;
  std::unique_ptr<std::thread> _rosgraphPollThread;
  size_t _minQosDepth = DEFAULT_MIN_QOS_DEPTH;
  size_t _maxQosDepth = DEFAULT_MAX_QOS_DEPTH;
  std::shared_ptr<rclcpp::Subscription<rosgraph_msgs::msg::Clock>> _clockSubscription;
  bool _useSimTime = false;
  std::vector<std::string> _capabilities;
  std::atomic<bool> _subscribeGraphUpdates = false;
  bool _includeHidden = false;
  std::unique_ptr<foxglove::CallbackQueue> _fetchAssetQueue;

  void subscribeConnectionGraph(bool subscribe);

  void subscribe(foxglove::ChannelId channelId, ConnectionHandle clientHandle);

  void unsubscribe(foxglove::ChannelId channelId, ConnectionHandle clientHandle);

  void clientAdvertise(const foxglove::ClientAdvertisement& advertisement, ConnectionHandle hdl);

  void clientUnadvertise(foxglove::ChannelId channelId, ConnectionHandle hdl);

  void clientMessage(const foxglove::ClientMessage& message, ConnectionHandle hdl);

  void setParameters(const std::vector<foxglove::Parameter>& parameters,
                     const std::optional<std::string>& requestId, ConnectionHandle hdl);

  void getParameters(const std::vector<std::string>& parameters,
                     const std::optional<std::string>& requestId, ConnectionHandle hdl);

  void subscribeParameters(const std::vector<std::string>& parameters,
                           foxglove::ParameterSubscriptionOperation op, ConnectionHandle);

  void parameterUpdates(const std::vector<foxglove::Parameter>& parameters);

  void logHandler(LogLevel level, char const* msg);

  void rosMessageHandler(const foxglove::ChannelId& channelId, ConnectionHandle clientHandle,
                         std::shared_ptr<rclcpp::SerializedMessage> msg);

  void serviceRequest(const foxglove::ServiceRequest& request, ConnectionHandle clientHandle);

  void fetchAsset(const std::string& assetId, uint32_t requestId, ConnectionHandle clientHandle);

  bool hasCapability(const std::string& capability);
};

}  // namespace foxglove_bridge
