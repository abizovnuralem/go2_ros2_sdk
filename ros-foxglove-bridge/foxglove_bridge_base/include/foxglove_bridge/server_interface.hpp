#pragma once

#include <functional>
#include <optional>
#include <regex>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "common.hpp"
#include "parameter.hpp"

namespace foxglove {

constexpr size_t DEFAULT_SEND_BUFFER_LIMIT_BYTES = 10000000UL;  // 10 MB

using MapOfSets = std::unordered_map<std::string, std::unordered_set<std::string>>;

template <typename IdType>
class ExeptionWithId : public std::runtime_error {
public:
  explicit ExeptionWithId(IdType id, const std::string& what_arg)
      : std::runtime_error(what_arg)
      , _id(id) {}

  IdType id() const {
    return _id;
  }

private:
  IdType _id;
};

class ChannelError : public ExeptionWithId<ChannelId> {
  using ExeptionWithId::ExeptionWithId;
};
class ClientChannelError : public ExeptionWithId<ClientChannelId> {
  using ExeptionWithId::ExeptionWithId;
};
class ServiceError : public ExeptionWithId<ServiceId> {
  using ExeptionWithId::ExeptionWithId;
};

struct ServerOptions {
  std::vector<std::string> capabilities;
  std::vector<std::string> supportedEncodings;
  std::unordered_map<std::string, std::string> metadata;
  size_t sendBufferLimitBytes = DEFAULT_SEND_BUFFER_LIMIT_BYTES;
  bool useTls = false;
  std::string certfile = "";
  std::string keyfile = "";
  std::string sessionId;
  bool useCompression = false;
  std::vector<std::regex> clientTopicWhitelistPatterns;
};

template <typename ConnectionHandle>
struct ServerHandlers {
  std::function<void(ChannelId, ConnectionHandle)> subscribeHandler;
  std::function<void(ChannelId, ConnectionHandle)> unsubscribeHandler;
  std::function<void(const ClientAdvertisement&, ConnectionHandle)> clientAdvertiseHandler;
  std::function<void(ClientChannelId, ConnectionHandle)> clientUnadvertiseHandler;
  std::function<void(const ClientMessage&, ConnectionHandle)> clientMessageHandler;
  std::function<void(const std::vector<std::string>&, const std::optional<std::string>&,
                     ConnectionHandle)>
    parameterRequestHandler;
  std::function<void(const std::vector<Parameter>&, const std::optional<std::string>&,
                     ConnectionHandle)>
    parameterChangeHandler;
  std::function<void(const std::vector<std::string>&, ParameterSubscriptionOperation,
                     ConnectionHandle)>
    parameterSubscriptionHandler;
  std::function<void(const ServiceRequest&, ConnectionHandle)> serviceRequestHandler;
  std::function<void(bool)> subscribeConnectionGraphHandler;
  std::function<void(const std::string&, uint32_t, ConnectionHandle)> fetchAssetHandler;
};

template <typename ConnectionHandle>
class ServerInterface {
public:
  virtual ~ServerInterface() {}
  virtual void start(const std::string& host, uint16_t port) = 0;
  virtual void stop() = 0;

  virtual std::vector<ChannelId> addChannels(const std::vector<ChannelWithoutId>& channels) = 0;
  virtual void removeChannels(const std::vector<ChannelId>& channelIds) = 0;
  virtual void publishParameterValues(ConnectionHandle clientHandle,
                                      const std::vector<Parameter>& parameters,
                                      const std::optional<std::string>& requestId) = 0;
  virtual void updateParameterValues(const std::vector<Parameter>& parameters) = 0;
  virtual std::vector<ServiceId> addServices(const std::vector<ServiceWithoutId>& services) = 0;
  virtual void removeServices(const std::vector<ServiceId>& serviceIds) = 0;

  virtual void setHandlers(ServerHandlers<ConnectionHandle>&& handlers) = 0;

  virtual void sendMessage(ConnectionHandle clientHandle, ChannelId chanId, uint64_t timestamp,
                           const uint8_t* payload, size_t payloadSize) = 0;
  virtual void broadcastTime(uint64_t timestamp) = 0;
  virtual void sendServiceResponse(ConnectionHandle clientHandle,
                                   const ServiceResponse& response) = 0;
  virtual void updateConnectionGraph(const MapOfSets& publishedTopics,
                                     const MapOfSets& subscribedTopics,
                                     const MapOfSets& advertisedServices) = 0;
  virtual void sendFetchAssetResponse(ConnectionHandle clientHandle,
                                      const FetchAssetResponse& response) = 0;

  virtual uint16_t getPort() = 0;
  virtual std::string remoteEndpointString(ConnectionHandle clientHandle) = 0;
};

}  // namespace foxglove
