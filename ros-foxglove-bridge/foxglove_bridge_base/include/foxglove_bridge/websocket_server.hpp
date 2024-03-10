#pragma once

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <optional>
#include <shared_mutex>
#include <string_view>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <nlohmann/json.hpp>
#include <websocketpp/config/asio.hpp>
#include <websocketpp/server.hpp>

#include "callback_queue.hpp"
#include "common.hpp"
#include "parameter.hpp"
#include "regex_utils.hpp"
#include "serialization.hpp"
#include "server_interface.hpp"
#include "websocket_logging.hpp"

// Debounce a function call (tied to the line number)
// This macro takes in a function and the debounce time in milliseconds
#define FOXGLOVE_DEBOUNCE(f, ms)                                                               \
  {                                                                                            \
    static auto last_call = std::chrono::system_clock::now();                                  \
    const auto now = std::chrono::system_clock::now();                                         \
    if (std::chrono::duration_cast<std::chrono::milliseconds>(now - last_call).count() > ms) { \
      last_call = now;                                                                         \
      f();                                                                                     \
    }                                                                                          \
  }

namespace {

constexpr uint32_t StringHash(const std::string_view str) {
  uint32_t result = 0x811C9DC5;  // FNV-1a 32-bit algorithm
  for (char c : str) {
    result = (static_cast<uint32_t>(c) ^ result) * 0x01000193;
  }
  return result;
}

constexpr auto SUBSCRIBE = StringHash("subscribe");
constexpr auto UNSUBSCRIBE = StringHash("unsubscribe");
constexpr auto ADVERTISE = StringHash("advertise");
constexpr auto UNADVERTISE = StringHash("unadvertise");
constexpr auto GET_PARAMETERS = StringHash("getParameters");
constexpr auto SET_PARAMETERS = StringHash("setParameters");
constexpr auto SUBSCRIBE_PARAMETER_UPDATES = StringHash("subscribeParameterUpdates");
constexpr auto UNSUBSCRIBE_PARAMETER_UPDATES = StringHash("unsubscribeParameterUpdates");
constexpr auto SUBSCRIBE_CONNECTION_GRAPH = StringHash("subscribeConnectionGraph");
constexpr auto UNSUBSCRIBE_CONNECTION_GRAPH = StringHash("unsubscribeConnectionGraph");
constexpr auto FETCH_ASSET = StringHash("fetchAsset");
}  // namespace

namespace foxglove {

using json = nlohmann::json;

using ConnHandle = websocketpp::connection_hdl;
using OpCode = websocketpp::frame::opcode::value;

static const websocketpp::log::level APP = websocketpp::log::alevel::app;
static const websocketpp::log::level WARNING = websocketpp::log::elevel::warn;
static const websocketpp::log::level RECOVERABLE = websocketpp::log::elevel::rerror;

/// Map of required capability by client operation (text).
const std::unordered_map<std::string, std::string> CAPABILITY_BY_CLIENT_OPERATION = {
  // {"subscribe", },   // No required capability.
  // {"unsubscribe", }, // No required capability.
  {"advertise", CAPABILITY_CLIENT_PUBLISH},
  {"unadvertise", CAPABILITY_CLIENT_PUBLISH},
  {"getParameters", CAPABILITY_PARAMETERS},
  {"setParameters", CAPABILITY_PARAMETERS},
  {"subscribeParameterUpdates", CAPABILITY_PARAMETERS_SUBSCRIBE},
  {"unsubscribeParameterUpdates", CAPABILITY_PARAMETERS_SUBSCRIBE},
  {"subscribeConnectionGraph", CAPABILITY_CONNECTION_GRAPH},
  {"unsubscribeConnectionGraph", CAPABILITY_CONNECTION_GRAPH},
  {"fetchAsset", CAPABILITY_ASSETS},
};

/// Map of required capability by client operation (binary).
const std::unordered_map<ClientBinaryOpcode, std::string> CAPABILITY_BY_CLIENT_BINARY_OPERATION = {
  {ClientBinaryOpcode::MESSAGE_DATA, CAPABILITY_CLIENT_PUBLISH},
  {ClientBinaryOpcode::SERVICE_CALL_REQUEST, CAPABILITY_SERVICES},
};

enum class StatusLevel : uint8_t {
  Info = 0,
  Warning = 1,
  Error = 2,
};

constexpr websocketpp::log::level StatusLevelToLogLevel(StatusLevel level) {
  switch (level) {
    case StatusLevel::Info:
      return APP;
    case StatusLevel::Warning:
      return WARNING;
    case StatusLevel::Error:
      return RECOVERABLE;
    default:
      return RECOVERABLE;
  }
}

template <typename ServerConfiguration>
class Server final : public ServerInterface<ConnHandle> {
public:
  using ServerType = websocketpp::server<ServerConfiguration>;
  using ConnectionType = websocketpp::connection<ServerConfiguration>;
  using MessagePtr = typename ServerType::message_ptr;
  using Tcp = websocketpp::lib::asio::ip::tcp;

  explicit Server(std::string name, LogCallback logger, const ServerOptions& options);
  virtual ~Server();

  Server(const Server&) = delete;
  Server(Server&&) = delete;
  Server& operator=(const Server&) = delete;
  Server& operator=(Server&&) = delete;

  void start(const std::string& host, uint16_t port) override;
  void stop() override;

  std::vector<ChannelId> addChannels(const std::vector<ChannelWithoutId>& channels) override;
  void removeChannels(const std::vector<ChannelId>& channelIds) override;
  void publishParameterValues(ConnHandle clientHandle, const std::vector<Parameter>& parameters,
                              const std::optional<std::string>& requestId = std::nullopt) override;
  void updateParameterValues(const std::vector<Parameter>& parameters) override;
  std::vector<ServiceId> addServices(const std::vector<ServiceWithoutId>& services) override;
  void removeServices(const std::vector<ServiceId>& serviceIds) override;

  void setHandlers(ServerHandlers<ConnHandle>&& handlers) override;

  void sendMessage(ConnHandle clientHandle, ChannelId chanId, uint64_t timestamp,
                   const uint8_t* payload, size_t payloadSize) override;
  void broadcastTime(uint64_t timestamp) override;
  void sendServiceResponse(ConnHandle clientHandle, const ServiceResponse& response) override;
  void updateConnectionGraph(const MapOfSets& publishedTopics, const MapOfSets& subscribedTopics,
                             const MapOfSets& advertisedServices) override;
  void sendFetchAssetResponse(ConnHandle clientHandle, const FetchAssetResponse& response) override;

  uint16_t getPort() override;
  std::string remoteEndpointString(ConnHandle clientHandle) override;

private:
  struct ClientInfo {
    std::string name;
    ConnHandle handle;
    std::unordered_map<ChannelId, SubscriptionId> subscriptionsByChannel;
    std::unordered_set<ClientChannelId> advertisedChannels;
    bool subscribedToConnectionGraph = false;

    explicit ClientInfo(const std::string& name, ConnHandle handle)
        : name(name)
        , handle(handle) {}

    ClientInfo(const ClientInfo&) = delete;
    ClientInfo& operator=(const ClientInfo&) = delete;

    ClientInfo(ClientInfo&&) = default;
    ClientInfo& operator=(ClientInfo&&) = default;
  };

  std::string _name;
  LogCallback _logger;
  ServerOptions _options;
  ServerType _server;
  std::unique_ptr<std::thread> _serverThread;
  std::unique_ptr<CallbackQueue> _handlerCallbackQueue;

  uint32_t _nextChannelId = 0;
  std::map<ConnHandle, ClientInfo, std::owner_less<>> _clients;
  std::unordered_map<ChannelId, Channel> _channels;
  std::map<ConnHandle, std::unordered_map<ClientChannelId, ClientAdvertisement>, std::owner_less<>>
    _clientChannels;
  std::map<ConnHandle, std::unordered_set<std::string>, std::owner_less<>>
    _clientParamSubscriptions;
  ServiceId _nextServiceId = 0;
  std::unordered_map<ServiceId, ServiceWithoutId> _services;
  ServerHandlers<ConnHandle> _handlers;
  std::shared_mutex _clientsMutex;
  std::shared_mutex _channelsMutex;
  std::shared_mutex _clientChannelsMutex;
  std::shared_mutex _servicesMutex;
  std::mutex _clientParamSubscriptionsMutex;

  struct {
    int subscriptionCount = 0;
    MapOfSets publishedTopics;
    MapOfSets subscribedTopics;
    MapOfSets advertisedServices;
  } _connectionGraph;
  std::shared_mutex _connectionGraphMutex;

  void setupTlsHandler();
  void socketInit(ConnHandle hdl);
  bool validateConnection(ConnHandle hdl);
  void handleConnectionOpened(ConnHandle hdl);
  void handleConnectionClosed(ConnHandle hdl);
  void handleMessage(ConnHandle hdl, MessagePtr msg);
  void handleTextMessage(ConnHandle hdl, MessagePtr msg);
  void handleBinaryMessage(ConnHandle hdl, MessagePtr msg);

  void sendJson(ConnHandle hdl, json&& payload);
  void sendJsonRaw(ConnHandle hdl, const std::string& payload);
  void sendBinary(ConnHandle hdl, const uint8_t* payload, size_t payloadSize);
  void sendStatusAndLogMsg(ConnHandle clientHandle, const StatusLevel level,
                           const std::string& message);
  void unsubscribeParamsWithoutSubscriptions(ConnHandle hdl,
                                             const std::unordered_set<std::string>& paramNames);
  bool isParameterSubscribed(const std::string& paramName) const;
  bool hasCapability(const std::string& capability) const;
  bool hasHandler(uint32_t op) const;
  void handleSubscribe(const nlohmann::json& payload, ConnHandle hdl);
  void handleUnsubscribe(const nlohmann::json& payload, ConnHandle hdl);
  void handleAdvertise(const nlohmann::json& payload, ConnHandle hdl);
  void handleUnadvertise(const nlohmann::json& payload, ConnHandle hdl);
  void handleGetParameters(const nlohmann::json& payload, ConnHandle hdl);
  void handleSetParameters(const nlohmann::json& payload, ConnHandle hdl);
  void handleSubscribeParameterUpdates(const nlohmann::json& payload, ConnHandle hdl);
  void handleUnsubscribeParameterUpdates(const nlohmann::json& payload, ConnHandle hdl);
  void handleSubscribeConnectionGraph(ConnHandle hdl);
  void handleUnsubscribeConnectionGraph(ConnHandle hdl);
  void handleFetchAsset(const nlohmann::json& payload, ConnHandle hdl);
};

template <typename ServerConfiguration>
inline Server<ServerConfiguration>::Server(std::string name, LogCallback logger,
                                           const ServerOptions& options)
    : _name(std::move(name))
    , _logger(logger)
    , _options(options) {
  // Redirect logging
  _server.get_alog().set_callback(_logger);
  _server.get_elog().set_callback(_logger);

  websocketpp::lib::error_code ec;
  _server.init_asio(ec);
  if (ec) {
    throw std::runtime_error("Failed to initialize websocket server: " + ec.message());
  }

  _server.clear_access_channels(websocketpp::log::alevel::all);
  _server.set_access_channels(APP);
  _server.set_tcp_pre_init_handler(std::bind(&Server::socketInit, this, std::placeholders::_1));
  this->setupTlsHandler();
  _server.set_validate_handler(std::bind(&Server::validateConnection, this, std::placeholders::_1));
  _server.set_open_handler(std::bind(&Server::handleConnectionOpened, this, std::placeholders::_1));
  _server.set_close_handler([this](ConnHandle hdl) {
    _handlerCallbackQueue->addCallback([this, hdl]() {
      this->handleConnectionClosed(hdl);
    });
  });
  _server.set_message_handler([this](ConnHandle hdl, MessagePtr msg) {
    _handlerCallbackQueue->addCallback([this, hdl, msg]() {
      this->handleMessage(hdl, msg);
    });
  });
  _server.set_reuse_addr(true);
  _server.set_listen_backlog(128);

  // Callback queue for handling client requests and disconnections.
  _handlerCallbackQueue = std::make_unique<CallbackQueue>(_logger, /*numThreads=*/1ul);
}

template <typename ServerConfiguration>
inline Server<ServerConfiguration>::~Server() {}

template <typename ServerConfiguration>
inline void Server<ServerConfiguration>::socketInit(ConnHandle hdl) {
  websocketpp::lib::asio::error_code ec;
  _server.get_con_from_hdl(hdl)->get_raw_socket().set_option(Tcp::no_delay(true), ec);
  if (ec) {
    _server.get_elog().write(RECOVERABLE, "Failed to set TCP_NODELAY: " + ec.message());
  }
}

template <typename ServerConfiguration>
inline bool Server<ServerConfiguration>::validateConnection(ConnHandle hdl) {
  auto con = _server.get_con_from_hdl(hdl);

  const auto& subprotocols = con->get_requested_subprotocols();
  if (std::find(subprotocols.begin(), subprotocols.end(), SUPPORTED_SUBPROTOCOL) !=
      subprotocols.end()) {
    con->select_subprotocol(SUPPORTED_SUBPROTOCOL);
    return true;
  }
  _server.get_alog().write(APP, "Rejecting client " + remoteEndpointString(hdl) +
                                  " which did not declare support for subprotocol " +
                                  SUPPORTED_SUBPROTOCOL);
  return false;
}

template <typename ServerConfiguration>
inline void Server<ServerConfiguration>::handleConnectionOpened(ConnHandle hdl) {
  auto con = _server.get_con_from_hdl(hdl);
  const auto endpoint = remoteEndpointString(hdl);
  _server.get_alog().write(APP, "Client " + endpoint + " connected via " + con->get_resource());

  {
    std::unique_lock<std::shared_mutex> lock(_clientsMutex);
    _clients.emplace(hdl, ClientInfo(endpoint, hdl));
  }

  con->send(json({
                   {"op", "serverInfo"},
                   {"name", _name},
                   {"capabilities", _options.capabilities},
                   {"supportedEncodings", _options.supportedEncodings},
                   {"metadata", _options.metadata},
                   {"sessionId", _options.sessionId},
                 })
              .dump());

  std::vector<Channel> channels;
  {
    std::shared_lock<std::shared_mutex> lock(_channelsMutex);
    for (const auto& [id, channel] : _channels) {
      (void)id;
      channels.push_back(channel);
    }
  }
  sendJson(hdl, {
                  {"op", "advertise"},
                  {"channels", std::move(channels)},
                });

  std::vector<Service> services;
  {
    std::shared_lock<std::shared_mutex> lock(_servicesMutex);
    for (const auto& [id, service] : _services) {
      services.push_back(Service(service, id));
    }
  }
  sendJson(hdl, {
                  {"op", "advertiseServices"},
                  {"services", std::move(services)},
                });
}

template <typename ServerConfiguration>
inline void Server<ServerConfiguration>::handleConnectionClosed(ConnHandle hdl) {
  std::unordered_map<ChannelId, SubscriptionId> oldSubscriptionsByChannel;
  std::unordered_set<ClientChannelId> oldAdvertisedChannels;
  std::string clientName;
  bool wasSubscribedToConnectionGraph;
  {
    std::unique_lock<std::shared_mutex> lock(_clientsMutex);
    const auto clientIt = _clients.find(hdl);
    if (clientIt == _clients.end()) {
      _server.get_elog().write(RECOVERABLE, "Client " + remoteEndpointString(hdl) +
                                              " disconnected but not found in _clients");
      return;
    }

    const auto& client = clientIt->second;
    clientName = client.name;
    _server.get_alog().write(APP, "Client " + clientName + " disconnected");

    oldSubscriptionsByChannel = std::move(client.subscriptionsByChannel);
    oldAdvertisedChannels = std::move(client.advertisedChannels);
    wasSubscribedToConnectionGraph = client.subscribedToConnectionGraph;
    _clients.erase(clientIt);
  }

  // Unadvertise all channels this client advertised
  for (const auto clientChannelId : oldAdvertisedChannels) {
    _server.get_alog().write(APP, "Client " + clientName + " unadvertising channel " +
                                    std::to_string(clientChannelId) + " due to disconnect");
    if (_handlers.clientUnadvertiseHandler) {
      try {
        _handlers.clientUnadvertiseHandler(clientChannelId, hdl);
      } catch (const std::exception& ex) {
        _server.get_elog().write(
          RECOVERABLE, "Exception caught when closing connection: " + std::string(ex.what()));
      } catch (...) {
        _server.get_elog().write(RECOVERABLE, "Exception caught when closing connection");
      }
    }
  }

  {
    std::unique_lock<std::shared_mutex> lock(_clientChannelsMutex);
    _clientChannels.erase(hdl);
  }

  // Unsubscribe all channels this client subscribed to
  if (_handlers.unsubscribeHandler) {
    for (const auto& [chanId, subs] : oldSubscriptionsByChannel) {
      (void)subs;
      try {
        _handlers.unsubscribeHandler(chanId, hdl);
      } catch (const std::exception& ex) {
        _server.get_elog().write(
          RECOVERABLE, "Exception caught when closing connection: " + std::string(ex.what()));
      } catch (...) {
        _server.get_elog().write(RECOVERABLE, "Exception caught when closing connection");
      }
    }
  }

  // Unsubscribe from parameters this client subscribed to
  std::unordered_set<std::string> clientSubscribedParameters;
  {
    std::lock_guard<std::mutex> lock(_clientParamSubscriptionsMutex);
    clientSubscribedParameters = _clientParamSubscriptions[hdl];
    _clientParamSubscriptions.erase(hdl);
  }
  unsubscribeParamsWithoutSubscriptions(hdl, clientSubscribedParameters);

  if (wasSubscribedToConnectionGraph) {
    std::unique_lock<std::shared_mutex> lock(_connectionGraphMutex);
    _connectionGraph.subscriptionCount--;
    if (_connectionGraph.subscriptionCount == 0 && _handlers.subscribeConnectionGraphHandler) {
      _server.get_alog().write(APP, "Unsubscribing from connection graph updates.");
      try {
        _handlers.subscribeConnectionGraphHandler(false);
      } catch (const std::exception& ex) {
        _server.get_elog().write(
          RECOVERABLE, "Exception caught when closing connection: " + std::string(ex.what()));
      } catch (...) {
        _server.get_elog().write(RECOVERABLE, "Exception caught when closing connection");
      }
    }
  }

}  // namespace foxglove

template <typename ServerConfiguration>
inline void Server<ServerConfiguration>::setHandlers(ServerHandlers<ConnHandle>&& handlers) {
  _handlers = handlers;
}

template <typename ServerConfiguration>
inline void Server<ServerConfiguration>::stop() {
  if (_server.stopped()) {
    return;
  }

  _server.get_alog().write(APP, "Stopping WebSocket server");
  websocketpp::lib::error_code ec;

  _server.stop_perpetual();

  if (_server.is_listening()) {
    _server.stop_listening(ec);
    if (ec) {
      _server.get_elog().write(RECOVERABLE, "Failed to stop listening: " + ec.message());
    }
  }

  std::vector<std::shared_ptr<ConnectionType>> connections;
  {
    std::shared_lock<std::shared_mutex> lock(_clientsMutex);
    connections.reserve(_clients.size());
    for (const auto& [hdl, client] : _clients) {
      (void)client;
      if (auto connection = _server.get_con_from_hdl(hdl, ec)) {
        connections.push_back(connection);
      }
    }
  }

  if (!connections.empty()) {
    _server.get_alog().write(
      APP, "Closing " + std::to_string(connections.size()) + " client connection(s)");

    // Iterate over all client connections and start the close connection handshake
    for (const auto& connection : connections) {
      connection->close(websocketpp::close::status::going_away, "server shutdown", ec);
      if (ec) {
        _server.get_elog().write(RECOVERABLE, "Failed to close connection: " + ec.message());
      }
    }

    // Wait for all connections to close
    constexpr size_t MAX_SHUTDOWN_MS = 1000;
    constexpr size_t SLEEP_MS = 10;
    size_t durationMs = 0;
    while (!_server.stopped() && durationMs < MAX_SHUTDOWN_MS) {
      std::this_thread::sleep_for(std::chrono::milliseconds(SLEEP_MS));
      _server.poll_one();
      durationMs += SLEEP_MS;
    }

    if (!_server.stopped()) {
      _server.get_elog().write(RECOVERABLE, "Failed to close all connections, forcefully stopping");
      for (const auto& hdl : connections) {
        if (auto con = _server.get_con_from_hdl(hdl, ec)) {
          _server.get_elog().write(RECOVERABLE,
                                   "Terminating connection to " + remoteEndpointString(hdl));
          con->terminate(ec);
        }
      }
      _server.stop();
    }
  }

  _server.get_alog().write(APP, "All WebSocket connections closed");

  if (_serverThread) {
    _server.get_alog().write(APP, "Waiting for WebSocket server run loop to terminate");
    _serverThread->join();
    _serverThread.reset();
    _server.get_alog().write(APP, "WebSocket server run loop terminated");
  }

  std::unique_lock<std::shared_mutex> lock(_clientsMutex);
  _clients.clear();
}

template <typename ServerConfiguration>
inline void Server<ServerConfiguration>::start(const std::string& host, uint16_t port) {
  if (_serverThread) {
    throw std::runtime_error("Server already started");
  }

  websocketpp::lib::error_code ec;

  _server.listen(host, std::to_string(port), ec);
  if (ec) {
    throw std::runtime_error("Failed to listen on port " + std::to_string(port) + ": " +
                             ec.message());
  }

  _server.start_accept(ec);
  if (ec) {
    throw std::runtime_error("Failed to start accepting connections: " + ec.message());
  }

  _serverThread = std::make_unique<std::thread>([this]() {
    _server.get_alog().write(APP, "WebSocket server run loop started");
    _server.run();
    _server.get_alog().write(APP, "WebSocket server run loop stopped");
  });

  if (!_server.is_listening()) {
    throw std::runtime_error("WebSocket server failed to listen on port " + std::to_string(port));
  }

  websocketpp::lib::asio::error_code asioEc;
  auto endpoint = _server.get_local_endpoint(asioEc);
  if (asioEc) {
    throw std::runtime_error("Failed to resolve the local endpoint: " + ec.message());
  }

  const std::string protocol = _options.useTls ? "wss" : "ws";
  auto address = endpoint.address();
  _server.get_alog().write(APP, "WebSocket server listening at " + protocol + "://" +
                                  IPAddressToString(address) + ":" +
                                  std::to_string(endpoint.port()));
}

template <typename ServerConfiguration>
inline void Server<ServerConfiguration>::sendJson(ConnHandle hdl, json&& payload) {
  try {
    _server.send(hdl, std::move(payload).dump(), OpCode::TEXT);
  } catch (std::exception const& e) {
    _server.get_elog().write(RECOVERABLE, e.what());
  }
}

template <typename ServerConfiguration>
inline void Server<ServerConfiguration>::sendJsonRaw(ConnHandle hdl, const std::string& payload) {
  try {
    _server.send(hdl, payload, OpCode::TEXT);
  } catch (std::exception const& e) {
    _server.get_elog().write(RECOVERABLE, e.what());
  }
}

template <typename ServerConfiguration>
inline void Server<ServerConfiguration>::sendBinary(ConnHandle hdl, const uint8_t* payload,
                                                    size_t payloadSize) {
  try {
    _server.send(hdl, payload, payloadSize, OpCode::BINARY);
  } catch (std::exception const& e) {
    _server.get_elog().write(RECOVERABLE, e.what());
  }
}

template <typename ServerConfiguration>
inline void Server<ServerConfiguration>::sendStatusAndLogMsg(ConnHandle clientHandle,
                                                             const StatusLevel level,
                                                             const std::string& message) {
  const std::string endpoint = remoteEndpointString(clientHandle);
  const std::string logMessage = endpoint + ": " + message;
  const auto logLevel = StatusLevelToLogLevel(level);
  auto logger = level == StatusLevel::Info ? _server.get_alog() : _server.get_elog();
  logger.write(logLevel, logMessage);

  sendJson(clientHandle, json{
                           {"op", "status"},
                           {"level", static_cast<uint8_t>(level)},
                           {"message", message},
                         });
}

template <typename ServerConfiguration>
inline void Server<ServerConfiguration>::handleMessage(ConnHandle hdl, MessagePtr msg) {
  const OpCode op = msg->get_opcode();
  try {
    if (op == OpCode::TEXT) {
      handleTextMessage(hdl, msg);
    } else if (op == OpCode::BINARY) {
      handleBinaryMessage(hdl, msg);
    }
  } catch (const std::exception& e) {
    sendStatusAndLogMsg(hdl, StatusLevel::Error, e.what());
  } catch (...) {
    sendStatusAndLogMsg(hdl, StatusLevel::Error,
                        "Exception occurred when executing message handler");
  }
}

template <typename ServerConfiguration>
inline void Server<ServerConfiguration>::handleTextMessage(ConnHandle hdl, MessagePtr msg) {
  const json payload = json::parse(msg->get_payload());
  const std::string& op = payload.at("op").get<std::string>();

  const auto requiredCapabilityIt = CAPABILITY_BY_CLIENT_OPERATION.find(op);
  if (requiredCapabilityIt != CAPABILITY_BY_CLIENT_OPERATION.end() &&
      !hasCapability(requiredCapabilityIt->second)) {
    sendStatusAndLogMsg(hdl, StatusLevel::Error,
                        "Operation '" + op + "' not supported as server capability '" +
                          requiredCapabilityIt->second + "' is missing");
    return;
  }

  if (!hasHandler(StringHash(op))) {
    sendStatusAndLogMsg(
      hdl, StatusLevel::Error,
      "Operation '" + op + "' not supported as server handler function is missing");
    return;
  }

  try {
    switch (StringHash(op)) {
      case SUBSCRIBE:
        handleSubscribe(payload, hdl);
        break;
      case UNSUBSCRIBE:
        handleUnsubscribe(payload, hdl);
        break;
      case ADVERTISE:
        handleAdvertise(payload, hdl);
        break;
      case UNADVERTISE:
        handleUnadvertise(payload, hdl);
        break;
      case GET_PARAMETERS:
        handleGetParameters(payload, hdl);
        break;
      case SET_PARAMETERS:
        handleSetParameters(payload, hdl);
        break;
      case SUBSCRIBE_PARAMETER_UPDATES:
        handleSubscribeParameterUpdates(payload, hdl);
        break;
      case UNSUBSCRIBE_PARAMETER_UPDATES:
        handleUnsubscribeParameterUpdates(payload, hdl);
        break;
      case SUBSCRIBE_CONNECTION_GRAPH:
        handleSubscribeConnectionGraph(hdl);
        break;
      case UNSUBSCRIBE_CONNECTION_GRAPH:
        handleUnsubscribeConnectionGraph(hdl);
        break;
      case FETCH_ASSET:
        handleFetchAsset(payload, hdl);
        break;
      default:
        sendStatusAndLogMsg(hdl, StatusLevel::Error, "Unrecognized client opcode \"" + op + "\"");
        break;
    }
  } catch (const ExeptionWithId<uint32_t>& e) {
    const std::string postfix = " (op: " + op + ", id: " + std::to_string(e.id()) + ")";
    sendStatusAndLogMsg(hdl, StatusLevel::Error, e.what() + postfix);
  } catch (const std::exception& e) {
    const std::string postfix = " (op: " + op + ")";
    sendStatusAndLogMsg(hdl, StatusLevel::Error, e.what() + postfix);
  } catch (...) {
    const std::string postfix = " (op: " + op + ")";
    sendStatusAndLogMsg(hdl, StatusLevel::Error, "Failed to execute handler" + postfix);
  }
}  // namespace foxglove

template <typename ServerConfiguration>
inline void Server<ServerConfiguration>::handleBinaryMessage(ConnHandle hdl, MessagePtr msg) {
  const auto& payload = msg->get_payload();
  const uint8_t* data = reinterpret_cast<const uint8_t*>(payload.data());
  const size_t length = payload.size();

  if (length < 1) {
    sendStatusAndLogMsg(hdl, StatusLevel::Error, "Received an empty binary message");
    return;
  }

  const auto op = static_cast<ClientBinaryOpcode>(data[0]);

  const auto requiredCapabilityIt = CAPABILITY_BY_CLIENT_BINARY_OPERATION.find(op);
  if (requiredCapabilityIt != CAPABILITY_BY_CLIENT_BINARY_OPERATION.end() &&
      !hasCapability(requiredCapabilityIt->second)) {
    sendStatusAndLogMsg(hdl, StatusLevel::Error,
                        "Binary operation '" + std::to_string(static_cast<int>(op)) +
                          "' not supported as server capability '" + requiredCapabilityIt->second +
                          "' is missing");
    return;
  }

  switch (op) {
    case ClientBinaryOpcode::MESSAGE_DATA: {
      if (!_handlers.clientMessageHandler) {
        return;
      }

      if (length < 5) {
        sendStatusAndLogMsg(hdl, StatusLevel::Error,
                            "Invalid message length " + std::to_string(length));
        return;
      }
      const auto timestamp = std::chrono::duration_cast<std::chrono::nanoseconds>(
                               std::chrono::high_resolution_clock::now().time_since_epoch())
                               .count();
      const ClientChannelId channelId = *reinterpret_cast<const ClientChannelId*>(data + 1);
      std::shared_lock<std::shared_mutex> lock(_clientChannelsMutex);

      auto clientPublicationsIt = _clientChannels.find(hdl);
      if (clientPublicationsIt == _clientChannels.end()) {
        sendStatusAndLogMsg(hdl, StatusLevel::Error, "Client has no advertised channels");
        return;
      }

      auto& clientPublications = clientPublicationsIt->second;
      const auto& channelIt = clientPublications.find(channelId);
      if (channelIt == clientPublications.end()) {
        sendStatusAndLogMsg(hdl, StatusLevel::Error,
                            "Channel " + std::to_string(channelId) + " is not advertised");
        return;
      }

      try {
        const auto& advertisement = channelIt->second;
        const uint32_t sequence = 0;
        const ClientMessage clientMessage{static_cast<uint64_t>(timestamp),
                                          static_cast<uint64_t>(timestamp),
                                          sequence,
                                          advertisement,
                                          length,
                                          data};
        _handlers.clientMessageHandler(clientMessage, hdl);
      } catch (const ServiceError& e) {
        sendStatusAndLogMsg(hdl, StatusLevel::Error, e.what());
      } catch (...) {
        sendStatusAndLogMsg(hdl, StatusLevel::Error, "callService: Failed to execute handler");
      }
    } break;
    case ClientBinaryOpcode::SERVICE_CALL_REQUEST: {
      ServiceRequest request;
      if (length < request.size()) {
        sendStatusAndLogMsg(hdl, StatusLevel::Error,
                            "Invalid service call request length " + std::to_string(length));
        return;
      }

      request.read(data + 1, length - 1);

      {
        std::shared_lock<std::shared_mutex> lock(_servicesMutex);
        if (_services.find(request.serviceId) == _services.end()) {
          sendStatusAndLogMsg(
            hdl, StatusLevel::Error,
            "Service " + std::to_string(request.serviceId) + " is not advertised");
          return;
        }
      }

      if (_handlers.serviceRequestHandler) {
        _handlers.serviceRequestHandler(request, hdl);
      }
    } break;
    default: {
      sendStatusAndLogMsg(hdl, StatusLevel::Error,
                          "Unrecognized client opcode " + std::to_string(uint8_t(op)));
    } break;
  }
}

template <typename ServerConfiguration>
inline std::vector<ChannelId> Server<ServerConfiguration>::addChannels(
  const std::vector<ChannelWithoutId>& channels) {
  if (channels.empty()) {
    return {};
  }

  std::vector<ChannelId> channelIds;
  channelIds.reserve(channels.size());
  json::array_t channelsJson;

  {
    std::unique_lock<std::shared_mutex> lock(_channelsMutex);
    for (const auto& channelWithoutId : channels) {
      const auto newId = ++_nextChannelId;
      channelIds.push_back(newId);
      Channel newChannel{newId, channelWithoutId};
      channelsJson.push_back(newChannel);
      _channels.emplace(newId, std::move(newChannel));
    }
  }

  const auto msg = json{{"op", "advertise"}, {"channels", channelsJson}}.dump();
  std::shared_lock<std::shared_mutex> clientsLock(_clientsMutex);
  for (const auto& [hdl, clientInfo] : _clients) {
    (void)clientInfo;
    sendJsonRaw(hdl, msg);
  }

  return channelIds;
}

template <typename ServerConfiguration>
inline void Server<ServerConfiguration>::removeChannels(const std::vector<ChannelId>& channelIds) {
  if (channelIds.empty()) {
    return;
  }

  {
    std::unique_lock<std::shared_mutex> channelsLock(_channelsMutex);
    for (auto channelId : channelIds) {
      _channels.erase(channelId);
    }
  }

  const auto msg = json{{"op", "unadvertise"}, {"channelIds", channelIds}}.dump();

  std::unique_lock<std::shared_mutex> clientsLock(_clientsMutex);
  for (auto& [hdl, clientInfo] : _clients) {
    for (auto channelId : channelIds) {
      if (const auto it = clientInfo.subscriptionsByChannel.find(channelId);
          it != clientInfo.subscriptionsByChannel.end()) {
        clientInfo.subscriptionsByChannel.erase(it);
      }
    }
    sendJsonRaw(hdl, msg);
  }
}

template <typename ServerConfiguration>
inline void Server<ServerConfiguration>::publishParameterValues(
  ConnHandle hdl, const std::vector<Parameter>& parameters,
  const std::optional<std::string>& requestId) {
  // Filter out parameters which are not set.
  std::vector<Parameter> nonEmptyParameters;
  std::copy_if(parameters.begin(), parameters.end(), std::back_inserter(nonEmptyParameters),
               [](const auto& p) {
                 return p.getType() != ParameterType::PARAMETER_NOT_SET;
               });

  nlohmann::json jsonPayload{{"op", "parameterValues"}, {"parameters", nonEmptyParameters}};
  if (requestId) {
    jsonPayload["id"] = requestId.value();
  }
  sendJsonRaw(hdl, jsonPayload.dump());
}

template <typename ServerConfiguration>
inline void Server<ServerConfiguration>::updateParameterValues(
  const std::vector<Parameter>& parameters) {
  std::lock_guard<std::mutex> lock(_clientParamSubscriptionsMutex);
  for (const auto& clientParamSubscriptions : _clientParamSubscriptions) {
    std::vector<Parameter> paramsToSendToClient;

    // Only consider parameters that are subscribed by the client
    std::copy_if(parameters.begin(), parameters.end(), std::back_inserter(paramsToSendToClient),
                 [clientParamSubscriptions](const Parameter& param) {
                   return clientParamSubscriptions.second.find(param.getName()) !=
                          clientParamSubscriptions.second.end();
                 });

    if (!paramsToSendToClient.empty()) {
      publishParameterValues(clientParamSubscriptions.first, paramsToSendToClient);
    }
  }
}

template <typename ServerConfiguration>
inline std::vector<ServiceId> Server<ServerConfiguration>::addServices(
  const std::vector<ServiceWithoutId>& services) {
  if (services.empty()) {
    return {};
  }

  std::unique_lock<std::shared_mutex> lock(_servicesMutex);
  std::vector<ServiceId> serviceIds;
  json newServices;
  for (const auto& service : services) {
    const ServiceId serviceId = ++_nextServiceId;
    _services.emplace(serviceId, service);
    serviceIds.push_back(serviceId);
    newServices.push_back(Service(service, serviceId));
  }

  const auto msg = json{{"op", "advertiseServices"}, {"services", std::move(newServices)}}.dump();
  std::shared_lock<std::shared_mutex> clientsLock(_clientsMutex);
  for (const auto& [hdl, clientInfo] : _clients) {
    (void)clientInfo;
    sendJsonRaw(hdl, msg);
  }

  return serviceIds;
}

template <typename ServerConfiguration>
inline void Server<ServerConfiguration>::removeServices(const std::vector<ServiceId>& serviceIds) {
  std::unique_lock<std::shared_mutex> lock(_servicesMutex);
  std::vector<ServiceId> removedServices;
  for (const auto& serviceId : serviceIds) {
    if (const auto it = _services.find(serviceId); it != _services.end()) {
      _services.erase(it);
      removedServices.push_back(serviceId);
    }
  }

  if (!removedServices.empty()) {
    const auto msg =
      json{{"op", "unadvertiseServices"}, {"serviceIds", std::move(removedServices)}}.dump();
    std::shared_lock<std::shared_mutex> clientsLock(_clientsMutex);
    for (const auto& [hdl, clientInfo] : _clients) {
      (void)clientInfo;
      sendJsonRaw(hdl, msg);
    }
  }
}

template <typename ServerConfiguration>
inline void Server<ServerConfiguration>::sendMessage(ConnHandle clientHandle, ChannelId chanId,
                                                     uint64_t timestamp, const uint8_t* payload,
                                                     size_t payloadSize) {
  websocketpp::lib::error_code ec;
  const auto con = _server.get_con_from_hdl(clientHandle, ec);
  if (ec || !con) {
    return;
  }

  const auto bufferSizeinBytes = con->get_buffered_amount();
  if (bufferSizeinBytes + payloadSize >= _options.sendBufferLimitBytes) {
    const auto logFn = [this, clientHandle]() {
      sendStatusAndLogMsg(clientHandle, StatusLevel::Warning, "Send buffer limit reached");
    };
    FOXGLOVE_DEBOUNCE(logFn, 2500);
    return;
  }

  SubscriptionId subId = std::numeric_limits<SubscriptionId>::max();

  {
    std::shared_lock<std::shared_mutex> lock(_clientsMutex);
    const auto clientHandleAndInfoIt = _clients.find(clientHandle);
    if (clientHandleAndInfoIt == _clients.end()) {
      return;  // Client got removed in the meantime.
    }

    const auto& client = clientHandleAndInfoIt->second;
    const auto& subs = client.subscriptionsByChannel.find(chanId);
    if (subs == client.subscriptionsByChannel.end()) {
      return;  // Client not subscribed to this channel.
    }
    subId = subs->second;
  }

  std::array<uint8_t, 1 + 4 + 8> msgHeader;
  msgHeader[0] = uint8_t(BinaryOpcode::MESSAGE_DATA);
  foxglove::WriteUint32LE(msgHeader.data() + 1, subId);
  foxglove::WriteUint64LE(msgHeader.data() + 5, timestamp);

  const size_t messageSize = msgHeader.size() + payloadSize;
  auto message = con->get_message(OpCode::BINARY, messageSize);
  message->set_compressed(_options.useCompression);

  message->set_payload(msgHeader.data(), msgHeader.size());
  message->append_payload(payload, payloadSize);
  con->send(message);
}

template <typename ServerConfiguration>
inline void Server<ServerConfiguration>::broadcastTime(uint64_t timestamp) {
  std::array<uint8_t, 1 + 8> message;
  message[0] = uint8_t(BinaryOpcode::TIME_DATA);
  foxglove::WriteUint64LE(message.data() + 1, timestamp);

  std::shared_lock<std::shared_mutex> lock(_clientsMutex);
  for (const auto& [hdl, clientInfo] : _clients) {
    (void)clientInfo;
    sendBinary(hdl, message.data(), message.size());
  }
}

template <typename ServerConfiguration>
inline void Server<ServerConfiguration>::sendServiceResponse(ConnHandle clientHandle,
                                                             const ServiceResponse& response) {
  std::vector<uint8_t> payload(1 + response.size());
  payload[0] = uint8_t(BinaryOpcode::SERVICE_CALL_RESPONSE);
  response.write(payload.data() + 1);
  sendBinary(clientHandle, payload.data(), payload.size());
}

template <typename ServerConfiguration>
inline uint16_t Server<ServerConfiguration>::getPort() {
  websocketpp::lib::asio::error_code ec;
  auto endpoint = _server.get_local_endpoint(ec);
  if (ec) {
    throw std::runtime_error("Server not listening on any port. Has it been started before?");
  }
  return endpoint.port();
}

template <typename ServerConfiguration>
inline void Server<ServerConfiguration>::updateConnectionGraph(
  const MapOfSets& publishedTopics, const MapOfSets& subscribedTopics,
  const MapOfSets& advertisedServices) {
  json::array_t publisherDiff, subscriberDiff, servicesDiff;
  std::unordered_set<std::string> topicNames, serviceNames;
  std::unordered_set<std::string> knownTopicNames, knownServiceNames;
  {
    std::unique_lock<std::shared_mutex> lock(_connectionGraphMutex);
    for (const auto& [name, publisherIds] : publishedTopics) {
      const auto it = _connectionGraph.publishedTopics.find(name);
      if (it == _connectionGraph.publishedTopics.end() ||
          _connectionGraph.publishedTopics[name] != publisherIds) {
        publisherDiff.push_back(nlohmann::json{{"name", name}, {"publisherIds", publisherIds}});
      }
      topicNames.insert(name);
    }
    for (const auto& [name, subscriberIds] : subscribedTopics) {
      const auto it = _connectionGraph.subscribedTopics.find(name);
      if (it == _connectionGraph.subscribedTopics.end() ||
          _connectionGraph.subscribedTopics[name] != subscriberIds) {
        subscriberDiff.push_back(nlohmann::json{{"name", name}, {"subscriberIds", subscriberIds}});
      }
      topicNames.insert(name);
    }
    for (const auto& [name, providerIds] : advertisedServices) {
      const auto it = _connectionGraph.advertisedServices.find(name);
      if (it == _connectionGraph.advertisedServices.end() ||
          _connectionGraph.advertisedServices[name] != providerIds) {
        servicesDiff.push_back(nlohmann::json{{"name", name}, {"providerIds", providerIds}});
      }
      serviceNames.insert(name);
    }

    for (const auto& nameWithIds : _connectionGraph.publishedTopics) {
      knownTopicNames.insert(nameWithIds.first);
    }
    for (const auto& nameWithIds : _connectionGraph.subscribedTopics) {
      knownTopicNames.insert(nameWithIds.first);
    }
    for (const auto& nameWithIds : _connectionGraph.advertisedServices) {
      knownServiceNames.insert(nameWithIds.first);
    }

    _connectionGraph.publishedTopics = publishedTopics;
    _connectionGraph.subscribedTopics = subscribedTopics;
    _connectionGraph.advertisedServices = advertisedServices;
  }

  std::vector<std::string> removedTopics, removedServices;
  std::copy_if(knownTopicNames.begin(), knownTopicNames.end(), std::back_inserter(removedTopics),
               [&topicNames](const std::string& topic) {
                 return topicNames.find(topic) == topicNames.end();
               });
  std::copy_if(knownServiceNames.begin(), knownServiceNames.end(),
               std::back_inserter(removedServices), [&serviceNames](const std::string& service) {
                 return serviceNames.find(service) == serviceNames.end();
               });

  if (publisherDiff.empty() && subscriberDiff.empty() && servicesDiff.empty() &&
      removedTopics.empty() && removedServices.empty()) {
    return;
  }

  const json msg = {
    {"op", "connectionGraphUpdate"},      {"publishedTopics", publisherDiff},
    {"subscribedTopics", subscriberDiff}, {"advertisedServices", servicesDiff},
    {"removedTopics", removedTopics},     {"removedServices", removedServices},
  };
  const auto payload = msg.dump();

  std::shared_lock<std::shared_mutex> clientsLock(_clientsMutex);
  for (const auto& [hdl, clientInfo] : _clients) {
    if (clientInfo.subscribedToConnectionGraph) {
      _server.send(hdl, payload, OpCode::TEXT);
    }
  }
}

template <typename ServerConfiguration>
inline std::string Server<ServerConfiguration>::remoteEndpointString(ConnHandle clientHandle) {
  websocketpp::lib::error_code ec;
  const auto con = _server.get_con_from_hdl(clientHandle, ec);
  return con ? con->get_remote_endpoint() : "(unknown)";
}

template <typename ServerConfiguration>
inline bool Server<ServerConfiguration>::isParameterSubscribed(const std::string& paramName) const {
  return std::find_if(_clientParamSubscriptions.begin(), _clientParamSubscriptions.end(),
                      [paramName](const auto& paramSubscriptions) {
                        return paramSubscriptions.second.find(paramName) !=
                               paramSubscriptions.second.end();
                      }) != _clientParamSubscriptions.end();
}

template <typename ServerConfiguration>
inline void Server<ServerConfiguration>::unsubscribeParamsWithoutSubscriptions(
  ConnHandle hdl, const std::unordered_set<std::string>& paramNames) {
  std::vector<std::string> paramsToUnsubscribe;
  {
    std::lock_guard<std::mutex> lock(_clientParamSubscriptionsMutex);
    std::copy_if(paramNames.begin(), paramNames.end(), std::back_inserter(paramsToUnsubscribe),
                 [this](const std::string& paramName) {
                   return !isParameterSubscribed(paramName);
                 });
  }

  if (_handlers.parameterSubscriptionHandler && !paramsToUnsubscribe.empty()) {
    for (const auto& param : paramsToUnsubscribe) {
      _server.get_alog().write(APP, "Unsubscribing from parameter '" + param + "'.");
    }

    try {
      _handlers.parameterSubscriptionHandler(paramsToUnsubscribe,
                                             ParameterSubscriptionOperation::UNSUBSCRIBE, hdl);
    } catch (const std::exception& e) {
      sendStatusAndLogMsg(hdl, StatusLevel::Error, e.what());
    } catch (...) {
      sendStatusAndLogMsg(hdl, StatusLevel::Error,
                          "Failed to unsubscribe from one more more parameters");
    }
  }
}

template <typename ServerConfiguration>
inline bool Server<ServerConfiguration>::hasCapability(const std::string& capability) const {
  return std::find(_options.capabilities.begin(), _options.capabilities.end(), capability) !=
         _options.capabilities.end();
}

template <typename ServerConfiguration>
inline bool Server<ServerConfiguration>::hasHandler(uint32_t op) const {
  switch (op) {
    case SUBSCRIBE:
      return bool(_handlers.subscribeHandler);
    case UNSUBSCRIBE:
      return bool(_handlers.unsubscribeHandler);
    case ADVERTISE:
      return bool(_handlers.clientAdvertiseHandler);
    case UNADVERTISE:
      return bool(_handlers.clientUnadvertiseHandler);
    case GET_PARAMETERS:
      return bool(_handlers.parameterRequestHandler);
    case SET_PARAMETERS:
      return bool(_handlers.parameterChangeHandler);
    case SUBSCRIBE_PARAMETER_UPDATES:
    case UNSUBSCRIBE_PARAMETER_UPDATES:
      return bool(_handlers.parameterSubscriptionHandler);
    case SUBSCRIBE_CONNECTION_GRAPH:
    case UNSUBSCRIBE_CONNECTION_GRAPH:
      return bool(_handlers.subscribeConnectionGraphHandler);
    case FETCH_ASSET:
      return bool(_handlers.fetchAssetHandler);
    default:
      throw std::runtime_error("Unknown operation: " + std::to_string(op));
  }
}

template <typename ServerConfiguration>
void Server<ServerConfiguration>::handleSubscribe(const nlohmann::json& payload, ConnHandle hdl) {
  std::unordered_map<ChannelId, SubscriptionId> clientSubscriptionsByChannel;
  {
    std::shared_lock<std::shared_mutex> clientsLock(_clientsMutex);
    clientSubscriptionsByChannel = _clients.at(hdl).subscriptionsByChannel;
  }

  const auto findSubscriptionBySubId =
    [](const std::unordered_map<ChannelId, SubscriptionId>& subscriptionsByChannel,
       SubscriptionId subId) {
      return std::find_if(subscriptionsByChannel.begin(), subscriptionsByChannel.end(),
                          [&subId](const auto& mo) {
                            return mo.second == subId;
                          });
    };

  for (const auto& sub : payload.at("subscriptions")) {
    SubscriptionId subId = sub.at("id");
    ChannelId channelId = sub.at("channelId");
    if (findSubscriptionBySubId(clientSubscriptionsByChannel, subId) !=
        clientSubscriptionsByChannel.end()) {
      sendStatusAndLogMsg(hdl, StatusLevel::Error,
                          "Client subscription id " + std::to_string(subId) +
                            " was already used; ignoring subscription");
      continue;
    }
    const auto& channelIt = _channels.find(channelId);
    if (channelIt == _channels.end()) {
      sendStatusAndLogMsg(
        hdl, StatusLevel::Warning,
        "Channel " + std::to_string(channelId) + " is not available; ignoring subscription");
      continue;
    }

    _handlers.subscribeHandler(channelId, hdl);
    std::unique_lock<std::shared_mutex> clientsLock(_clientsMutex);
    _clients.at(hdl).subscriptionsByChannel.emplace(channelId, subId);
  }
}

template <typename ServerConfiguration>
void Server<ServerConfiguration>::handleUnsubscribe(const nlohmann::json& payload, ConnHandle hdl) {
  std::unordered_map<ChannelId, SubscriptionId> clientSubscriptionsByChannel;
  {
    std::shared_lock<std::shared_mutex> clientsLock(_clientsMutex);
    clientSubscriptionsByChannel = _clients.at(hdl).subscriptionsByChannel;
  }

  const auto findSubscriptionBySubId =
    [](const std::unordered_map<ChannelId, SubscriptionId>& subscriptionsByChannel,
       SubscriptionId subId) {
      return std::find_if(subscriptionsByChannel.begin(), subscriptionsByChannel.end(),
                          [&subId](const auto& mo) {
                            return mo.second == subId;
                          });
    };

  for (const auto& subIdJson : payload.at("subscriptionIds")) {
    SubscriptionId subId = subIdJson;
    const auto& sub = findSubscriptionBySubId(clientSubscriptionsByChannel, subId);
    if (sub == clientSubscriptionsByChannel.end()) {
      sendStatusAndLogMsg(hdl, StatusLevel::Warning,
                          "Client subscription id " + std::to_string(subId) +
                            " did not exist; ignoring unsubscription");
      continue;
    }

    ChannelId chanId = sub->first;
    _handlers.unsubscribeHandler(chanId, hdl);
    std::unique_lock<std::shared_mutex> clientsLock(_clientsMutex);
    _clients.at(hdl).subscriptionsByChannel.erase(chanId);
  }
}

template <typename ServerConfiguration>
void Server<ServerConfiguration>::handleAdvertise(const nlohmann::json& payload, ConnHandle hdl) {
  std::unique_lock<std::shared_mutex> clientChannelsLock(_clientChannelsMutex);
  auto [clientPublicationsIt, isFirstPublication] =
    _clientChannels.emplace(hdl, std::unordered_map<ClientChannelId, ClientAdvertisement>());

  auto& clientPublications = clientPublicationsIt->second;

  for (const auto& chan : payload.at("channels")) {
    ClientChannelId channelId = chan.at("id");
    if (!isFirstPublication && clientPublications.find(channelId) != clientPublications.end()) {
      sendStatusAndLogMsg(hdl, StatusLevel::Error,
                          "Channel " + std::to_string(channelId) + " was already advertised");
      continue;
    }

    const auto topic = chan.at("topic").get<std::string>();
    if (!isWhitelisted(topic, _options.clientTopicWhitelistPatterns)) {
      sendStatusAndLogMsg(hdl, StatusLevel::Error,
                          "Can't advertise channel " + std::to_string(channelId) + ", topic '" +
                            topic + "' not whitelisted");
      continue;
    }
    ClientAdvertisement advertisement{};
    advertisement.channelId = channelId;
    advertisement.topic = topic;
    advertisement.encoding = chan.at("encoding").get<std::string>();
    advertisement.schemaName = chan.at("schemaName").get<std::string>();

    _handlers.clientAdvertiseHandler(advertisement, hdl);
    std::unique_lock<std::shared_mutex> clientsLock(_clientsMutex);
    _clients.at(hdl).advertisedChannels.emplace(channelId);
    clientPublications.emplace(channelId, advertisement);
  }
}

template <typename ServerConfiguration>
void Server<ServerConfiguration>::handleUnadvertise(const nlohmann::json& payload, ConnHandle hdl) {
  std::unique_lock<std::shared_mutex> clientChannelsLock(_clientChannelsMutex);
  auto clientPublicationsIt = _clientChannels.find(hdl);
  if (clientPublicationsIt == _clientChannels.end()) {
    sendStatusAndLogMsg(hdl, StatusLevel::Error, "Client has no advertised channels");
    return;
  }

  auto& clientPublications = clientPublicationsIt->second;

  for (const auto& chanIdJson : payload.at("channelIds")) {
    ClientChannelId channelId = chanIdJson.get<ClientChannelId>();
    const auto& channelIt = clientPublications.find(channelId);
    if (channelIt == clientPublications.end()) {
      continue;
    }

    _handlers.clientUnadvertiseHandler(channelId, hdl);
    std::unique_lock<std::shared_mutex> clientsLock(_clientsMutex);
    auto& clientInfo = _clients.at(hdl);
    clientPublications.erase(channelIt);
    const auto advertisedChannelIt = clientInfo.advertisedChannels.find(channelId);
    if (advertisedChannelIt != clientInfo.advertisedChannels.end()) {
      clientInfo.advertisedChannels.erase(advertisedChannelIt);
    }
  }
}

template <typename ServerConfiguration>
void Server<ServerConfiguration>::handleGetParameters(const nlohmann::json& payload,
                                                      ConnHandle hdl) {
  const auto paramNames = payload.at("parameterNames").get<std::vector<std::string>>();
  const auto requestId = payload.find("id") == payload.end()
                           ? std::nullopt
                           : std::optional<std::string>(payload["id"].get<std::string>());
  _handlers.parameterRequestHandler(paramNames, requestId, hdl);
}

template <typename ServerConfiguration>
void Server<ServerConfiguration>::handleSetParameters(const nlohmann::json& payload,
                                                      ConnHandle hdl) {
  const auto parameters = payload.at("parameters").get<std::vector<Parameter>>();
  const auto requestId = payload.find("id") == payload.end()
                           ? std::nullopt
                           : std::optional<std::string>(payload["id"].get<std::string>());
  _handlers.parameterChangeHandler(parameters, requestId, hdl);
}

template <typename ServerConfiguration>
void Server<ServerConfiguration>::handleSubscribeParameterUpdates(const nlohmann::json& payload,
                                                                  ConnHandle hdl) {
  const auto paramNames = payload.at("parameterNames").get<std::unordered_set<std::string>>();
  std::vector<std::string> paramsToSubscribe;
  {
    // Only consider parameters that are not subscribed yet (by this or by other clients)
    std::lock_guard<std::mutex> lock(_clientParamSubscriptionsMutex);
    std::copy_if(paramNames.begin(), paramNames.end(), std::back_inserter(paramsToSubscribe),
                 [this](const std::string& paramName) {
                   return !isParameterSubscribed(paramName);
                 });

    // Update the client's parameter subscriptions.
    auto& clientSubscribedParams = _clientParamSubscriptions[hdl];
    clientSubscribedParams.insert(paramNames.begin(), paramNames.end());
  }

  if (!paramsToSubscribe.empty()) {
    _handlers.parameterSubscriptionHandler(paramsToSubscribe,
                                           ParameterSubscriptionOperation::SUBSCRIBE, hdl);
  }
}

template <typename ServerConfiguration>
void Server<ServerConfiguration>::handleUnsubscribeParameterUpdates(const nlohmann::json& payload,
                                                                    ConnHandle hdl) {
  const auto paramNames = payload.at("parameterNames").get<std::unordered_set<std::string>>();
  {
    std::lock_guard<std::mutex> lock(_clientParamSubscriptionsMutex);
    auto& clientSubscribedParams = _clientParamSubscriptions[hdl];
    for (const auto& paramName : paramNames) {
      clientSubscribedParams.erase(paramName);
    }
  }

  unsubscribeParamsWithoutSubscriptions(hdl, paramNames);
}

template <typename ServerConfiguration>
void Server<ServerConfiguration>::handleSubscribeConnectionGraph(ConnHandle hdl) {
  bool subscribeToConnnectionGraph = false;
  {
    std::unique_lock<std::shared_mutex> lock(_connectionGraphMutex);
    _connectionGraph.subscriptionCount++;
    subscribeToConnnectionGraph = _connectionGraph.subscriptionCount == 1;
  }

  if (subscribeToConnnectionGraph) {
    // First subscriber, let the handler know that we are interested in updates.
    _server.get_alog().write(APP, "Subscribing to connection graph updates.");
    _handlers.subscribeConnectionGraphHandler(true);
    std::unique_lock<std::shared_mutex> clientsLock(_clientsMutex);
    _clients.at(hdl).subscribedToConnectionGraph = true;
  }

  json::array_t publishedTopicsJson, subscribedTopicsJson, advertisedServicesJson;
  {
    std::shared_lock<std::shared_mutex> lock(_connectionGraphMutex);
    for (const auto& [name, ids] : _connectionGraph.publishedTopics) {
      publishedTopicsJson.push_back(nlohmann::json{{"name", name}, {"publisherIds", ids}});
    }
    for (const auto& [name, ids] : _connectionGraph.subscribedTopics) {
      subscribedTopicsJson.push_back(nlohmann::json{{"name", name}, {"subscriberIds", ids}});
    }
    for (const auto& [name, ids] : _connectionGraph.advertisedServices) {
      advertisedServicesJson.push_back(nlohmann::json{{"name", name}, {"providerIds", ids}});
    }
  }

  const json jsonMsg = {
    {"op", "connectionGraphUpdate"},
    {"publishedTopics", publishedTopicsJson},
    {"subscribedTopics", subscribedTopicsJson},
    {"advertisedServices", advertisedServicesJson},
    {"removedTopics", json::array()},
    {"removedServices", json::array()},
  };

  sendJsonRaw(hdl, jsonMsg.dump());
}

template <typename ServerConfiguration>
void Server<ServerConfiguration>::handleUnsubscribeConnectionGraph(ConnHandle hdl) {
  bool clientWasSubscribed = false;
  {
    std::unique_lock<std::shared_mutex> clientsLock(_clientsMutex);
    auto& clientInfo = _clients.at(hdl);
    if (clientInfo.subscribedToConnectionGraph) {
      clientWasSubscribed = true;
      clientInfo.subscribedToConnectionGraph = false;
    }
  }

  if (clientWasSubscribed) {
    bool unsubscribeFromConnnectionGraph = false;
    {
      std::unique_lock<std::shared_mutex> lock(_connectionGraphMutex);
      _connectionGraph.subscriptionCount--;
      unsubscribeFromConnnectionGraph = _connectionGraph.subscriptionCount == 0;
    }
    if (unsubscribeFromConnnectionGraph) {
      _server.get_alog().write(APP, "Unsubscribing from connection graph updates.");
      _handlers.subscribeConnectionGraphHandler(false);
    }
  } else {
    sendStatusAndLogMsg(hdl, StatusLevel::Error,
                        "Client was not subscribed to connection graph updates");
  }
}

template <typename ServerConfiguration>
void Server<ServerConfiguration>::handleFetchAsset(const nlohmann::json& payload, ConnHandle hdl) {
  const auto uri = payload.at("uri").get<std::string>();
  const auto requestId = payload.at("requestId").get<uint32_t>();
  _handlers.fetchAssetHandler(uri, requestId, hdl);
}

template <typename ServerConfiguration>
inline void Server<ServerConfiguration>::sendFetchAssetResponse(
  ConnHandle clientHandle, const FetchAssetResponse& response) {
  websocketpp::lib::error_code ec;
  const auto con = _server.get_con_from_hdl(clientHandle, ec);
  if (ec || !con) {
    return;
  }

  const size_t errMsgSize =
    response.status == FetchAssetStatus::Error ? response.errorMessage.size() : 0ul;
  const size_t dataSize = response.status == FetchAssetStatus::Success ? response.data.size() : 0ul;
  const size_t messageSize = 1 + 4 + 1 + 4 + errMsgSize + dataSize;

  auto message = con->get_message(OpCode::BINARY, messageSize);

  const auto op = BinaryOpcode::FETCH_ASSET_RESPONSE;
  message->append_payload(&op, 1);

  std::array<uint8_t, 4> uint32Data;
  foxglove::WriteUint32LE(uint32Data.data(), response.requestId);
  message->append_payload(uint32Data.data(), uint32Data.size());

  const uint8_t status = static_cast<uint8_t>(response.status);
  message->append_payload(&status, 1);

  foxglove::WriteUint32LE(uint32Data.data(), response.errorMessage.size());
  message->append_payload(uint32Data.data(), uint32Data.size());
  message->append_payload(response.errorMessage.data(), errMsgSize);

  message->append_payload(response.data.data(), dataSize);
  con->send(message);
}

}  // namespace foxglove
