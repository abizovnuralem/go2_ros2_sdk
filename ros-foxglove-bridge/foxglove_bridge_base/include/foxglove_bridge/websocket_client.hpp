#pragma once

#include <functional>
#include <future>
#include <optional>
#include <shared_mutex>
#include <utility>
#include <vector>

#include <nlohmann/json.hpp>
#include <websocketpp/client.hpp>
#include <websocketpp/common/memory.hpp>
#include <websocketpp/common/thread.hpp>

#include "common.hpp"
#include "parameter.hpp"
#include "serialization.hpp"

namespace foxglove {

inline void to_json(nlohmann::json& j, const ClientAdvertisement& p) {
  j = nlohmann::json{{"id", p.channelId},
                     {"topic", p.topic},
                     {"encoding", p.encoding},
                     {"schemaName", p.schemaName}};
}

using TextMessageHandler = std::function<void(const std::string&)>;
using BinaryMessageHandler = std::function<void(const uint8_t*, size_t)>;
using OpCode = websocketpp::frame::opcode::value;

class ClientInterface {
public:
  virtual void connect(
    const std::string& uri, std::function<void(websocketpp::connection_hdl)> onOpenHandler,
    std::function<void(websocketpp::connection_hdl)> onCloseHandler = nullptr) = 0;
  virtual std::future<void> connect(const std::string& uri) = 0;
  virtual void close() = 0;

  virtual void subscribe(
    const std::vector<std::pair<SubscriptionId, ChannelId>>& subscriptions) = 0;
  virtual void unsubscribe(const std::vector<SubscriptionId>& subscriptionIds) = 0;
  virtual void advertise(const std::vector<ClientAdvertisement>& channels) = 0;
  virtual void unadvertise(const std::vector<ClientChannelId>& channelIds) = 0;
  virtual void publish(ClientChannelId channelId, const uint8_t* buffer, size_t size) = 0;
  virtual void sendServiceRequest(const ServiceRequest& request) = 0;
  virtual void getParameters(const std::vector<std::string>& parameterNames,
                             const std::optional<std::string>& requestId) = 0;
  virtual void setParameters(const std::vector<Parameter>& parameters,
                             const std::optional<std::string>& requestId) = 0;
  virtual void subscribeParameterUpdates(const std::vector<std::string>& parameterNames) = 0;
  virtual void unsubscribeParameterUpdates(const std::vector<std::string>& parameterNames) = 0;
  virtual void fetchAsset(const std::string& name, uint32_t requestId) = 0;

  virtual void setTextMessageHandler(TextMessageHandler handler) = 0;
  virtual void setBinaryMessageHandler(BinaryMessageHandler handler) = 0;
};

template <typename ClientConfiguration>
class Client : public ClientInterface {
public:
  using ClientType = websocketpp::client<ClientConfiguration>;
  using MessagePtr = typename ClientType::message_ptr;
  using ConnectionPtr = typename ClientType::connection_ptr;

  Client() {
    _endpoint.clear_access_channels(websocketpp::log::alevel::all);
    _endpoint.clear_error_channels(websocketpp::log::elevel::all);

    _endpoint.init_asio();
    _endpoint.start_perpetual();

    _endpoint.set_message_handler(
      bind(&Client::messageHandler, this, std::placeholders::_1, std::placeholders::_2));

    _thread.reset(new websocketpp::lib::thread(&ClientType::run, &_endpoint));
  }

  virtual ~Client() {
    close();
    _endpoint.stop_perpetual();
    _thread->join();
  }

  void connect(const std::string& uri,
               std::function<void(websocketpp::connection_hdl)> onOpenHandler,
               std::function<void(websocketpp::connection_hdl)> onCloseHandler = nullptr) override {
    std::unique_lock<std::shared_mutex> lock(_mutex);

    websocketpp::lib::error_code ec;
    _con = _endpoint.get_connection(uri, ec);

    if (ec) {
      throw std::runtime_error("Failed to get connection from URI " + uri);
    }

    if (onOpenHandler) {
      _con->set_open_handler(onOpenHandler);
    }
    if (onCloseHandler) {
      _con->set_close_handler(onCloseHandler);
    }

    _con->add_subprotocol(SUPPORTED_SUBPROTOCOL);
    _endpoint.connect(_con);
  }

  std::future<void> connect(const std::string& uri) override {
    auto promise = std::make_shared<std::promise<void>>();
    auto future = promise->get_future();

    connect(uri, [p = std::move(promise)](websocketpp::connection_hdl) mutable {
      p->set_value();
    });

    return future;
  }

  void close() override {
    std::unique_lock<std::shared_mutex> lock(_mutex);
    if (!_con) {
      return;  // Already disconnected
    }

    _endpoint.close(_con, websocketpp::close::status::going_away, "");
    _con.reset();
  }

  void messageHandler(websocketpp::connection_hdl hdl, MessagePtr msg) {
    (void)hdl;
    const OpCode op = msg->get_opcode();

    switch (op) {
      case OpCode::TEXT: {
        std::shared_lock<std::shared_mutex> lock(_mutex);
        if (_textMessageHandler) {
          _textMessageHandler(msg->get_payload());
        }
      } break;
      case OpCode::BINARY: {
        std::shared_lock<std::shared_mutex> lock(_mutex);
        const auto& payload = msg->get_payload();
        if (_binaryMessageHandler) {
          _binaryMessageHandler(reinterpret_cast<const uint8_t*>(payload.data()), payload.size());
        }
      } break;
      default:
        break;
    }
  }

  void subscribe(const std::vector<std::pair<SubscriptionId, ChannelId>>& subscriptions) override {
    nlohmann::json subscriptionsJson;
    for (const auto& [subId, channelId] : subscriptions) {
      subscriptionsJson.push_back({{"id", subId}, {"channelId", channelId}});
    }

    const std::string payload =
      nlohmann::json{{"op", "subscribe"}, {"subscriptions", std::move(subscriptionsJson)}}.dump();
    sendText(payload);
  }

  void unsubscribe(const std::vector<SubscriptionId>& subscriptionIds) override {
    const std::string payload =
      nlohmann::json{{"op", "unsubscribe"}, {"subscriptionIds", subscriptionIds}}.dump();
    sendText(payload);
  }

  void advertise(const std::vector<ClientAdvertisement>& channels) override {
    const std::string payload = nlohmann::json{{"op", "advertise"}, {"channels", channels}}.dump();
    sendText(payload);
  }

  void unadvertise(const std::vector<ClientChannelId>& channelIds) override {
    const std::string payload =
      nlohmann::json{{"op", "unadvertise"}, {"channelIds", channelIds}}.dump();
    sendText(payload);
  }

  void publish(ClientChannelId channelId, const uint8_t* buffer, size_t size) override {
    std::vector<uint8_t> payload(1 + 4 + size);
    payload[0] = uint8_t(ClientBinaryOpcode::MESSAGE_DATA);
    foxglove::WriteUint32LE(payload.data() + 1, channelId);
    std::memcpy(payload.data() + 1 + 4, buffer, size);
    sendBinary(payload.data(), payload.size());
  }

  void sendServiceRequest(const ServiceRequest& request) override {
    std::vector<uint8_t> payload(1 + request.size());
    payload[0] = uint8_t(ClientBinaryOpcode::SERVICE_CALL_REQUEST);
    request.write(payload.data() + 1);
    sendBinary(payload.data(), payload.size());
  }

  void getParameters(const std::vector<std::string>& parameterNames,
                     const std::optional<std::string>& requestId = std::nullopt) override {
    nlohmann::json jsonPayload{{"op", "getParameters"}, {"parameterNames", parameterNames}};
    if (requestId) {
      jsonPayload["id"] = requestId.value();
    }
    sendText(jsonPayload.dump());
  }

  void setParameters(const std::vector<Parameter>& parameters,
                     const std::optional<std::string>& requestId = std::nullopt) override {
    nlohmann::json jsonPayload{{"op", "setParameters"}, {"parameters", parameters}};
    if (requestId) {
      jsonPayload["id"] = requestId.value();
    }
    sendText(jsonPayload.dump());
  }

  void subscribeParameterUpdates(const std::vector<std::string>& parameterNames) override {
    nlohmann::json jsonPayload{{"op", "subscribeParameterUpdates"},
                               {"parameterNames", parameterNames}};
    sendText(jsonPayload.dump());
  }

  void unsubscribeParameterUpdates(const std::vector<std::string>& parameterNames) override {
    nlohmann::json jsonPayload{{"op", "unsubscribeParameterUpdates"},
                               {"parameterNames", parameterNames}};
    sendText(jsonPayload.dump());
  }

  void fetchAsset(const std::string& uri, uint32_t requestId) override {
    nlohmann::json jsonPayload{{"op", "fetchAsset"}, {"uri", uri}, {"requestId", requestId}};
    sendText(jsonPayload.dump());
  }

  void setTextMessageHandler(TextMessageHandler handler) override {
    std::unique_lock<std::shared_mutex> lock(_mutex);
    _textMessageHandler = std::move(handler);
  }

  void setBinaryMessageHandler(BinaryMessageHandler handler) override {
    std::unique_lock<std::shared_mutex> lock(_mutex);
    _binaryMessageHandler = std::move(handler);
  }

  void sendText(const std::string& payload) {
    std::shared_lock<std::shared_mutex> lock(_mutex);
    _endpoint.send(_con, payload, OpCode::TEXT);
  }

  void sendBinary(const uint8_t* data, size_t dataLength) {
    std::shared_lock<std::shared_mutex> lock(_mutex);
    _endpoint.send(_con, data, dataLength, OpCode::BINARY);
  }

protected:
  ClientType _endpoint;
  websocketpp::lib::shared_ptr<websocketpp::lib::thread> _thread;
  ConnectionPtr _con;
  std::shared_mutex _mutex;
  TextMessageHandler _textMessageHandler;
  BinaryMessageHandler _binaryMessageHandler;
};

}  // namespace foxglove
