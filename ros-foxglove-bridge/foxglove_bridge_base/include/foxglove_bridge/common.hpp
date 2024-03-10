#pragma once

#include <array>
#include <cstring>
#include <optional>
#include <stdint.h>
#include <string>
#include <vector>

namespace foxglove {

constexpr char SUPPORTED_SUBPROTOCOL[] = "foxglove.websocket.v1";
constexpr char CAPABILITY_CLIENT_PUBLISH[] = "clientPublish";
constexpr char CAPABILITY_TIME[] = "time";
constexpr char CAPABILITY_PARAMETERS[] = "parameters";
constexpr char CAPABILITY_PARAMETERS_SUBSCRIBE[] = "parametersSubscribe";
constexpr char CAPABILITY_SERVICES[] = "services";
constexpr char CAPABILITY_CONNECTION_GRAPH[] = "connectionGraph";
constexpr char CAPABILITY_ASSETS[] = "assets";

constexpr std::array<const char*, 6> DEFAULT_CAPABILITIES = {
  CAPABILITY_CLIENT_PUBLISH, CAPABILITY_CONNECTION_GRAPH, CAPABILITY_PARAMETERS_SUBSCRIBE,
  CAPABILITY_PARAMETERS,     CAPABILITY_SERVICES,         CAPABILITY_ASSETS,
};

using ChannelId = uint32_t;
using ClientChannelId = uint32_t;
using SubscriptionId = uint32_t;
using ServiceId = uint32_t;

enum class BinaryOpcode : uint8_t {
  MESSAGE_DATA = 1,
  TIME_DATA = 2,
  SERVICE_CALL_RESPONSE = 3,
  FETCH_ASSET_RESPONSE = 4,
};

enum class ClientBinaryOpcode : uint8_t {
  MESSAGE_DATA = 1,
  SERVICE_CALL_REQUEST = 2,
};

enum class WebSocketLogLevel {
  Debug,
  Info,
  Warn,
  Error,
  Critical,
};

struct ChannelWithoutId {
  std::string topic;
  std::string encoding;
  std::string schemaName;
  std::string schema;
  std::optional<std::string> schemaEncoding;

  bool operator==(const ChannelWithoutId& other) const {
    return topic == other.topic && encoding == other.encoding && schemaName == other.schemaName &&
           schema == other.schema && schemaEncoding == other.schemaEncoding;
  }
};

struct Channel : ChannelWithoutId {
  ChannelId id;

  Channel() = default;  // requirement for json conversions.
  Channel(ChannelId id, ChannelWithoutId ch)
      : ChannelWithoutId(std::move(ch))
      , id(id) {}

  bool operator==(const Channel& other) const {
    return id == other.id && ChannelWithoutId::operator==(other);
  }
};

struct ClientAdvertisement {
  ClientChannelId channelId;
  std::string topic;
  std::string encoding;
  std::string schemaName;
  std::vector<uint8_t> schema;
};

struct ClientMessage {
  uint64_t logTime;
  uint64_t publishTime;
  uint32_t sequence;
  ClientAdvertisement advertisement;
  size_t dataLength;
  std::vector<uint8_t> data;

  ClientMessage(uint64_t logTime, uint64_t publishTime, uint32_t sequence,
                const ClientAdvertisement& advertisement, size_t dataLength, const uint8_t* rawData)
      : logTime(logTime)
      , publishTime(publishTime)
      , sequence(sequence)
      , advertisement(advertisement)
      , dataLength(dataLength)
      , data(dataLength) {
    std::memcpy(data.data(), rawData, dataLength);
  }

  static const size_t MSG_PAYLOAD_OFFSET = 5;

  const uint8_t* getData() const {
    return data.data() + MSG_PAYLOAD_OFFSET;
  }
  std::size_t getLength() const {
    return data.size() - MSG_PAYLOAD_OFFSET;
  }
};

struct ServiceWithoutId {
  std::string name;
  std::string type;
  std::string requestSchema;
  std::string responseSchema;
};

struct Service : ServiceWithoutId {
  ServiceId id;

  Service() = default;
  Service(const ServiceWithoutId& s, const ServiceId& id)
      : ServiceWithoutId(s)
      , id(id) {}
};

struct ServiceResponse {
  ServiceId serviceId;
  uint32_t callId;
  std::string encoding;
  std::vector<uint8_t> data;

  size_t size() const {
    return 4 + 4 + 4 + encoding.size() + data.size();
  }
  void read(const uint8_t* data, size_t size);
  void write(uint8_t* data) const;

  bool operator==(const ServiceResponse& other) const {
    return serviceId == other.serviceId && callId == other.callId && encoding == other.encoding &&
           data == other.data;
  }
};

using ServiceRequest = ServiceResponse;

enum class FetchAssetStatus : uint8_t {
  Success = 0,
  Error = 1,
};

struct FetchAssetResponse {
  uint32_t requestId;
  FetchAssetStatus status;
  std::string errorMessage;
  std::vector<uint8_t> data;
};

}  // namespace foxglove
