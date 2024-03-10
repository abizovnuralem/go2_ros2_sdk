#pragma once

#include <functional>

#include <websocketpp/common/asio.hpp>
#include <websocketpp/logger/levels.hpp>

#include "common.hpp"

namespace foxglove {

using LogCallback = std::function<void(WebSocketLogLevel, char const*)>;

inline std::string IPAddressToString(const websocketpp::lib::asio::ip::address& addr) {
  if (addr.is_v6()) {
    return "[" + addr.to_string() + "]";
  }
  return addr.to_string();
}

inline void NoOpLogCallback(WebSocketLogLevel, char const*) {}

class CallbackLogger {
public:
  using channel_type_hint = websocketpp::log::channel_type_hint;

  CallbackLogger(channel_type_hint::value hint = channel_type_hint::access)
      : _staticChannels(0xffffffff)
      , _dynamicChannels(0)
      , _channelTypeHint(hint)
      , _callback(NoOpLogCallback) {}

  CallbackLogger(websocketpp::log::level channels,
                 channel_type_hint::value hint = channel_type_hint::access)
      : _staticChannels(channels)
      , _dynamicChannels(0)
      , _channelTypeHint(hint)
      , _callback(NoOpLogCallback) {}

  void set_callback(LogCallback callback) {
    _callback = callback;
  }

  void set_channels(websocketpp::log::level channels) {
    if (channels == 0) {
      clear_channels(0xffffffff);
      return;
    }

    _dynamicChannels |= (channels & _staticChannels);
  }

  void clear_channels(websocketpp::log::level channels) {
    _dynamicChannels &= ~channels;
  }

  void write(websocketpp::log::level channel, std::string const& msg) {
    write(channel, msg.c_str());
  }

  void write(websocketpp::log::level channel, char const* msg) {
    if (!this->dynamic_test(channel)) {
      return;
    }

    if (_channelTypeHint == channel_type_hint::access) {
      _callback(WebSocketLogLevel::Info, msg);
    } else {
      if (channel == websocketpp::log::elevel::devel) {
        _callback(WebSocketLogLevel::Debug, msg);
      } else if (channel == websocketpp::log::elevel::library) {
        _callback(WebSocketLogLevel::Debug, msg);
      } else if (channel == websocketpp::log::elevel::info) {
        _callback(WebSocketLogLevel::Info, msg);
      } else if (channel == websocketpp::log::elevel::warn) {
        _callback(WebSocketLogLevel::Warn, msg);
      } else if (channel == websocketpp::log::elevel::rerror) {
        _callback(WebSocketLogLevel::Error, msg);
      } else if (channel == websocketpp::log::elevel::fatal) {
        _callback(WebSocketLogLevel::Critical, msg);
      }
    }
  }

  constexpr bool static_test(websocketpp::log::level channel) const {
    return ((channel & _staticChannels) != 0);
  }

  bool dynamic_test(websocketpp::log::level channel) {
    return ((channel & _dynamicChannels) != 0);
  }

private:
  websocketpp::log::level const _staticChannels;
  websocketpp::log::level _dynamicChannels;
  channel_type_hint::value _channelTypeHint;
  LogCallback _callback;
};

}  // namespace foxglove
