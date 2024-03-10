#include "foxglove_bridge/foxglove_bridge.hpp"

#include "websocketpp/version.hpp"

namespace foxglove {

const char* WebSocketUserAgent() {
  return websocketpp::user_agent;
}

}  // namespace foxglove
