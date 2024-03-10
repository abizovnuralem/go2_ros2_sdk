#pragma once

#include <memory>
#include <string>

#include <websocketpp/common/connection_hdl.hpp>

#include "common.hpp"
#include "server_interface.hpp"

namespace foxglove {

class ServerFactory {
public:
  template <typename ConnectionHandle>
  static std::unique_ptr<ServerInterface<ConnectionHandle>> createServer(
    const std::string& name, const std::function<void(WebSocketLogLevel, char const*)>& logHandler,
    const ServerOptions& options);
};

}  // namespace foxglove
