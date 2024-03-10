#include <websocketpp/common/connection_hdl.hpp>

#include <foxglove_bridge/server_factory.hpp>
#include <foxglove_bridge/websocket_notls.hpp>
#include <foxglove_bridge/websocket_server.hpp>
#include <foxglove_bridge/websocket_tls.hpp>

namespace foxglove {

template <>
std::unique_ptr<ServerInterface<websocketpp::connection_hdl>> ServerFactory::createServer(
  const std::string& name, const std::function<void(WebSocketLogLevel, char const*)>& logHandler,
  const ServerOptions& options) {
  if (options.useTls) {
    return std::make_unique<foxglove::Server<foxglove::WebSocketTls>>(name, logHandler, options);
  } else {
    return std::make_unique<foxglove::Server<foxglove::WebSocketNoTls>>(name, logHandler, options);
  }
}

template <>
inline void Server<WebSocketNoTls>::setupTlsHandler() {
  _server.get_alog().write(APP, "Server running without TLS");
}

template <>
inline void Server<WebSocketTls>::setupTlsHandler() {
  _server.set_tls_init_handler([this](ConnHandle hdl) {
    (void)hdl;

    namespace asio = websocketpp::lib::asio;
    auto ctx = websocketpp::lib::make_shared<asio::ssl::context>(asio::ssl::context::sslv23);

    try {
      ctx->set_options(asio::ssl::context::default_workarounds | asio::ssl::context::no_tlsv1 |
                       asio::ssl::context::no_sslv2 | asio::ssl::context::no_sslv3);
      ctx->use_certificate_chain_file(_options.certfile);
      ctx->use_private_key_file(_options.keyfile, asio::ssl::context::pem);

      // Ciphers are taken from the websocketpp example echo tls server:
      // https://github.com/zaphoyd/websocketpp/blob/1b11fd301/examples/echo_server_tls/echo_server_tls.cpp#L119
      constexpr char ciphers[] =
        "ECDHE-RSA-AES128-GCM-SHA256:ECDHE-ECDSA-AES128-GCM-SHA256:ECDHE-RSA-AES256-GCM-SHA384:"
        "ECDHE-ECDSA-AES256-GCM-SHA384:DHE-RSA-AES128-GCM-SHA256:DHE-DSS-AES128-GCM-SHA256:kEDH+"
        "AESGCM:ECDHE-RSA-AES128-SHA256:ECDHE-ECDSA-AES128-SHA256:ECDHE-RSA-AES128-SHA:ECDHE-ECDSA-"
        "AES128-SHA:ECDHE-RSA-AES256-SHA384:ECDHE-ECDSA-AES256-SHA384:ECDHE-RSA-AES256-SHA:ECDHE-"
        "ECDSA-AES256-SHA:DHE-RSA-AES128-SHA256:DHE-RSA-AES128-SHA:DHE-DSS-AES128-SHA256:DHE-RSA-"
        "AES256-SHA256:DHE-DSS-AES256-SHA:DHE-RSA-AES256-SHA:!aNULL:!eNULL:!EXPORT:!DES:!RC4:!3DES:"
        "!MD5:!PSK";

      if (SSL_CTX_set_cipher_list(ctx->native_handle(), ciphers) != 1) {
        _server.get_elog().write(RECOVERABLE, "Error setting cipher list");
      }
    } catch (const std::exception& ex) {
      _server.get_elog().write(RECOVERABLE,
                               std::string("Exception in TLS handshake: ") + ex.what());
    }
    return ctx;
  });
}

}  // namespace foxglove
