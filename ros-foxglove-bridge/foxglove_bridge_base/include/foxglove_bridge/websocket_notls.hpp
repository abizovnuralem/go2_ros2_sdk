#pragma once

#include <websocketpp/config/asio_no_tls.hpp>
#include <websocketpp/extensions/permessage_deflate/enabled.hpp>
#include <websocketpp/server.hpp>

#include "./websocket_logging.hpp"

namespace foxglove {

struct WebSocketNoTls : public websocketpp::config::core {
  typedef WebSocketNoTls type;
  typedef core base;

  typedef base::concurrency_type concurrency_type;

  typedef base::request_type request_type;
  typedef base::response_type response_type;

  typedef base::message_type message_type;
  typedef base::con_msg_manager_type con_msg_manager_type;
  typedef base::endpoint_msg_manager_type endpoint_msg_manager_type;

  typedef CallbackLogger alog_type;
  typedef CallbackLogger elog_type;

  typedef base::rng_type rng_type;

  struct transport_config : public base::transport_config {
    typedef type::concurrency_type concurrency_type;
    typedef CallbackLogger alog_type;
    typedef CallbackLogger elog_type;
    typedef type::request_type request_type;
    typedef type::response_type response_type;
    typedef websocketpp::transport::asio::basic_socket::endpoint socket_type;
  };

  typedef websocketpp::transport::asio::endpoint<transport_config> transport_type;

  struct permessage_deflate_config {};

  typedef websocketpp::extensions::permessage_deflate::enabled<permessage_deflate_config>
    permessage_deflate_type;
};

}  // namespace foxglove
