#pragma once

#include "common.h"
#include "sessionState.h"

class Session : public std::enable_shared_from_this<Session> {
  websocket::stream<beast::tcp_stream> _ws;
  http::request<http::string_body> _req;
  net::strand<net::io_context::executor_type> _strand;

  std::shared_ptr<SessionState> _state;
  beast::flat_buffer _recvBuffer;

public:
  explicit Session(tcp::socket &&socket, net::io_context &ioc,
                   std::shared_ptr<SessionState> state);
  // Start the asynchronous operation
  void run();
  void on_http_read(beast::error_code ec, std::size_t bytes_transferred);
  void on_accept(beast::error_code ec);
  void send(const std::string &message);
  void do_read();
  void on_read(beast::error_code ec, std::size_t bytesTransferred);

  void on_write(beast::error_code ec, std::size_t bytesTransferred);
};
