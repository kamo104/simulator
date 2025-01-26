#pragma once

#include "common.h"
#include "session.h"
#include "websocketServerState.h"

// Accepts incoming connections and launches the sessions
class Listener : public std::enable_shared_from_this<Listener> {
  net::io_context &_ioc;
  tcp::acceptor _acceptor;

  std::shared_ptr<ServerState> _state;
  uuids::random_generator _generator;

public:
  Listener(net::io_context &ioc, tcp::endpoint endpoint,
           std::shared_ptr<ServerState> state);
  void run();

private:
  void do_accept();

  void on_accept(beast::error_code ec, tcp::socket socket);
};
