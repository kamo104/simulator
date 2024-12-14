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
           std::shared_ptr<ServerState> state)
      : _ioc(ioc), _acceptor(ioc), _state(state) {
    beast::error_code ec;

    // Open the acceptor
    _acceptor.open(endpoint.protocol(), ec);
    if (ec) {
      fail(ec, "open");
      return;
    }

    // Allow address reuse
    _acceptor.set_option(net::socket_base::reuse_address(true), ec);
    if (ec) {
      fail(ec, "set_option");
      return;
    }

    // Bind to the server address
    _acceptor.bind(endpoint, ec);
    if (ec) {
      fail(ec, "bind");
      return;
    }

    // Start listening for connections
    _acceptor.listen(net::socket_base::max_listen_connections, ec);
    if (ec) {
      fail(ec, "listen");
      return;
    }
  }

  // Start accepting incoming connections
  void run() { do_accept(); }

private:
  void do_accept() {
    // The new connection gets its own strand
    _acceptor.async_accept(
        net::make_strand(_ioc),
        beast::bind_front_handler(&Listener::on_accept, shared_from_this()));
  }

  void on_accept(beast::error_code ec, tcp::socket socket) {
    if (ec) {
      fail(ec, "accept");
    } else {
      // Create the session and run it
      auto sessionState = std::make_shared<SessionState>();
      sessionState->uid = uuids::to_string(_generator());
      sessionState->isConnected = true;
      _state->acceptCallback(sessionState);
      sessionState->session =
          std::make_shared<Session>(std::move(socket), _ioc);
      sessionState->session->run();
    }

    // Accept another connection
    do_accept();
  }
};
