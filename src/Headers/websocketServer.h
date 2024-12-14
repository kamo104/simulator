#pragma once
#include "common.h"
#include "listener.h"
#include "session.h"
#include "websocketServerState.h"

class WebsocketServer {
  std::shared_ptr<ServerState> _state;

public:
  WebsocketServer(int threads, std::string address, uint16_t port) {
    _state = std::make_shared<ServerState>();
    _state->threads = threads;
    _state->address = address;
    _state->port = port;
  };
  void newSession(std::string uid, std::shared_ptr<SessionState> session) {
    _state->sessions.emplace(uid, session);
  }

  void send(std::string uid, std::string message) {
    auto it = _state->sessions.find(uid);
    if (it == _state->sessions.end()) {
      return;
    }
    it->second->session->send(message);
  }
  void broadcast(std::vector<uint8_t> data) {
    // sessionMap.find()
  }

  void run() {
    auto const addr = net::ip::make_address(_state->address);
    // The io_context is required for all I/O
    net::io_context ioc{_state->threads};

    // Create and launch a listening port
    std::make_shared<Listener>(ioc, tcp::endpoint{addr, _state->port}, _state)
        ->run();

    // Run the I/O service on the requested number of threads
    std::vector<std::thread> v;
    v.reserve(_state->threads - 1);
    for (auto i = _state->threads - 1; i > 0; --i)
      v.emplace_back([&ioc] { ioc.run(); });
    ioc.run();
  }
};
