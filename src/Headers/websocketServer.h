#pragma once
#include "common.h"
#include "listener.h"
#include "session.h"
#include "websocketServerState.h"

class WebsocketServer {
  std::shared_ptr<ServerState> _state;

public:
  WebsocketServer(std::shared_ptr<ServerState> state) : _state(state) {}

  void newSession(std::shared_ptr<SessionState> session) {
    _state->sessions.emplace(session->uuid, session);
  }

  std::shared_ptr<SessionState> getSession(const std::string &uuid) {
    auto it = _state->sessions.find(uuid);
    if (it == _state->sessions.end()) {
      return nullptr;
    }
    return it->second;
  }

  bool deleteSession(const std::string &uuid) {
    auto it = _state->sessions.find(uuid);
    if (it == _state->sessions.end()) {
      return false;
    }
    _state->sessions.erase(it);
    return true;
  }

  bool send(std::string uuid, std::string message) {
    auto it = _state->sessions.find(uuid);
    if (it == _state->sessions.end()) {
      return false;
    }
    it->second->session->send(message);
    return true;
  }
  void broadcast(std::string data) {
    for (auto &[key, val] : _state->sessions) {
      val->session->send(data);
    }
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
