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

    _state->acceptCallback = [this](auto state) {
      this->newSession(state);
      std::cout << "New client with uuid: " << state->uuid << std::endl;
    };
    _state->disconnectCallback = [this](auto state) {
      std::cout << "Client disconnected with uuid: " << state->uuid
                << std::endl;
    };
    _state->readCallback = [this](auto state, const auto &msg) {
      std::cout << "Message from " << state->uuid << " : " << msg << std::endl;
      // this->send(state->uuid, msg);
      // this->broadcast(msg);
    };
    _state->writeCallback = [this](auto state, size_t len) {
      std::cout << "Wrote message to: " << state->uuid << " len: " << len
                << std::endl;
    };
  };
  void newSession(std::shared_ptr<SessionState> session) {
    _state->sessions.emplace(session->uuid, session);
  }

  void send(std::string uuid, std::string message) {
    auto it = _state->sessions.find(uuid);
    if (it == _state->sessions.end()) {
      return;
    }
    it->second->session->send(message);
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
