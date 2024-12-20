#pragma once

// server based upon the example in boost beast library:
//
// Copyright (c) 2016-2019 Vinnie Falco (vinnie dot falco at gmail dot com)
//
// Distributed under the Boost Software License, Version 1.0. (See accompanying
// file LICENSE_1_0.txt or copy at http://www.boost.org/LICENSE_1_0.txt)
//
// Official repository: https://github.com/boostorg/beast
//

//------------------------------------------------------------------------------
//
// Example: WebSocket server, asynchronous
//
//------------------------------------------------------------------------------

#include "common.h"
#include "listener.h"
#include "session.h"
#include "websocketServerState.h"

class WebsocketServer {
  net::io_context _ioc;
  std::shared_ptr<ServerState> _state;
  std::vector<std::thread> _threads;

public:
  std::atomic_bool isRunning;

  WebsocketServer(std::shared_ptr<ServerState> state)
      : _state(state), _ioc(net::io_context{state->threads}) {
    auto const addr = net::ip::make_address(_state->address);
    // Create and launch a listening port
    auto listener = std::make_shared<Listener>(
        _ioc, tcp::endpoint{addr, _state->port}, _state);

    listener->run();
  }

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

  void wait() {
    if (!isRunning)
      return;

    for (std::thread &t : _threads) {
      if (t.joinable()) {
        t.join();
      }
    }
  }

  void run() {
    if (isRunning)
      return;

    isRunning = true;
    for (int i = 0; i < _state->threads; i++)
      _threads.emplace_back([this] { _ioc.run(); });
  }
};
