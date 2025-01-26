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
#include "sessionState.h"
#include "websocketServerState.h"

class WebsocketServer {
  net::io_context _ioc;
  std::shared_ptr<ServerState> _state;
  std::vector<std::thread> _threads;

public:
  std::atomic_bool isRunning;

  WebsocketServer(std::shared_ptr<ServerState> state);

  void newSession(std::shared_ptr<SessionState> session);

  std::shared_ptr<SessionState> getSession(const std::string &uuid);

  bool deleteSession(const std::string &uuid);
  bool send(std::string uuid, const std::string &data);
  void broadcast(const std::string &data,
                 // const std::vector<ClientType> &clientTypes,
                 const std::vector<std::string> exceptUuids = {});
  void wait();
  void run();
};
