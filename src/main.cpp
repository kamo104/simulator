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
#include "websocketServer.h"

int main(int argc, char *argv[]) {
  if (argc != 4) {
    std::cerr << "Usage: websocket-server <address> <port> <threads>\n"
              << "Example:\n"
              << "    websocket-server 0.0.0.0 8080 1\n";
    return EXIT_FAILURE;
  }

  auto const address = std::string(argv[1]);
  auto const port = static_cast<unsigned short>(std::atoi(argv[2]));
  auto const threads = std::max<int>(1, std::atoi(argv[3]));

  std::unique_ptr<WebsocketServer> websocketServer;
  std::shared_ptr<ServerState> serverState = std::make_shared<ServerState>();
  serverState->threads = threads;
  serverState->address = address;
  serverState->port = port;

  serverState->acceptCallback = [&websocketServer](auto sessionState) {
    websocketServer->newSession(sessionState);
    std::cout << "New client with uuid: " << sessionState->uuid << std::endl;
  };
  serverState->disconnectCallback = [&websocketServer](auto sessionState) {
    websocketServer->deleteSession(sessionState->uuid);
    std::cout << "Client disconnected with uuid: " << sessionState->uuid
              << std::endl;
  };
  serverState->readCallback = [&websocketServer](auto sessionState,
                                                 const auto &msg) {
    std::cout << "Message from " << sessionState->uuid << " : " << msg
              << std::endl;
    try {
      json jsonMsg = json::parse(msg);
      std::string msgType = jsonMsg["type"].template get<std::string>();

      if (msgType == "scenario") {
        // TODO: parse the scenario here
      } else if (msgType == "start") {
        // TODO: we shouldn't be getting this message
        // FIX: maybe the examiner should be starting?
      } else if (msgType == "positions") {
        // TODO: update the simulation with the "real" planes's positions
      } else if (msgType == "order") {
        // TODO: pass the order to the "real" planes and update the simulation
      }
    } catch (const json::parse_error &er) {
      std::cerr << "Couldn't parse the json message" << std::endl;
    } catch (const json::type_error &er) {
      std::cerr << "Wrong type used during parsing" << std::endl;
    }
  };
  serverState->writeCallback = [&websocketServer](auto sessionState,
                                                  size_t len) {
    std::cout << "Wrote message to: " << sessionState->uuid << " len: " << len
              << std::endl;
  };
  websocketServer = std::make_unique<WebsocketServer>(serverState);
  websocketServer->run();

  websocketServer->wait();

  return EXIT_SUCCESS;
}
