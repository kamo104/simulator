#pragma once
#include "common.h"
#include "session.h"

struct ServerState {
  std::unordered_map<std::string, std::shared_ptr<SessionState>> sessions;
  std::vector<std::string> voiceSessions;
  std::function<void(std::shared_ptr<SessionState>)> acceptCallback =
      [](auto state) {
        std::cout << "New connection from: " << state->uid << std::endl;
      };
  int threads;
  std::string address;
  uint16_t port;
};
