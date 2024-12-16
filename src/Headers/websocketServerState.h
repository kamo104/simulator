#pragma once
#include "common.h"
#include "session.h"

struct ServerState {
  std::unordered_map<std::string, std::shared_ptr<SessionState>> sessions;

  std::function<void(std::shared_ptr<SessionState>)> acceptCallback;
  std::function<void(std::shared_ptr<SessionState>)> disconnectCallback;
  std::function<void(std::shared_ptr<SessionState>, const std::string &)>
      readCallback;
  std::function<void(std::shared_ptr<SessionState>, size_t)> writeCallback;

  int threads;
  std::string address;
  uint16_t port;
};
