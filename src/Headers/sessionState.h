#pragma once

#include "common.h"

class Session;

struct SessionState {
  enum Client { Unknown, Voice, Pilot, Examiner } program{Unknown};
  bool isConnected{false};
  std::string uuid;
  std::shared_ptr<Session> session;

  std::function<void(void)> acceptCallback;
  std::function<void(void)> disconnectCallback;
  std::function<void(const std::string &)> readCallback;
  std::function<void(size_t)> writeCallback;
};
