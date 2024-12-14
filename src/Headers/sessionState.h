#pragma once

#include "common.h"

class Session;

struct SessionState {
  enum Client { Unknown, Voice, Pilot, Examiner } program{Unknown};
  bool isConnected{false};
  std::string uid;
  std::shared_ptr<Session> session;

  std::function<void(std::string &)> read_cb = [](auto &msg) {
    std::cout << "Message from client: " << msg << std::endl;
  };
  std::function<void(size_t)> write_cb = [](size_t bytesTransfered) {
    std::cout << "Transfered bytes: " << bytesTransfered << std::endl;
  };
};
