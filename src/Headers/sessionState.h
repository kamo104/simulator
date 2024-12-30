#pragma once

#include "common.h"

class Session;

class ClientType {
public:
  enum Value { Unknown, Voice, Pilot, Examiner };

  constexpr ClientType(Value aClientType) : value(aClientType) {}

  constexpr operator Value() const { return value; }

  std::string toString() {
    switch (this->value) {
    case Unknown:
      return "Unknown";
    case Voice:
      return "Voice";
    case Pilot:
      return "Pilot";
    case Examiner:
      return "Examiner";
      break;
    }
  }
  static ClientType fromString(const std::string &str) {
    if (str == "Voice")
      return Value::Voice;
    if (str == "Pilot")
      return Value::Pilot;
    if (str == "Examiner")
      return Value::Examiner;
    return Value::Unknown;
  }

private:
  Value value;
};

struct SessionState {
  // ClientType clientType = ClientType::Unknown;
  bool isConnected{false};
  std::string uuid;
  std::shared_ptr<Session> session;

  std::function<void(void)> acceptCallback;
  std::function<void(void)> disconnectCallback;
  std::function<void(const std::string &)> readCallback;
  std::function<void(size_t)> writeCallback;
};
