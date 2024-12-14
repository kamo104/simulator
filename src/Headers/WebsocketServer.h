#include "common.h"
#include "session.h"

struct ServerState {
  std::unordered_map<std::string, std::shared_ptr<SessionState>> sessionMap;
  std::vector<std::string> voiceSessions;
};

class WebsocketServer {
  ServerState serverState;

public:
  WebsocketServer() = default;
  void newSession(std::string uid, std::shared_ptr<SessionState> session) {
    serverState.sessionMap.emplace(uid, session);
  }

  void send(std::string uid, std::string message) {
    auto it = serverState.sessionMap.find(uid);
    if (it == serverState.sessionMap.end()) {
      return;
    }
    it->second->session.send(message);
  }
  void broadcastVoice(std::vector<uint8_t> data) {
    // sessionMap.find()
  }
} websocketServer;
