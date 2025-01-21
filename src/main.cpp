#include "common.h"
#include "plane.h"
#include "scenario.h"
#include "simulator.h"
// #include "simulatorState.h"
#include "messageParsing.h"
#include "websocketServer.h"
#include <chrono>
#include <sstream>

int main(int argc, char *argv[]) {
  if (argc != 4) {
    std::cerr << "Usage: simulator <address> <sim-port> <sim-threads>\n"
              << "Example:\n"
              << "    websocket-server 0.0.0.0 8080 1\n";
    // std::cerr << "Usage: simulator <address> <sim-port> <sim-threads> "
    //              "<voice-address> <voice-port> <voice-threads>\n"
    //           << "Example:\n"
    //           << "    websocket-server 0.0.0.0 8080 1 0.0.0.0 8081 1\n";
    return EXIT_FAILURE;
  }

  auto const address = std::string(argv[1]);
  auto const port = static_cast<unsigned short>(std::atoi(argv[2]));
  auto const threads = std::max<int>(1, std::atoi(argv[3]));

  std::shared_ptr<WebsocketServer> websocketServer;
  std::unique_ptr<Simulator> simulator;
  std::shared_ptr<ServerState> serverState = std::make_shared<ServerState>();
  serverState->threads = threads;
  serverState->address = address;
  serverState->port = port;

  std::shared_ptr<SimulatorState> simState = std::make_shared<SimulatorState>(
      SimulatorState{std::move(std::make_unique<std::shared_mutex>()),
                     {},
                     {},
                     std::chrono::milliseconds(1000 / 60)});

  // End of testing stuff
  serverState->acceptCallback = [&websocketServer,
                                 &simulator](auto sessionState) {
    websocketServer->newSession(sessionState);
    std::cout << "New client with uuid: " << sessionState->uuid << std::endl;
    if (!simulator->running) {
      return;
    }
    const auto &aircrafts = simulator->getPlaneData();
    json base = R"({"type":"start","aircrafts":[]})"_json;
    base["aircrafts"] = aircrafts;
    websocketServer->send(sessionState->uuid, base.dump());
  };
  serverState->disconnectCallback = [&websocketServer](auto sessionState) {
    websocketServer->deleteSession(sessionState->uuid);
    std::cout << "Client disconnected with uuid: " << sessionState->uuid
              << std::endl;
  };

  serverState->readCallback = [&websocketServer, &simState, &simulator](
                                  std::shared_ptr<SessionState> sessionState,
                                  const std::string &msg) {
    std::cout << "Message from " << sessionState->uuid << " : " << msg
              << std::endl;
    if (sessionState->nextMsgIsScenario) {
      sessionState->nextMsgIsScenario = false;
      data::Scenario scenario = json(msg);
      simState->planes = scenario.getPlanes();

      const auto &aircrafts = simulator->getPlaneData();
      json base = R"({"type":"start","aircrafts":[]})"_json;
      base["aircrafts"] = aircrafts;
      websocketServer->broadcast(base.dump());

      // start the simulator here upon receiving the scenario
      simulator->start();
      return;
    }
    // lock the state to make sure it doesn't change during parsing
    try {
      json jsonMsg = json::parse(msg);
      // make sure the sim thread is synchronised
      std::lock_guard<std::shared_mutex> guard(*simState->mtx);
      // parse message here
      messageParsing::parseMessage(jsonMsg, simState, sessionState);
      return;
    } catch (const json::parse_error &er) {
      std::cerr << "Couldn't parse the json message" << std::endl;
    } catch (const json::type_error &er) {
      std::cerr << "Wrong type used during parsing: " << er.what() << std::endl;
    }
    websocketServer->send(sessionState->uuid,
                          R"({"error":"Couldn't parse the json message."})");
  };
  serverState->writeCallback = [](auto sessionState, size_t len) {};
  websocketServer = std::make_shared<WebsocketServer>(serverState);
  simulator = std::make_unique<Simulator>(simState, websocketServer);

  websocketServer->run();
  // for testing
  // simulator->start();

  websocketServer->wait();
  simulator->wait();

  return EXIT_SUCCESS;
}
