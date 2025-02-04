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

  // Testing stuff
  std::shared_ptr<SimulatorState> simState = std::make_shared<SimulatorState>(
      SimulatorState{std::move(std::make_unique<std::shared_mutex>()),
                     {},
                     {},
                     // std::chrono::duration_cast<std::chrono::seconds>(
                     //     std::chrono::duration<double>(1 / 60))});
                     std::chrono::microseconds(16666)});

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

    // find the sim_id id
    const auto it = std::find_if(
        aircrafts.begin(), aircrafts.end(),
        [](const data::PlaneData &p) { return p.info.sim_id == true; });
    base["sim_id"] = -1;
    if (it != aircrafts.end()) {
      base["sim_id"] = it->info.id;
    }
    websocketServer->send(sessionState->uuid, base.dump());
    std::cerr << "sending init(to: " << sessionState->uuid
              << "): " << base.dump() << std::endl;
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
      data::Scenario scenario;
      try {
        scenario = json::parse(msg);
      } catch (const std::exception &e) {
        std::cerr << "Error parsing the scenario file: " << e.what()
                  << std::endl;
        websocketServer->send(
            sessionState->uuid,
            R"({"type":"error", "data":"Couldn't parse the scenario."})");
        return;
      }

      simState->planes = scenario.getPlanes();
      // simState

      const auto &aircrafts = simulator->getPlaneData();
      json base = R"({"type":"start","aircrafts":[]})"_json;
      base["sim_id"] = 0; // !!! tu wartosc sim_id z scenariusza
      base["aircrafts"] = aircrafts;
      // find the sim_id id
      const auto it = std::find_if(
          aircrafts.begin(), aircrafts.end(),
          [](const data::PlaneData &p) { return p.info.sim_id == true; });
      base["sim_id"] = -1;
      if (it != aircrafts.end()) {
        base["sim_id"] = it->info.id;
      }

      std::cerr << "sending init broadcast: " << base.dump() << std::endl;
      websocketServer->broadcast(base.dump());

      // start the simulator here upon receiving the scenario
      simulator->start();
      return;
    }
    try {
      json jsonMsg = json::parse(msg);
      // lock the state to make sure it doesn't change during parsing
      std::lock_guard<std::shared_mutex> guard(*simState->mtx);
      // parse message here
      messageParsing::parseMessage(jsonMsg, simState, sessionState);
      return;
    } catch (const json::parse_error &er) {
      std::cerr << "Invalid json message" << std::endl;
    } catch (const json::type_error &er) {
      std::cerr << "Wrong type used during parsing: " << er.what() << std::endl;
    }
    websocketServer->send(
        sessionState->uuid,
        R"({"type":"error", "data":"Couldn't parse the json message."})");
  };
  serverState->writeCallback = [](auto sessionState, size_t len) {};
  websocketServer = std::make_shared<WebsocketServer>(serverState);
  simulator = std::make_unique<Simulator>(simState, websocketServer);

  websocketServer->run();

  websocketServer->wait();
  simulator->wait();

  return EXIT_SUCCESS;
}
