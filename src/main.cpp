#include "common.h"
#include "plane.h"
#include "simulator.h"
// #include "simulatorState.h"
#include "websocketServer.h"
#include <chrono>
#include <sstream>

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
                     std::chrono::microseconds(16666)});

  std::shared_ptr<const PlaneConfig> configPtr =
      std::make_shared<const PlaneConfig>(
          PlaneConfig{60.5, 241.9, 12000, 25, 20, 1.1, 1.35, 70, 30, 1000});

  FlightPlan plan;
  /*plan.route.push_back(FlightSegment{ geo2xy(GeoPos<double>{{52.7, 17,
  10000.0}}), Velocity{80, hdg2rad(0)}, true});
  plan.route.push_back(FlightSegment{ geo2xy(GeoPos<double>{{54, 16, 10000.0}}),
                                     Velocity{240, hdg2rad(0)}, false });*/

  simState->planes.push_back(
      Plane(data::PlaneData{{0, 0, false, "LOT", "286", "SP-LVN", "LOT286",
                             "2000", "Airbus A320"},
                            Velocity{100, hdg2rad(0)},
                            {{52.4, 16.9, 1000}}},
            plan, configPtr));

  FlightPlan plan2;
  plan2.route.push_back(
      FlightSegment{geo2xy(GeoPos<double>{{52.5, 16, 10000.0}}),
                    Velocity{300, hdg2rad(0)}, false});
  plan2.route.push_back(FlightSegment{geo2xy(GeoPos<double>{{54, 16, 10000.0}}),
                                      Velocity{240, hdg2rad(0)}, false});
  simState->planes.push_back(
      Plane(data::PlaneData{{1, 1, false, "LOT", "287", "SP-XTZ", "LOT287",
                             "2010", "Airbus A320"},
                            Velocity{100, hdg2rad(0)},
                            {{52.4, 16, 1000}}},
            plan2, configPtr));

  simState->planes[0].generateLandingWaypoints(true);

  // End of testing stuff

  serverState->acceptCallback = [&websocketServer](auto sessionState) {
    websocketServer->newSession(sessionState);
    std::cout << "New client with uuid: " << sessionState->uuid << std::endl;
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
      // TODO: parse the scenario here

      simulator->start();
      return;
    }
    // lock the state to make sure it doesn't change during parsing
    try {
      std::lock_guard<std::shared_mutex> guard(*simState->mtx);
      json jsonMsg = json::parse(msg);
      std::string msgType = jsonMsg["type"].template get<std::string>();

      if (msgType == "scenario") {
        sessionState->nextMsgIsScenario = true;
      } else if (msgType == "info") {
        // TODO: we shouldn't be getting this message
        // FIX: maybe the examiner should be starting?
      } else if (msgType == "positions") {
        // TODO: update the simulation with the "real" planes's positions
        std::vector<data::PlaneData> planeData =
            jsonMsg["aircrafts"].template get<std::vector<data::PlaneData>>();
        for (Plane &plane : simState->planes) {
          auto it = std::find_if(planeData.begin(), planeData.end(),
                                 [&plane](const data::PlaneData &p) {
                                   return p.info.id == plane.info.id;
                                 });
          if (it != planeData.end()) {
            plane.setData(*it);
          }
        }
      } else if (msgType == "order") {
        // TODO: pass the order to the "real" planes and update the simulation
      }
      // else if (msgType == "plane-pick") {
      //   // TODO: pick a plane used by the client
      //   int pickedId = jsonMsg["id"].template get<int>();
      //   { // check if the pilot already has a plane in use
      //     auto it =
      //         std::find_if(simState->planes.begin(), simState->planes.end(),
      //                      [&sessionState](const Plane &p) {
      //                        return p.uuid == sessionState->uuid;
      //                      });
      //     if (it != simState->planes.end()) {
      //       websocketServer->send(
      //           sessionState->uuid,
      //           R"({"error":"This pilot already has a different plane."})");
      //       return;
      //     }
      //   }
      //   { // check for open planes
      //     auto it2 = std::find_if(
      //         simState->planes.begin(), simState->planes.end(),
      //         [pickedId](const Plane &p) { return p.info.id == pickedId; });
      //     if (it2 != simState->planes.end()) {
      //       it2->uuid = sessionState->uuid;
      //       websocketServer->send(sessionState->uuid, R"({"status":"OK"})");
      //       return;
      //     }
      //     websocketServer->send(sessionState->uuid,
      //                           R"({"error":"The plane with the selected id "
      //                           "has already been picked."})");
      //   }
      // }

    } catch (const json::parse_error &er) {
      std::cerr << "Couldn't parse the json message" << std::endl;
      websocketServer->send(sessionState->uuid,
                            R"({"error":"Couldn't parse the json message."})");
    } catch (const json::type_error &er) {
      std::cerr << "Wrong type used during parsing" << std::endl;
    }
  };
  serverState->writeCallback = [](auto sessionState, size_t len) {
    // std::cout << "Wrote message to: " << sessionState->uuid << " len: " <<
    // len
    //           << std::endl;
  };
  websocketServer = std::make_shared<WebsocketServer>(serverState);
  simulator = std::make_unique<Simulator>(simState, websocketServer);

  websocketServer->run();
  // simulator->start();

  websocketServer->wait();
  simulator->wait();

  return EXIT_SUCCESS;
}
