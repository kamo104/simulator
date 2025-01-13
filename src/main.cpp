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
          PlaneConfig{60.5, 241.9, 12000, 25, 20, 1.35, 2.56, 70, 30, 1000});

  FlightPlan plan;
  // plan.route.push_back(FlightSegment{ geo2xy(GeoPos<double>{{52.42, 16.82,
  // 10000.0}}),
  //                                   Velocity{80, hdg2rad(0)}, false});
  ///*plan.route.push_back(FlightSegment{ geo2xy(GeoPos<double>{{54, 16,
  /// 10000.0}}),
  //                                   Velocity{240, hdg2rad(0)}, false });*/

  simState->planes.push_back(
      Plane(data::PlaneData{{0, 0, false, "LOT", "286", "SP-LVN", "LOT286",
                             "2000", "Airbus A320"},
                            Velocity{100, hdg2rad(0)},
                            {{52.4, 16.9, 1000}}},
            plan, configPtr));

  FlightPlan plan2;
  plan2.route.push_back(
      FlightSegment{geo2xy(GeoPos<double>{{52.5, 16.5, 10000.0}}),
                    Velocity{100, hdg2rad(90)}, true});
  plan2.route.push_back(
      FlightSegment{geo2xy(GeoPos<double>{{52.5, 16.7, 10000.0}}),
                    Velocity{120, hdg2rad(0)}, false});
  simState->planes.push_back(
      Plane(data::PlaneData{{1, 1, false, "LOT", "287", "SP-XTZ", "LOT287",
                             "2010", "Airbus A320"},
                            Velocity{100, hdg2rad(0)},
                            {{52.4, 16.4, 1000}}},
            plan2, configPtr));

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
      // TODO: parse the scenario here
      const auto &aircrafts = simulator->getPlaneData();
      json base = R"({"type":"start","aircrafts":[]})"_json;
      base["aircrafts"] = aircrafts;
      websocketServer->broadcast(base.dump());

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
                                   return p.info.id == plane._info.id;
                                 });
          if (it != planeData.end()) {
            plane.setData(*it);
          }
        }
      } else if (msgType == "order") {
        // TODO: pass the order to the "real" planes and update the simulation
        std::string orderType = jsonMsg["order_type"];
        int id = jsonMsg["id"];
        auto it =
            std::find_if(simState->planes.begin(), simState->planes.end(),
                         [id](const Plane &p1) { return p1._info.id == id; });
        if (it == simState->planes.end()) {
          std::cerr << "Plane from the order not found! id: " << id
                    << std::endl;
        }
        std::unordered_map<std::string, const std::function<void()>> fnMap = {
            {"altitude",
             [it, &jsonMsg]() {
               it->setAltitude(jsonMsg["data"]["value"].template get<float>());
             }},
            {"headtopoint",
             [it, &jsonMsg]() {
               double lat = jsonMsg["data"]["lat"];
               double lon = jsonMsg["data"]["long"];
               it->setHeadpoint(GeoPos<double>{{lat, lon, -1}});
             }},
            {"heading",
             [it, &jsonMsg]() {
               it->setHeading(jsonMsg["data"]["value"].template get<float>());
             }},
            {"velocity",
             [it, &jsonMsg]() {
               it->setVelocity(jsonMsg["data"]["value"].template get<float>());
             }},
            {"squawk",
             [it, &jsonMsg]() {
               it->setSquawk(
                   jsonMsg["data"]["value"].template get<std::string>());
             }},
            {"follow_flight_plan",
             [it, &jsonMsg]() { it->followFlightPlan(); }},
            {"enter_holding", [it, &jsonMsg]() { it->enterHolding(); }},
            {"landing", [it, &jsonMsg]() { it->landing(); }},
            {"touch_and_go", [it, &jsonMsg]() { it->touchAndGo(); }},
            {"enter_airport_loop",
             [it, &jsonMsg]() { it->enterAirportLoop(); }}};

        if (fnMap.find(orderType) != fnMap.end()) {
          fnMap[orderType]();
        } else {
          std::cerr << "Unknown order type: " << orderType << std::endl;
        }
      }
    } catch (const json::parse_error &er) {
      std::cerr << "Couldn't parse the json message" << std::endl;
      websocketServer->send(sessionState->uuid,
                            R"({"error":"Couldn't parse the json message."})");
    } catch (const json::type_error &er) {
      std::cerr << "Wrong type used during parsing" << std::endl;
    }
  };
  serverState->writeCallback = [](auto sessionState, size_t len) {};
  websocketServer = std::make_shared<WebsocketServer>(serverState);
  simulator = std::make_unique<Simulator>(simState, websocketServer);

  websocketServer->run();
  // simulator->start();

  websocketServer->wait();
  simulator->wait();

  return EXIT_SUCCESS;
}
