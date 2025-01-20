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

  //This should be loaded into map from map and used when loading scenario 
  std::shared_ptr<const PlaneConfig> configPtr =
      std::make_shared<const PlaneConfig>(
          PlaneConfig{40, 160, 12000, 5, 25, 220, 10, 1.35, 2.56, 70, 1000, 
                      kts2ms(20), kts2ms(30) }); //in sim units

  std::shared_ptr<const PlaneConfig> configPtr2 =
    std::make_shared<const PlaneConfig>(
      PlaneConfig{ 60.5, 241.9, 12000, 5, 25, 220, 10, 1.35, 2.56, 70, 2000, 
                   kts2ms(20), kts2ms(30) }); //in sim units


  FlightPlan plan0;
  simState->planes.push_back(
      Plane(data::PlaneData{ {0, true, 10000, "LOT", "286",
          "SP-LVN", "PLAYER", "2500", "Mooney Bravo"},
          Velocity{0, hdg2rad(-50)}, xy2geo(exitInner10_2.pos) },
          plan0, configPtr));

  simState->planes.front().setModePlayer();

  FlightPlan plan;
  plan.route.push_back(FlightSegment{ geo2xy(GeoPos<double>{{52.42, 16.82, 10000.0}}),
                                    Velocity{80, hdg2rad(0)}});
  simState->planes.push_back(
      Plane(data::PlaneData{ {1, false, 10000, "LOT", "286",
          "SP-LVN", "LOT286", "2000", "Mooney Bravo"},
          Velocity{100, hdg2rad(-30)}, {{52.4, 16.8, 1000}} }, 
          plan, configPtr2));

  FlightPlan plan2;
  plan2.route.push_back(FlightSegment{ geo2xy(GeoPos<double>{{52.5, 16.5, 10000.0}}),
                                     Velocity{100, hdg2rad(270)}, true });
  plan2.route.push_back(FlightSegment{ geo2xy(GeoPos<double>{{52.5, 16.7, 10000.0}}),
                                     Velocity{120, hdg2rad(0)} });
  simState->planes.push_back(
      Plane(data::PlaneData{ {2, false, 10000, "LOT", "287",
          "SP-XTZ", "LOT287", "2010", "Mooney Bravo"},
          Velocity{100, hdg2rad(0)}, {{52.4, 16.45, 1500}} },
          plan2, configPtr2));
 
  simState->planes.push_back(
    Plane(data::PlaneData{ {3, false, 1000, "", "",
        "SP-256", "SP-256", "2010", "Mooney Bravo"},
        Velocity{100, hdg2rad(270)}, {{52.41, 16.89, 2000}} },
      plan2, configPtr));

  simState->planes.push_back(
    Plane(data::PlaneData{ {4, true, 1000, "", "",
        "SP-001", "SP-001", "2000", "Mooney Bravo"},
        Velocity{0, hdg2rad(-20)}, {{52.416081, 16.827386, 0}} },
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
    base["sim_id"] = 0; // !!! tu wartosc sim_id z scenariusza
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
      base["sim_id"] = 0; // !!! tu wartosc sim_id z scenariusza
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
        std::vector<data::PlaneFlightData> planeData =
            jsonMsg["aircrafts"].template get<std::vector<data::PlaneFlightData>>();
        for (Plane &plane : simState->planes) {
          auto it = std::find_if(planeData.begin(), planeData.end(),
                                 [&plane](const data::PlaneFlightData &p) {
                                   return p.id == plane._info.id;
                                 });
          if (it != planeData.end()) {
            plane.setFlightData(*it);
          }
        }
      } else if (msgType == "order") {
        // TODO: pass the order to the "real" planes and update the simulation
        std::string orderType = jsonMsg["order_type"];
        int id = jsonMsg["id"];
        if (id == 0) return; // !!! tu wartosc sim_id z scenariusza

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
            {"enter_holding", 
             [it, &jsonMsg]() { 
              it->taxiToRunway(jsonMsg["data"]["name"].template get<std::string>());
             }},
            {"landing", 
             [it, &jsonMsg]() { 
              it->landing(jsonMsg["data"]["name"].template get<std::string>()); 
             }},
            {"touch_and_go", 
             [it, &jsonMsg]() { 
              it->landing(jsonMsg["data"]["name"].template get<std::string>()); 
             }},
            {"enter_runway", [it, &jsonMsg]() { it->enterRunway(); }},
            {"enter_airport_loop",
             [it, &jsonMsg]() { it->takeOff(); }}};

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
