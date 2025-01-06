#pragma once
#include "common.h"
#include "geoPos.h"
#include "map.h"
#include "plane.h"
#include "planeConfig.h"
#include "simulatorState.h"
#include "vec.h"
#include "websocketServer.h"
#include <chrono>
#include <mutex>
#include <thread>

class Simulator {
  std::shared_ptr<SimulatorState> _state;
  std::shared_ptr<WebsocketServer> _wsServer;
  std::thread _workerThread;

  std::atomic_bool _runLoop{true};
  std::chrono::steady_clock::time_point _lastTime;

  void _loop() {
    {
      std::lock_guard<std::shared_mutex> guard(*_state->mtx);
      json base = R"({"type":"start","aircrafts":[]})"_json;

      std::vector<data::PlaneData> aircrafts;
      aircrafts.reserve(_state->planes.size());
      for (const auto &plane : _state->planes) {
        aircrafts.emplace_back(plane.getData());
      }
      base["aircrafts"] = aircrafts;
      // std::cout << base.dump() << std ::endl;
      _wsServer->broadcast(base.dump());
    }
    while (_runLoop) {
      // calculating time delta
      auto now = std::chrono::steady_clock::now();
      auto deltaDuration = std::chrono::duration<double>(now - _lastTime);
      _lastTime = now;

      double timeDelta = deltaDuration.count();

      // updating planes
      {
        std::lock_guard<std::shared_mutex> guard(*_state->mtx);
        for (auto &plane : _state->planes) {
          plane.update(timeDelta);
        }
      }

      // sending the planes' positions
      std::vector<data::PlaneFlightData> flightData;
      flightData.reserve(_state->planes.size());
      for (const auto &plane : _state->planes) {
        flightData.emplace_back(plane.getFlightData());
      }
      json base = R"({"type":"positions","aircrafts":[]})"_json;
      base["aircrafts"] = flightData;
      _wsServer->broadcast(base.dump());

      auto n2 = std::chrono::steady_clock::now();
      std::this_thread::sleep_for(_state->updateInterval - (n2 - now));
    }
  }

public:
  std::atomic_bool running{false};

  Simulator(std::shared_ptr<SimulatorState> state,
            std::shared_ptr<WebsocketServer> wsServer)
      : _state(state), _wsServer(wsServer) {}

  bool isRunning() { return running; }

  void stop() { _runLoop = false; }

  void wait() {
    if (!running) {
      _runLoop = false;
      return;
    }

    if (_workerThread.joinable()) {
      _workerThread.join();
    }
    running = false;
  }

  void start() {
    if (running) {
      _runLoop = true;
      return;
    }

    std::cout << "starting plane simulation" << std::endl;
    running = true;
    _runLoop = true;
    _lastTime = std::chrono::steady_clock::now();

    _workerThread = std::thread(&Simulator::_loop, this);
  }
};
