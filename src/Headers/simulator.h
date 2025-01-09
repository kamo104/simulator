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

  void _loop();

public:
  std::atomic_bool running{false};

  Simulator(std::shared_ptr<SimulatorState> state,
            std::shared_ptr<WebsocketServer> wsServer);

  bool isRunning();

  void stop();

  void wait();

  void start();
};
