#pragma once
#include "common.h"
#include "geoPos.h"
#include "map.h"
#include "plane.h"
#include "simulatorState.h"
#include "vec.h"

class Simulator {
  std::shared_ptr<SimulatorState> _state;
  // std::vector<std::shared_ptr<SimulatorThreadState>> _threads;
  // std::vector<std::thread> _threads;
  std::thread _workerThread;

  std::atomic_bool _runLoop{true};

public:
  std::atomic_bool running{false};

  Simulator(std::shared_ptr<SimulatorState> state) : _state(state) {}

  bool isRunning() { return running; }

  void loop() {
    while (_runLoop) {
      // _state->planes;
      for (auto &plane : _state->planes) {
        // plane.
      }
    }
  }

  void stop() { _runLoop = false; }

  void wait() {
    if (!running) {
      _runLoop = false;
      return;
    }

    // for (auto &t : _threads) {
    //   if (t->_thread.joinable()) {
    //     t->_thread.join();
    //   }
    // }
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

    running = true;
    _runLoop = true;

    _workerThread = std::thread(&Simulator::loop, this);
    // for (int i = 0; i < _state->threads; i++) {
    //   std::shared_ptr<SimulatorThreadState> threadState =
    //       std::make_shared<SimulatorThreadState>();
    //   threadState->_thread = std::thread(&Simulator::loop, this);
    //   _threads.emplace_back(threadState);
    // }
  }
};
