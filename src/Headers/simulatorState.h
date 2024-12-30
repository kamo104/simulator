#pragma once
#include "common.h"
#include "geoPos.h"
#include "map.h"
#include "plane.h"
#include "planeConfig.h"
#include "vec.h"
#include <chrono>
#include <shared_mutex>

struct SimulatorState {
  std::unique_ptr<std::shared_mutex> mtx;
  std::vector<Plane> planes;
  std::unique_ptr<Map> map;
  std::chrono::duration<double, std::milli> updateInterval;
};
