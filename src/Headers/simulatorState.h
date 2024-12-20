#pragma once
#include "common.h"
#include "geoPos.h"
#include "map.h"
#include "plane.h"
#include "vec.h"

struct SimulatorState {
  std::vector<Plane> planes;
  // int threads;
  std::unique_ptr<Map> map;
};
