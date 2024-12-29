#pragma once
#include "common.h"
#include "geoPos.h"
#include "map.h"
#include "plane.h"
#include "planeConfig.h"
#include "vec.h"

struct SimulatorState {
  std::vector<Plane> planes;
  //std::map<std::string, PlaneConfig> planeConfigs; 
  // int threads;
  std::unique_ptr<Map> map;
};
