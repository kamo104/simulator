#pragma once
#include "common.h"
#include "geoPos.h"
#include "vec.h"
#include <atomic>

struct PlaneInfo {
  int id;
  int sim_id;
  bool isGrounded;
  std::string airline;
  std::string flightNumber;
  std::string planeNumber;
  std::string callSign;
  std::string squawk;
  std::string model;
};

class Plane {
public:
  PlaneInfo info;
  Vec<float, 2> vel;
  GeoPos<float> pos;
  GeoPos<float> targetPos;
};
