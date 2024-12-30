#pragma once
#include "common.h"
#include "geoPos.h"
#include "velocity.h"

struct FlightSegment {
  GeoPos<double> targetPos;
  Velocity targetVel;
  bool ignoreHeading;
};

struct FlightPlan {
  std::vector<FlightSegment> route;
};
