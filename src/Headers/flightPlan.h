#pragma once
#include "common.h"
#include "geoPos.h"
#include <deque>
#include "velocity.h"

struct FlightSegment {
  GeoPos<double> pos;
  Velocity vel;
  bool useHeading;
};

struct FlightPlan {
  std::deque<FlightSegment> route;
  std::deque<FlightSegment> auxiliary;
};
