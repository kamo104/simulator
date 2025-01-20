#pragma once
#include "common.h"
#include "geoPos.h"
#include <deque>
#include "velocity.h"

enum class RUNWAY { L10, L28 };

struct FlightSegment {
  GeoPos<double> pos;
  Velocity vel        = {};
  bool useHeading     = false;
  bool interpolateAlt = false;
  bool interpolateVel = false;
};

struct FlightPlan {
  std::deque<FlightSegment> route;
  std::deque<FlightSegment> auxiliary;

  FlightSegment interTrg = {};
  bool vaildPathFound = true;
  RUNWAY setRunway = RUNWAY::L10;
};
