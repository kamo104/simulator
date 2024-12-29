#pragma once
#include "common.h"
#include "Geopos.h"
#include "Velocity.h"

struct FlightSegment {
    GeoPos<double> targetPos;
    Velocity targetVel;
    bool ignoreHeading;
};

struct FlightPlan {
    std::vector<FlightSegment> route;
};