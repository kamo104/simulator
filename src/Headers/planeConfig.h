#pragma once
#include "common.h"

struct PlaneConfig {
    double minSpeed;
    double maxSpeed;
    double maxAltitude;

    double fuelConsumption;

    double deafultClimbingSpeed;
    double accelerationFactor;

    double normalLoad;
    double maxLoad;

    double landingSpeed;
    double landingDecelration;
    double landingDistance;
};