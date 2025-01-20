#pragma once
#include "common.h"

struct PlaneConfig {
    double minSpeed;
    double maxSpeed;
    double maxAltitude;

    double fuelConsumption;

    double deafultClimbingSpeed;
    double cruiseSpeed;
    double accelerationFactor;

    double normalLoad;
    double maxLoad;

    double landingSpeed;
    double landingDistance;

    double taxiSpeed;
    double taxiMaxSpeed;
};