#pragma once
#include "common.h"

struct PlaneConfig {
    double minSpeed;
    double maxSpeed;
    double maxAltitude;

    double deafultClimbingSpeed;
    double accelerationFactor;

    double normalLoad;
    double maxLoad;
};