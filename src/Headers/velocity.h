#pragma once

#include "common.h"

struct Velocity {
  double value;   // ground speed in m/s
  double heading; // rad
};

inline void to_json(json &j, const Velocity &vel) {
  j = json{{"heading", vel.heading}, {"value", vel.value}};
}

inline void from_json(const json &j, Velocity &vel) {
  j.at("heading").get_to(vel.heading);
  j.at("value").get_to(vel.value);
}
