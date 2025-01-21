#pragma once

#include "common.h"
#include "flightPlan.h"
#include "geoPos.h"
#include "plane.h"
#include "velocity.h"

namespace data {
struct ScenarioWaypoint {
  int id;
  int statusId;
  int time;
  bool is_entrypoint;
  bool is_exit_point;
  double fuel_state;
  std::string weather;
  std::string events;
  GeoPos<double> position;
  Velocity velocity;

  static FlightSegment asFlightSegment(const ScenarioWaypoint &wp) {
    return FlightSegment{wp.position, wp.velocity, false};
  }
  FlightSegment asFlightSegment() const {
    return ScenarioWaypoint::asFlightSegment(*this);
  }
};

void to_json(json &j, const ScenarioWaypoint &p);
void from_json(const json &j, ScenarioWaypoint &p);

// Struct to represent flight data
struct ScenarioFlight {
  // PlaneInfo info;
  PlaneData planeData;
  std::string type_of_aircraft;
  std::string registrationnumber;
  std::string destination;
  std::string alternative_airports;
  std::string weather;
  std::vector<ScenarioWaypoint> waypoints;

  static FlightPlan asFlightPlan(const ScenarioFlight &sf) {
    FlightPlan plan;
    for (const auto &waypoint : sf.waypoints) {
      plan.route.push_front(waypoint.asFlightSegment());
    }
    return plan;
  }
  FlightPlan asFlightPlan() const {
    return ScenarioFlight::asFlightPlan(*this);
  }
};

void to_json(json &j, const ScenarioFlight &p);
void from_json(const json &j, ScenarioFlight &p);

// Struct to represent the entire scenario
struct Scenario {
  std::vector<ScenarioFlight> flights;

  std::vector<Plane> getPlanes() {
    std::vector<Plane> planes;
    planes.reserve(flights.size());
    for (const auto &flight : flights) {
      // TODO: create PlaneConfig based on the type_of_aircraft
      std::shared_ptr<const PlaneConfig> configPtr =
          std::make_shared<const PlaneConfig>(PlaneConfig{
              60.5, 241.9, 12000, 25, 20, 1.35, 2.56, 70, 30, 1000});

      planes.emplace_back(
          Plane(flight.planeData, flight.asFlightPlan(), configPtr));
    }
    return planes;
  }
};

void to_json(json &j, const Scenario &p);
void from_json(const json &j, Scenario &p);

} // namespace data
