#pragma once
#include "common.h"
#include "convertions.h"
#include "flightPlan.h"
#include "geoPos.h"
#include "planeConfig.h"
#include "vec.h"
#include "velocity.h"
#include <atomic>
#include <cmath>

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

// TO DO:
//  Advanced turning/pathfinding
//  landing, takoff, airport circle
//  ground behavior
//  order processing

namespace data {
struct PlaneData {
  PlaneInfo info;
  Velocity vel;
  GeoPos<double> pos;
  GeoPos<double> targetPos;
};

void to_json(json &j, const PlaneData &p);

void from_json(const json &j, PlaneData &p);

struct PlaneFlightData {
  int id;
  std::string squawk;
  Velocity vel;
  GeoPos<double> pos;
  GeoPos<double> targetPos;
};

// to_json function
void to_json(json &j, const PlaneFlightData &p);

// from_json function
void from_json(const json &j, PlaneFlightData &p);
} // namespace data

class Plane {
  bool vaildPathFound = true;

public:
  std::string uuid{""};
  PlaneInfo info;
  Velocity vel;
  GeoPos<double> pos;

  FlightSegment target;
  double setClimbSpeed;

  bool declaredEmergency = false;
  bool ignoreFlightPlan = false;
  FlightPlan flightPlan;
  std::unique_ptr<const PlaneConfig> config;

  void setData(const data::PlaneData &pd);
  data::PlaneData getData() const;

  void setFlightData(const data::PlaneFlightData &pd);
  data::PlaneFlightData getFlightData() const;

  Plane(const data::PlaneData &data, const FlightPlan &flightplan,
        std::unique_ptr<const PlaneConfig> configPointer);

  void update(float timeDelta);
  void updateVelocity(float timeDelta);
  void updatePosition(float timeDelta);
  void updateFlightPlan(bool force = false, double margin = 50);

private:
  double getTurnFactor();
  double findHeadingDelta(GeoPos<double> pos, GeoPos<double> targetPos);
  bool checkMinRadius();
  void generateHelperWaypoints(FlightSegment targetSegment);
};
