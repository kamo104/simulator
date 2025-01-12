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

enum class FlightState { AUTO, HDG, AUX, GROUNDED };

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
};

void to_json(json &j, const PlaneData &p);
void from_json(const json &j, PlaneData &p);


struct PlaneFlightData {
  int id;
  std::string squawk;
  Velocity vel;
  GeoPos<double> pos;
  std::vector<GeoPos<double>> targets;
};

void to_json(json& j, const PlaneFlightData& p);
void from_json(const json& j, PlaneFlightData& p);
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
  std::shared_ptr<const PlaneConfig> config;

  void setData(const data::PlaneData &pd);
  data::PlaneData getData() const;

  void setFlightData(const data::PlaneFlightData &pd);
  data::PlaneFlightData getFlightData() const;

  Plane(const data::PlaneData &data, const FlightPlan &flightplan,
        std::shared_ptr<const PlaneConfig> configPointer);

  void update(float timeDelta);

  void generateLandingWaypoints(bool orientation, double slopeAngle = 3,
                                double distance = 5000);

  // order handling
  void setAltitude(float altitude);
  void setHeadpoint(Vec<double, 2> point);
  void setHeading(float heading);
  void setVelocity(float vel);
  void setSquawk(const std::string &sq);
  void followFlightPlan();
  void enterHolding();
  void landing();
  void touchAndGo();
  void enterAirportLoop();
  // order handling

private:
  void updateVelocity(float timeDelta);
  void updatePosition(float timeDelta);
  void updateFlightPlan(bool force = false, double margin = 10);
  double getTurnFactor();
  double findHeadingDelta(GeoPos<double> pos, GeoPos<double> targetPos);
  bool checkMinRadius();
  void generateHelperWaypoints(FlightSegment targetSegment);
};
