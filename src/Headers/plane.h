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
#include <controlParam.h>

enum class MODE { AUTO, HDG, AUX, GROUNDED, PLAYER };

struct PlaneInfo {
  int id;
  int sim_id;
  bool isGrounded;
  double fuel;
  std::string airline;
  std::string flightNumber;
  std::string planeNumber;
  std::string callSign;
  std::string squawk;
  std::string model;
};

// TO DO:
//  fix advanced turning/pathfinding
//  landing, takoff, airport circle
//  ground behavior
//  finish order processing

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
  double fuel;
  Velocity vel;
  GeoPos<double> pos;
  std::vector<GeoPos<double>> targets;
};

void to_json(json& j, const PlaneFlightData& p);
void from_json(const json& j, PlaneFlightData& p);
} // namespace data

class Plane {
  bool _vaildPathFound = true;
  GeoPos<double> _interPos;

public:
  std::string uuid{""};
  PlaneInfo _info;
  Velocity _vel;
  GeoPos<double> _pos;

  MODE _mode;
  FlightSegment _target;
  double setClimbSpeed;

  controlParam _auxParam;
  bool _declaredEmergency = false;


  FlightPlan _flightPlan;
  std::shared_ptr<const PlaneConfig> config;

  void setData(const data::PlaneData &pd);
  data::PlaneData getData() const;

  void setFlightData(const data::PlaneFlightData &pd);
  data::PlaneFlightData getFlightData() const;

  Plane(const data::PlaneData &data, const FlightPlan &flightplan,
        std::shared_ptr<const PlaneConfig> configPointer);

  void update(float timeDelta);

  void generateLandingWaypoints(bool orientation, double slopeAngle,
                                double distance );

  // order handling
  void setAltitude(float altitude);
  void setHeadpoint(GeoPos<double> point);
  void setHeading(float heading);
  void setVelocity(float vel);
  void setSquawk(const std::string &sq);
  void followFlightPlan();
  void enterHolding();
  void landing();
  void touchAndGo();
  void enterAirportLoop();
  // order handling

  void setModePlayer();

private:
  void updateVelocity(float timeDelta);
  void updatePosition(float timeDelta);
  void updateFlightPlan(bool force = false, double margin = 25);
  void updateParameters(float timeDelta);
  void updateGroundBehavior(float timeDelta);

  double getTurnFactor();
  double findHeadingDelta(GeoPos<double> pos, GeoPos<double> targetPos);
  double getTurnRadius();
  bool checkMinRadius();
  void generateHelperWaypoints(FlightSegment targetSegment);
  void setAuxParam();

  void addWaypoint(FlightSegment segment, bool toFront = false);
  void setModeHdg();
  void setModeAux();
  void setModeAuto();
  
};
