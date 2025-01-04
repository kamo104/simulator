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
};

inline void to_json(json &j, const PlaneData &p) {
    
  j = json{{{"id", p.info.id},
            {"sim_id", p.info.sim_id},
            {"isGrounded", p.info.isGrounded},
            {"airline", p.info.airline},
            {"flightNumber", p.info.flightNumber},
            {"planeNumber", p.info.planeNumber},
            {"callSign", p.info.callSign},
            {"squawk", p.info.squawk},
            {"model", p.info.model}},
           {"velocity",
            {
                {"heading", fixAngle(p.vel.heading-PI/2)}, //radian conversion
                {"value", ms2kts(p.vel.value)}, // m/s to kts
            }},
           {"position",
            {{"latitude", p.pos.lat()},
             {"longitude", p.pos.lon()},
             {"altitude", p.pos.alt()}}}};
}

inline void from_json(const json &j, PlaneData &p) {
  const auto &info = j.at("info");
  info.at("id").get_to(p.info.id);
  info.at("sim_id").get_to(p.info.sim_id);
  info.at("isGrounded").get_to(p.info.isGrounded);
  info.at("airline").get_to(p.info.airline);
  info.at("flightNumber").get_to(p.info.flightNumber);
  info.at("planeNumber").get_to(p.info.planeNumber);
  info.at("callSign").get_to(p.info.callSign);
  info.at("squawk").get_to(p.info.squawk);
  info.at("model").get_to(p.info.model);

  const auto &vel = j.at("velocity");
  vel.at("heading").get_to(p.vel.heading);
  vel.at("value").get_to(p.vel.value);

  const auto &pos = j.at("position");
  pos.at("latitude").get_to(p.pos.lat());
  pos.at("longitude").get_to(p.pos.lon());
  pos.at("altitude").get_to(p.pos.alt());
}

struct PlaneFlightData {
  int id;
  std::string squawk;
  Velocity vel;
  GeoPos<double> pos;
  GeoPos<double> targetPos;
};

// to_json function
inline void to_json(json &j, const PlaneFlightData &p) {
  j = json{{"id", p.id},
           {"squawk", p.squawk},
           {"velocity",
            {
                {"heading", p.vel.heading},
                {"value", p.vel.value},
            }},
           {"position",
            {{"latitude", p.pos.lat()},
             {"longitude", p.pos.lon()},
             {"altitude", p.pos.alt()}}},
           {"targetPosition",
            {{"latitude", p.targetPos.lat()},
             {"longitude", p.targetPos.lon()},
             {"altitude", p.targetPos.alt()}}}};
}

// from_json function
inline void from_json(const json &j, PlaneFlightData &p) {
  j.at("squawk").get_to(p.squawk);
  j.at("id").get_to(p.id);

  const auto &vel = j.at("velocity");
  vel.at("heading").get_to(p.vel.heading);
  vel.at("value").get_to(p.vel.value);

  const auto &pos = j.at("position");
  pos.at("latitude").get_to(p.pos.lat());
  pos.at("longitude").get_to(p.pos.lon());
  pos.at("altitude").get_to(p.pos.alt());

  const auto &targetPos = j.at("targetPosition");
  targetPos.at("latitude").get_to(p.targetPos.lat());
  targetPos.at("longitude").get_to(p.targetPos.lon());
  targetPos.at("altitude").get_to(p.targetPos.alt());
}
} // namespace data

class Plane {
  bool vaildPathFound = true;
public:
  std::string uuid{""};
  PlaneInfo info;
  Velocity vel;
  GeoPos<double> pos;

  Velocity targetVel;
  GeoPos<double> targetPos;
  double setClimbSpeed;

  bool declaredEmergency = false;
  bool ignoreFlightPlan = false;
  FlightPlan flightPlan;
  std::unique_ptr<const PlaneConfig> config;

  void setData(const data::PlaneData &pd) {
    // TODO: implement a better function of setting the data
    this->info = pd.info;
    this->vel = pd.vel;
    this->pos = pd.pos;
  }
  data::PlaneData getData() {
    return data::PlaneData{this->info, this->vel, xy2geo(this->pos)}; //coordinate conversion
  }

  void setFlightData(const data::PlaneFlightData &pd) {
    // TODO: maybe implement a better function of setting the data
    this->pos = pd.pos;
    this->vel = pd.vel;
    this->targetPos = pd.targetPos;
  }
  data::PlaneFlightData getFlightData() const {
    return data::PlaneFlightData{this->info.id, this->info.squawk, this->vel,
                                 this->pos};
  }

  Plane(const data::PlaneData &data, const FlightPlan &flightplan,
        std::unique_ptr<const PlaneConfig> configPointer) {
    this->info = data.info;
    this->vel = data.vel;
    this->vel.value = std::min(std::max(vel.value, configPointer->minSpeed),
                               configPointer->maxSpeed);
    this->pos = geo2xy(data.pos);
    this->pos.alt() =
        std::min(std::max(pos.alt(), 0.0), configPointer->maxAltitude);
    this->flightPlan = flightplan;

    this->config = std::move(configPointer);
    setClimbSpeed = config->deafultClimbingSpeed;

    updateFlightPlan(true);
  }

  void update(float timeDelta) {
    // Debug
    std::cout << info.callSign << " target: " << xy2geo(targetPos)
              << " dist: " << distance(pos, targetPos) << std::endl;
    std::cout << "Hdg: " << rad2dgr(vel.heading) << " Speed: " << vel.value
              << std::endl;
    std::cout << "Pos(GEO): " << xy2geo(pos) << std::endl;
    std::cout << "Pos (XY): " << pos << std::endl << std::endl;
    updateVelocity(timeDelta);
    updatePosition(timeDelta);
    updateFlightPlan();
  }

  void updateVelocity(float timeDelta) {
    double velDelta = targetVel.value - vel.value;
    vel.value += sgn(velDelta) *
                 std::min(std::abs(velDelta),
                          std::pow(config->accelerationFactor, 2) / vel.value) *  timeDelta;
    vel.value =
        std::min(std::max(vel.value, config->minSpeed), config->maxSpeed);

    double hdgDelta = findHeadingDelta(pos, targetPos);
    vel.heading += sgn(hdgDelta) *
                   std::min(std::abs(hdgDelta), getTurnFactor() * timeDelta);
    vel.heading = fixAngle(vel.heading);
  }

  void updatePosition(float timeDelta) {
    pos.lat() += std::sin(vel.heading) * vel.value * timeDelta;
    pos.lon() += std::cos(vel.heading) * vel.value * timeDelta;
    double altitudeDelta = targetPos.alt() - pos.alt();
    pos.alt() += sgn(altitudeDelta) *
                 std::min(std::abs(altitudeDelta), setClimbSpeed * timeDelta);
    pos.alt() = std::max(std::min(pos.alt(), config->maxAltitude), 0.0);
  }

  void updateFlightPlan(bool force = false, double margin = 50) {
    if (force || distance(pos, targetPos) < margin) {
      if (flightPlan.route.size() == 0 && !info.isGrounded) {
        ignoreFlightPlan = true;
        return;
      }
      std::cout << "Changing Target " << std::endl;
      targetPos = geo2xy(flightPlan.route.front().targetPos);
      targetVel = flightPlan.route.front().targetVel;
      if (!flightPlan.route.front().ignoreHeading) {
          vaildPathFound = false;
          generateHelperWaypoints();
      }
      flightPlan.route.erase(flightPlan.route.begin());

    }
  }

private:
  double getTurnFactor() {
    double n = (declaredEmergency) ? config->maxLoad : config->normalLoad;
    return 9.81 / (vel.value * std::sqrt(std::pow(n, 2) - 1));
  }

  double findHeadingDelta(GeoPos<double> pos, GeoPos<double> targetPos) {
    double targetHeading = fixAngle(std::atan2(targetPos.lat() - pos.lat(), 
                                               targetPos.lon() - pos.lon()));
    //std::cout << "Target Hdg: " << rad2dgr(targetHeading) << std::endl;

    //Check if turn is possible 
    if (ignoreFlightPlan || checkMinRadius())  return 0;
    
    //Check if advanced pathfinding requires repositioning 
    if (!vaildPathFound) {
        generateHelperWaypoints();
        return 0;
    }

    double hdgDelta = targetHeading - vel.heading;
    double altDelta = (2 * PI - std::abs(hdgDelta)) * -sgn(hdgDelta);
    if (std::abs(altDelta) < std::abs(hdgDelta)) hdgDelta = altDelta;
    return hdgDelta;
  }

  bool checkMinRadius() {
    double n = (declaredEmergency) ? config->maxLoad : config->normalLoad;
    double r = std::sqrt(-std::pow(targetVel.value, 4) / (std::pow(G, 2) * (1 - std::pow(n, 2))));
    GeoPos<double> A = { {pos.lat() + std::sin(vel.heading + PI / 2) * r, 
                          pos.lon() + std::cos(vel.heading + PI / 2) * r, targetPos.alt()}};
    GeoPos<double> B = { {pos.lat() - std::sin(vel.heading + PI / 2) * r,
                          pos.lon() - std::cos(vel.heading + PI / 2) * r, targetPos.alt()}};
    return (distance(targetPos, A) < r || distance(targetPos, B) < r);
  }

  void generateHelperWaypoints() {
      //TO DO: check 4 circle non-overlaping match combinations and find coresponding 4 valid tangent lines
      //if match exist stop ignoring heading delta -> vaildPathFound = true;
  }



};
