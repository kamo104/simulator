#pragma once
#include "common.h"
#include "geoPos.h"
#include "vec.h"
#include "velocity.h"
#include "flightPlan.h"
#include "convertions.h"
#include "planeConfig.h"
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

//TO DO:
// Advanced turning/pathfinding
// landing, takoff, airport circle
// ground behavior
// order processing

class Plane {
public:
  PlaneInfo info;
  Velocity vel;
  GeoPos<double> pos;
  
  Velocity targetVel;
  GeoPos<double> targetPos;
  double setClimbSpeed;

  bool declaredEmergency;
  bool ignoreFlightPlan;
  FlightPlan flightPlan;
  const PlaneConfig* config;


  Plane(PlaneInfo planeInfo, Velocity vel, GeoPos<double> pos, FlightPlan flightplan, PlaneConfig* configPointer) {
    this->info = planeInfo;
    this->vel = vel;
    this->vel.value = std::min(std::max(vel.value, configPointer->minSpeed), configPointer->maxSpeed);
    this->pos = pos;
    this->pos.alt() = std::min(std::max(pos.alt(), 0.0), configPointer->maxAltitude);
    this->flightPlan = flightplan;
    this->ignoreFlightPlan = false;
    this->config = configPointer;
    setClimbSpeed = config->deafultClimbingSpeed;
    
    updateFlightPlan(true);
  }

  void update(float timeDelta) {
    //Debug
    std::cout << info.callSign << " target: "<< targetPos <<" dist: "<< distance(pos, targetPos) << std::endl;
    std::cout << "Hdg: " << rad2dgr(vel.heading) << " Speed: "<< vel.value << std::endl;
    std::cout << "Pos: " << pos << std::endl <<std::endl;
    updateVelocity(timeDelta);
    updatePosition(timeDelta);
    updateFlightPlan();
  }

  void updateVelocity(float timeDelta) {
    double velDelta = targetVel.value - vel.value;
    vel.value += sgn(velDelta) * std::min(std::abs(velDelta), std::pow(config->accelerationFactor, 2) / vel.value) * timeDelta;
    vel.value = std::min(std::max(vel.value, config->minSpeed), config->maxSpeed);

    //simplest turning 
    double targetHeading = fixAngle(std::atan2(targetPos.lat() - pos.lat(), targetPos.lon() - pos.lon()));
    double hdgDelta = targetHeading - vel.heading;
    vel.heading += sgn(hdgDelta) * std::min(std::abs(hdgDelta), getTurnFactor() * timeDelta) ;
    vel.heading = fixAngle(vel.heading); 
  }

  void updatePosition(float timeDelta) {
    pos.lat() += std::sin(vel.heading) * meter2lat(vel.value) * timeDelta;
    pos.lon() += -std::cos(vel.heading) * meter2long(vel.value, pos.lat()) * timeDelta;

    double altitudeDelta = targetPos.alt() - pos.alt();
    pos.alt() += sgn(altitudeDelta) * std::min(std::abs(altitudeDelta), setClimbSpeed * timeDelta);
    pos.alt() = std::max(std::min(pos.alt(), config->maxAltitude),0.0);
  }

  void updateFlightPlan(bool force = false, double margin = 0.02) {
    if (force ||  distance(pos, targetPos) < margin) {
        if (flightPlan.route.size() == 0 && !info.isGrounded) {
            ignoreFlightPlan = true;
            return;
        }
        std::cout << "Changing Target "<< std::endl;
        targetPos = flightPlan.route.front().targetPos;
        targetVel = flightPlan.route.front().targetVel;
        flightPlan.route.erase(flightPlan.route.begin());
    }
  }
private:
  double getTurnFactor() {
      double n = (declaredEmergency) ? config->maxLoad : config->normalLoad;
      return 9.81 / (vel.value * std::sqrt(std::pow(n, 2) - 1));
  }
};
