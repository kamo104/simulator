#include "plane.h"
#include "advanced_ai.h"

namespace data {

void to_json(json &j, const PlaneData &p) {
    j = json{ {"id", p.info.id},
             {"isGrounded", p.info.isGrounded},
             {"airline", p.info.airline},
             {"flight_number", p.info.flightNumber},
             {"plane_number", p.info.planeNumber},
             {"callsign", p.info.callSign},
             {"squawk", p.info.squawk},
             {"model", p.info.model},
             {"fuel", p.info.fuel},
             {"velocity", {{"direction", p.vel.heading}, {"value", p.vel.value}}},
             {"position",
              {{"latitude", p.pos.lat()},
               {"longitude", p.pos.lon()},
               {"altitude", p.pos.alt()}}}
    };
}

void from_json(const json &j, PlaneData &p) {
  const auto &info = j.at("info");
  info.at("id").get_to(p.info.id);
  info.at("isGrounded").get_to(p.info.isGrounded);
  info.at("airline").get_to(p.info.airline);
  info.at("flightNumber").get_to(p.info.flightNumber);
  info.at("planeNumber").get_to(p.info.planeNumber);
  info.at("callSign").get_to(p.info.callSign);
  info.at("squawk").get_to(p.info.squawk);
  info.at("model").get_to(p.info.model);

  const auto &vel = j.at("velocity");
  vel.at("direction").get_to(p.vel.heading);
  vel.at("value").get_to(p.vel.value);

  const auto &pos = j.at("position");
  pos.at("latitude").get_to(p.pos.lat());
  pos.at("longitude").get_to(p.pos.lon());
  pos.at("altitude").get_to(p.pos.alt());
}

// to_json function
void to_json(json& j, const PlaneFlightData& p) {
    j = json{ {"id", p.id},
             {"squawk", p.squawk},
             {"velocity",
              {
                  {"direction", p.vel.heading},
                  {"value", p.vel.value},
              }},
             {"position",
              {{"latitude", p.pos.lat()},
               {"longitude", p.pos.lon()},
               {"altitude", p.pos.alt()}}},
             {"targets", json::array()} };

    for (const auto& target : p.targets) {
        j["targets"].push_back({
            {"latitude", target.lat()},
            {"longitude", target.lon()},
            {"altitude", target.alt()}
        });
    }
}

// from_json function
void from_json(const json& j, PlaneFlightData& p) {
    j.at("squawk").get_to(p.squawk);
    j.at("id").get_to(p.id);

    const auto& vel = j.at("velocity");
    vel.at("direction").get_to(p.vel.heading);
    vel.at("value").get_to(p.vel.value);

    const auto& pos = j.at("position");
    pos.at("latitude").get_to(p.pos.lat());
    pos.at("longitude").get_to(p.pos.lon());
    pos.at("altitude").get_to(p.pos.alt());
}
} // namespace data

void Plane::setData(const data::PlaneData &pd) {
  // TODO: maybe implement a better function of setting the data
  this->_info = pd.info;
  this->_vel = pd.vel;
  this->_pos = pd.pos;
  // data::PlaneFlightData pd2;
}
data::PlaneData Plane::getData() const {
  return data::PlaneData{this->_info,
                         {ms2kts(this->_vel.value), -this->_vel.heading},
                         xy2geo(this->_pos)}; // coordinate conversion
}

void Plane::setFlightData(const data::PlaneFlightData &pd) {
  // TODO: maybe implement a better function of setting the data
  this->_pos = geo2xy(pd.pos);
  this->_vel.heading = pd.vel.heading;
  this->_vel.value = kts2ms(pd.vel.value);
  //this->_target.pos = geo2xy(pd.targets.front());
}
data::PlaneFlightData Plane::getFlightData() const {
    std::vector<GeoPos<double>> targets;

    // this is a heavy implementation, shoud be moved to a seperate msg that is sent only on
    // target updates
    
    targets.reserve(_flightPlan.route.size() + _flightPlan.auxiliary.size() + 1);
    if (mode != MODE::HDG && mode != MODE::PLAYER) targets.emplace_back(xy2geo(_target.pos));
    if (mode != MODE::HDG && mode != MODE::PLAYER) {
      for (auto& seg : _flightPlan.auxiliary) targets.emplace_back(xy2geo(seg.pos));
    }
    if (mode == MODE::AUTO) {
      for (auto& seg : _flightPlan.route) targets.emplace_back(xy2geo(seg.pos));
    }

    /*for (auto trg : targets) std::cout << trg << std::endl;
    std::cout << std::endl;*/
    return data::PlaneFlightData{ this->_info.id, this->_info.squawk, this->_info.fuel,
                                {ms2kts(this->_vel.value), -this->_vel.heading},
                                 xy2geo(this->_pos), targets };
}


Plane::Plane(const data::PlaneData &data, const FlightPlan &flightplan,
             std::shared_ptr<const PlaneConfig> configPointer) {
  this->_info = data.info;
  this->_vel = data.vel;
  this->_vel.value = std::min(std::max(kts2ms(_vel.value), configPointer->minSpeed),
                             configPointer->maxSpeed);
  this->_pos = geo2xy(data.pos);
  this->_pos.alt() =
      std::min(std::max(_pos.alt(), 0.0), configPointer->maxAltitude);
  this->_flightPlan = flightplan;

  this->mode = MODE::AUTO;
  this->grdMode = GRD_MODE::NONE;

  if (data.info.isGrounded) {
    this->mode = MODE::GRD;
    this->grdMode = GRD_MODE::IDLE;
  }

  this->config = configPointer;

  this->_auxParam.overwriteVel = false;
  this->_auxParam.overwriteAlt = false;
  this->_auxParam.altChange = config->deafultClimbingSpeed;

  if (_info.isGrounded) {
      mode = MODE::GRD;
  }


  updateFlightPlan(true);
}

void Plane::update(float timeDelta) {
  //Debug
  //std::cout << _info.callSign << " target: " << xy2geo(_target.pos)
  //        << " dist: " << distance(_pos, _target.pos) << std::endl;
  //std::cout << "Hdg: " << rad2hdg(_vel.heading) << " Speed: " << _vel.value
  //        << std::endl;
  //std::cout << "Pos(GEO): " << xy2geo(_pos) << std::endl;
  //std::cout << "Pos (XY): " << _pos << std::endl << std::endl;
  if (mode == MODE::PLAYER || grdMode == GRD_MODE::IDLE) return;
  
  updateFlightPlan();
  updateParameters(timeDelta);
  updatePosition(timeDelta);
  updateVelocity(timeDelta);
}

void Plane::updateVelocity(float timeDelta) {
  // Velcoity
  double velDelta = getTrgVel() - _vel.value;
  _vel.value += sgn(velDelta) *
               std::min(std::abs(velDelta),
                        std::pow(config->accelerationFactor, 2) 
                        / _vel.value * timeDelta);
  if(mode != MODE::GRD) _vel.value = std::min(std::max(_vel.value, config->minSpeed), config->maxSpeed);

  // Heading
  double hdgDelta = findHeadingDelta(_pos, _target.pos);
  _vel.heading +=
      sgn(hdgDelta) * std::min(std::abs(hdgDelta), getTurnFactor() * timeDelta);
  _vel.heading = fixAngle(_vel.heading);
}

void Plane::updatePosition(float timeDelta) {
  // XY Position
  _pos.lat() += std::sin(_vel.heading) * _vel.value * timeDelta;
  _pos.lon() += std::cos(_vel.heading) * _vel.value * timeDelta;

  // Altitude
  double altitudeDelta = getTrgAlt() - _pos.alt();

  _pos.alt() += sgn(altitudeDelta) *
    std::min(std::abs(altitudeDelta), _auxParam.altChange * timeDelta);
  
  _pos.alt() = std::max(std::min(_pos.alt(), config->maxAltitude), 0.0);
}

void Plane::updateFlightPlan(bool force, double margin) {
  if (mode == MODE::HDG) return;

  if (mode == MODE::GRD) margin = 5;

  if (force || distance(_pos, _target.pos) < margin) {
    if ((_flightPlan.route.size() == 0 && _flightPlan.auxiliary.size() == 0) 
        || (mode == MODE::AUX && _flightPlan.auxiliary.size() == 0)) {
      setModeHdg();
      return;
    }

    FlightSegment next;
    if (_flightPlan.auxiliary.size()) {
      next = _flightPlan.auxiliary.front();
      _flightPlan.auxiliary.pop_front();
    } else {
      next = _flightPlan.route.front();
      _flightPlan.route.pop_front();
    }

    if (next.interpolateAlt || next.interpolateVel) {
      _flightPlan.interTrg = {_pos, _vel};
    }

    if (next.useHeading) {
      std::cout << "Changing Target " << next.pos <<std::endl;
      _flightPlan.vaildPathFound = false;
      generateHelperWaypoints(next);
    } else {
      addWaypoint(next, true);
      this->_target = next;

    }
  }
}

void Plane::updateParameters(float timeDelta) {
  if (mode != MODE::GRD) {
    _info.fuel -= config->fuelConsumption * timeDelta;
  }

  if (_pos.alt() <= 0.1) {
    mode = MODE::GRD;
  }
}

double Plane::getTrgVel() {
  if (_auxParam.overwriteVel || mode == MODE::HDG) {
    return _auxParam.vel.value;
  }
  if (_target.interpolateVel) {
    return _flightPlan.interTrg.vel.value + (_target.vel.value - _flightPlan.interTrg.vel.value) /
      distance(_target.pos, _flightPlan.interTrg.pos) * distance(_pos, _flightPlan.interTrg.pos);
  }
  return _target.vel.value;
}

double Plane::getTrgAlt() {
  if (_auxParam.overwriteAlt || mode == MODE::HDG) {
    return _auxParam.alt;
  }
  if (_target.interpolateAlt) {
    return _flightPlan.interTrg.pos.alt() + (_target.pos.alt() - _flightPlan.interTrg.pos.alt()) /
      distance(_target.pos, _flightPlan.interTrg.pos) * distance(_pos, _flightPlan.interTrg.pos);
  }
  return _target.pos.alt();
}

void Plane::setModeHdg() {
  if(mode == MODE::GRD) {
    _auxParam.overwriteVel = true;
    _auxParam.vel.value = 0;
    if (grdMode == GRD_MODE::TAXI_OUT) {
      grdMode = GRD_MODE::IDLE;
    } else if(grdMode == GRD_MODE::TAXI_IN) {
      grdMode = GRD_MODE::HOLD_RWY;
    } else if (grdMode == GRD_MODE::HOLD_RWY) {
      grdMode = GRD_MODE::TAKEOFF;
    }
    return;
  }
  if(mode != MODE::HDG) setAuxParam();
  mode = MODE::HDG;
}

void Plane::setModeAux() {
  mode = MODE::AUX;
  _flightPlan.auxiliary.clear();
  setAuxParam();
}

void Plane::setModeAuto() {
  _flightPlan.auxiliary.clear();
  mode = MODE::AUTO;
}

void Plane::setModePlayer() {
    mode = MODE::PLAYER;
}

double Plane::getTurnFactor() {
  if (mode == MODE::GRD) return _vel.value/30;

  double n = (_declaredEmergency) ? config->maxLoad : config->normalLoad;
  return 9.81 / _vel.value * std::sqrt(std::pow(n, 2) - 1);
}

double Plane::findHeadingDelta(GeoPos<double> pos, GeoPos<double> targetPos) {
  double targetHeading;
  if (mode == MODE::HDG) targetHeading = _auxParam.vel.heading;
  else {
    targetHeading = fixAngle(
        std::atan2(targetPos.lat() - pos.lat(), targetPos.lon() - pos.lon()));
  }

  // Check if turn is possible
 /* if (_mode != MODE::HDG && checkMinRadius())
    return 0;*/

  // Check if advanced pathfinding requires repositioning
  if (mode != MODE::HDG && !_flightPlan.vaildPathFound) {
    generateHelperWaypoints(_target);
    return 0;
  }

  double hdgDelta = targetHeading - _vel.heading;
  double altDelta = (2 * PI - std::abs(hdgDelta)) * -sgn(hdgDelta);
  if (std::abs(altDelta) < std::abs(hdgDelta))
    hdgDelta = altDelta;
  return hdgDelta;
}

double Plane::getTurnRadius() {
  double vel = std::max(_vel.value, getTrgVel());
  vel = std::min(std::max(vel, config->minSpeed), config->maxSpeed);
  double n = (_declaredEmergency) ? config->maxLoad : config->normalLoad;
  return 1.1 * std::sqrt(-std::pow(vel, 4) /
    (std::pow(G, 2) * (1 - std::pow(n, 2))));
}

bool Plane::checkMinRadius() {
  double n = (_declaredEmergency) ? config->maxLoad : config->normalLoad;
  double r = getTurnRadius();
  GeoPos<double> A = {{_pos.lat() + std::sin(_vel.heading + PI / 2) * r,
                       _pos.lon() + std::cos(_vel.heading + PI / 2) * r,
                       _target.pos.alt()}};
  GeoPos<double> B = {{_pos.lat() - std::sin(_vel.heading + PI / 2) * r,
                       _pos.lon() - std::cos(_vel.heading + PI / 2) * r,
                       _target.pos.alt()}};
  return (distance(_target.pos, A) < r || distance(_target.pos, B) < r);
}



// Check 4 circle non-overlaping match combinations and find
// coresponding 4 valid tangent lines 
void Plane::generateHelperWaypoints(FlightSegment targetSegment) {
  targetSegment.useHeading = false;
  GeoPos<double> tPos = targetSegment.pos;
  Velocity tVel = targetSegment.vel;

  double r = getTurnRadius();
  
  Waypoint start = { _pos.lon(), _pos.lat(), _vel.heading };
  Waypoint end = { tPos.lon(), tPos.lat(), tVel.heading };

  auto route = generateShortestRoute(start, end, r, std::ceil(r / 25.0));


  addWaypoint({ {{route[0].y, route[0].x, tPos.alt()}}, tVel, false, false}, true);
  for (int i = 1; i < route.size(); i++) {
      addWaypoint({ {{route[i].y, route[i].x, tPos.alt()}}, tVel, false, false });
  }

  this->_vaildPathFound = true;

}

void Plane::generateLandingWaypoints(RUNWAY approach, bool succesful = true,
                              double slopeAngle = 3, double distance = 5000) {

  std::array<FlightSegment, LANDING_SIZE>  choice 
      = (approach == RUNWAY::L10) ? landing10 : landing28;
  FlightSegment first = choice[0];

  double dir = first.vel.heading - PI;
  double alt = distance * std::tan(dgr2rad(slopeAngle));

  GeoPos<double> wpLand = {{first.pos.lat() + sin(dir) * distance,
                              first.pos.lon() + cos(dir) * distance, alt}};
  generateHelperWaypoints({ wpLand, {config->landingSpeed, first.vel.heading}});
  first.vel.value = config->landingSpeed;
  addWaypoint(first);



  dir = first.vel.heading;
  double brakeDist = config->landingDistance;
  GeoPos<double> wpStop = {{first.pos.lat() + sin(dir) * brakeDist,
                               first.pos.lon() + cos(dir) * brakeDist, 0}};

  addWaypoint(FlightSegment{ wpStop, {config->taxiSpeed,0}, false, false, true});

  int exitway = EXIT_START;
  if (approach == RUNWAY::L28 && wpStop.lon() < choice[exitway].pos.lon() ||
      approach == RUNWAY::L10 && wpStop.lon() > choice[exitway].pos.lon()) {
    exitway += EXIT_SIZE;
  };
  for (int i = exitway; i < exitway+EXIT_SIZE; i++) {
    addWaypoint({ choice[i].pos, {config->taxiSpeed, 0} });
  }
  for (int i = EXIT_START+2*EXIT_SIZE; i < LANDING_SIZE-1; i++) {
    addWaypoint({ choice[i].pos, {config->taxiSpeed, 0} });
  }
  addWaypoint({ choice[LANDING_SIZE-1].pos, {0,0}, false, false, true});
}

void Plane::generateTaxiWaypoints(RUNWAY runway) {
  std::array<FlightSegment, TAKEOFF_SIZE> choice 
      = (runway == RUNWAY::L28) ? takeOff28 : takeOff10;

  for (int i = 0; i < TAKEOFF_SIZE; i++) {
    addWaypoint({ choice[i].pos, {config->taxiSpeed, 0} });
  }
}

void Plane::generateRunwayWaypoints() {
  FlightSegment choice 
      = (_flightPlan.setRunway == RUNWAY::L28) ? exitOuter10_1 : exitOuter28_1;

  addWaypoint({ choice.pos, {config->taxiSpeed, 0} });
}

void Plane::generateTakeOffWaypoints() {
  FlightSegment choice
    = (_flightPlan.setRunway == RUNWAY::L28) ? exitOuter10_1 : exitOuter28_1;

  addWaypoint({ choice.pos, {config->taxiSpeed, 0} });
}

void Plane::addWaypoint(FlightSegment segment, bool toFront) {
  if (toFront) {
    _target = segment;
  } else {
    _flightPlan.auxiliary.push_back(segment);
  }
}

void Plane::setAuxParam() {
    _auxParam.vel.heading = _vel.heading;
    if(!_auxParam.overwriteVel) _auxParam.vel.value = _target.vel.value;
    if(!_auxParam.overwriteAlt) _auxParam.alt = _target.pos.alt();
}

// order handling
void Plane::setAltitude(float altitude) {
  if(mode == MODE::GRD) return;

  _auxParam.overwriteAlt = true;
  _auxParam.alt = ft2meter(altitude);
}

void Plane::setHeadpoint(GeoPos<double> point) {
  setModeAux();
  FlightSegment seg = { geo2xy(point), _target.vel, false, false };
  addWaypoint(seg, true);
}

void Plane::setHeading(float heading) {
  setModeHdg();
  _auxParam.vel.heading = hdg2rad(heading);
}

void Plane::setVelocity(float vel) {
  _auxParam.overwriteVel = true;
  double val = kts2ms(vel);
  if (mode == MODE::GRD) {
    val = std::max(std::min(val, config->taxiMaxSpeed), 0.0);
  } else {
    val = std::max(std::min(val, config->maxSpeed), config->minSpeed);
  }
  _auxParam.vel.value = val;
}

void Plane::setSquawk(const std::string &sq) { this->_info.squawk = sq; }

void Plane::followFlightPlan() {
  if (mode != MODE::GRD) {
    _flightPlan.auxiliary.clear();
    setModeAuto();
  }

  _auxParam.overwriteAlt = false;
  _auxParam.overwriteVel = false;
}

void Plane::landing(std::string name) {
  setModeAux();
  if (name == "MAPt 28") {
    generateLandingWaypoints(RUNWAY::L28);
  } else {
    generateLandingWaypoints(RUNWAY::L10);
  }
  grdMode = GRD_MODE::TAXI_OUT;
}
void Plane::touchAndGo(std::string name) {
  setModeAux();
  if (name == "MAPt 28") {
    generateLandingWaypoints(RUNWAY::L28, false);
  } else {
    generateLandingWaypoints(RUNWAY::L10, false);
  }
}
void Plane::taxiToRunway(std::string name) {
  if (grdMode == GRD_MODE::IDLE) {
    if (name == "MAPt 28") {
      generateTaxiWaypoints(RUNWAY::L28);
      _flightPlan.setRunway = RUNWAY::L28;
    } else {
      generateTaxiWaypoints(RUNWAY::L10);
      _flightPlan.setRunway = RUNWAY::L10;
    }
    
    grdMode = GRD_MODE::TAXI_IN;
  }
}
void Plane::enterRunway() {
  if (grdMode == GRD_MODE::HOLD_RWY) {
    generateRunwayWaypoints();
  }
}
void Plane::takeOff() {
  if (grdMode == GRD_MODE::TAKEOFF) {
    setModeAux();
    generateTakeOffWaypoints();
    grdMode = GRD_MODE::NONE;
  }
}


void Plane::enterAirportLoop() {}
void Plane::enterHolding() {}
// order handling
