#include "plane.h"

namespace data {

void to_json(json &j, const PlaneData &p) {
    j = json{ {"id", p.info.id},
             {"sim_id", p.info.sim_id},
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
  info.at("sim_id").get_to(p.info.sim_id);
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

    /*for (const auto& target : j.at("targets")) {
        double lat, lon, alt;
        target.at("latitude").get_to(lat);
        target.at("longitude").get_to(lon);
        target.at("altitude").get_to(alt);
        p.targets.push_back({ { lat, lon, alt } });
    }*/
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
    if (_mode != MODE::HDG) targets.emplace_back(xy2geo(_target.pos));
    if (_mode != MODE::HDG) {
      for (auto& seg : _flightPlan.auxiliary) targets.emplace_back(xy2geo(seg.pos));
    }
    if (_mode == MODE::AUTO) {
      for (auto& seg : _flightPlan.route) targets.emplace_back(xy2geo(seg.pos));
    }
    return data::PlaneFlightData{ this->_info.id, this->_info.squawk, this->_info.fuel,
                                {ms2kts(this->_vel.value), -this->_vel.heading},
                                 xy2geo(this->_pos), targets };
}


Plane::Plane(const data::PlaneData &data, const FlightPlan &flightplan,
             std::shared_ptr<const PlaneConfig> configPointer) {
  this->_info = data.info;
  this->_vel = data.vel;
  this->_vel.value = std::min(std::max(_vel.value, configPointer->minSpeed),
                             configPointer->maxSpeed);
  this->_pos = geo2xy(data.pos);
  this->_pos.alt() =
      std::min(std::max(_pos.alt(), 0.0), configPointer->maxAltitude);
  this->_flightPlan = flightplan;
  this->_mode = MODE::AUTO;

  this->config = configPointer;
  setClimbSpeed = config->deafultClimbingSpeed;

  this->_auxParam.altMode = ALT_MODE::AUTO;

  if (_info.isGrounded) {
      _mode = MODE::GROUNDED;
  }


  updateFlightPlan(true);
}

void Plane::update(float timeDelta) {
  //Debug
  //std::cout << static_cast<std::underlying_type<MODE>::type>(_mode) << std::endl;
  //std::cout << _info.callSign << " target: " << xy2geo(_target.pos)
  //        << " dist: " << distance(_pos, _target.pos) << std::endl;
  //std::cout << "Hdg: " << rad2hdg(_vel.heading) << " Speed: " << _vel.value
  //        << std::endl;
  //std::cout << "Pos(GEO): " << xy2geo(_pos) << std::endl;
  //std::cout << "Pos (XY): " << _pos << std::endl << std::endl;
  if (_mode == MODE::PLAYER) {
      return;
  }

  if (_mode == MODE::GROUNDED) {
    updateGroundBehavior(timeDelta);
  }
  else {
    updateFlightPlan();
  }

  updateParameters(timeDelta);
  updatePosition(timeDelta);
  updateVelocity(timeDelta);
}

void Plane::updateVelocity(float timeDelta) {
  double targetVel = (_mode == MODE::AUTO) ? _target.vel.value : _auxParam.vel.value;

  double velDelta = targetVel - _vel.value;
  _vel.value += sgn(velDelta) *
               std::min(std::abs(velDelta),
                        std::pow(config->accelerationFactor, 2) / _vel.value)
                        * timeDelta;
  if(_mode != MODE::GROUNDED) _vel.value = std::min(std::max(_vel.value, config->minSpeed), config->maxSpeed);

  double hdgDelta = findHeadingDelta(_pos, _target.pos);
  _vel.heading +=
      sgn(hdgDelta) * std::min(std::abs(hdgDelta), getTurnFactor() * timeDelta);
  _vel.heading = fixAngle(_vel.heading);
}

void Plane::updatePosition(float timeDelta) {
  // Position
  _pos.lat() += std::sin(_vel.heading) * _vel.value * timeDelta;
  _pos.lon() += std::cos(_vel.heading) * _vel.value * timeDelta;


  // Altitude logic & Update
  double targetAlt = (_mode == MODE::AUTO) ? _target.pos.alt() : _auxParam.alt;
  if (_mode != MODE::HDG && _target.interpolateAlt) {
    targetAlt = _interPos.alt() + (_target.pos.alt() - _interPos.alt()) /
                distance(_target.pos, _interPos) * distance(_target.pos, _pos);
  }
  
  double altitudeDelta = targetAlt - _pos.alt();

  double climbSpeed = setClimbSpeed;
  if (_auxParam.altMode == ALT_MODE::CHANGE || _auxParam.altMode == ALT_MODE::SET_CHANGE) {
    climbSpeed = _auxParam.altChange;
  }

  if (_auxParam.altMode == ALT_MODE::CHANGE) {
    _pos.alt() += climbSpeed;
  } else {
    _pos.alt() += sgn(altitudeDelta) *
      std::min(std::abs(altitudeDelta), climbSpeed * timeDelta);
  }
  
  _pos.alt() = std::max(std::min(_pos.alt(), config->maxAltitude), 0.0);
}

void Plane::updateFlightPlan(bool force, double margin) {
  if (_mode == MODE::HDG) return;

  if (force || distance(_pos, _target.pos) < margin) {
    if ((_flightPlan.route.size() == 0 && _flightPlan.auxiliary.size() == 0) 
        || (_mode == MODE::AUX && _flightPlan.auxiliary.size() == 0)) {
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

    if (next.interpolateAlt) {
      _interPos = _pos;
    }

    if (next.useHeading) {
      std::cout << "Changing Target " << next.pos <<std::endl;
      _vaildPathFound = false;
      generateHelperWaypoints(next);
    } else {
      addWaypoint(next, true);
      this->_target = next;

    }
  }
}

void Plane::updateParameters(float timeDelta) {
  if (_mode != MODE::GROUNDED) {
    _info.fuel -= config->fuelConsumption * timeDelta;
  }

  if (_pos.alt() <= 0.1 && _vel.value < 10) {
    _mode = MODE::GROUNDED;
  }
}

void Plane::updateGroundBehavior(float timeDelta) {

}

void Plane::setModeHdg() {
  if(_mode == MODE::AUTO) setAuxParam();
  _mode = MODE::HDG;
}

void Plane::setModeAux() {
  _mode = MODE::AUX;
  _flightPlan.auxiliary.clear();
  setAuxParam();
}

void Plane::setModeAuto() {
  _flightPlan.auxiliary.clear();
  _mode = MODE::AUTO;
}

void Plane::setModePlayer() {
    _mode = MODE::PLAYER;
}

double Plane::getTurnFactor() {
  double n = (_declaredEmergency) ? config->maxLoad : config->normalLoad;
  return 9.81 / _vel.value * std::sqrt(std::pow(n, 2) - 1);
}

double Plane::findHeadingDelta(GeoPos<double> pos, GeoPos<double> targetPos) {
  double targetHeading;
  if (_mode == MODE::HDG) targetHeading = _auxParam.vel.heading;
  else {
    targetHeading = fixAngle(
        std::atan2(targetPos.lat() - pos.lat(), targetPos.lon() - pos.lon()));
  }

  // Check if turn is possible
 /* if (_mode != MODE::HDG && checkMinRadius())
    return 0;*/

  // Check if advanced pathfinding requires repositioning
  if (_mode != MODE::HDG && !_vaildPathFound) {
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
  double vel = std::max(_vel.value, _target.vel.value);
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
  double rr = r * r;
  std::cout << "R: " << r << std::endl;

  std::array<GeoPos<double>, 2> target_cs = {};
  std::array<GeoPos<double>, 2> my_cs = {};

  

  // left first
  double angle;
  angle = tVel.heading + PI / 2;
  target_cs[0] = {{std::sin(angle) * r + tPos.lat(),
                   std::cos(angle) * r + tPos.lon(), tPos.alt()}};

  angle = tVel.heading - PI / 2;
  target_cs[1] = {{std::sin(angle) * r + tPos.lat(),
                   std::cos(angle) * r + tPos.lon(), tPos.alt()}};

  angle = _vel.heading + PI / 2;
  my_cs[0] = {{std::sin(angle) * r + _pos.lat(),
               std::cos(angle) * r + _pos.lon(), _pos.alt()}};

  angle = _vel.heading - PI / 2;
  my_cs[1] = {{std::sin(angle) * r + _pos.lat(),
               std::cos(angle) * r + _pos.lon(), _pos.alt()}};


  auto lerp = [](double p_from, double p_to, double p_weight, bool cw = true) {
      double diff = cw ? fmod(p_to - p_from, 2 * PI) : fmod(p_from - p_to, 2 * PI);
      double shortest = fmod(2.0 * diff, 2 * PI) - diff;
      return p_from + shortest * p_weight;
      };


  auto get_non_intersections = [&](const std::array<GeoPos<double>, 2> &t,
                                   const std::array<GeoPos<double>, 2> &m)
      -> std::vector<std::pair<int, int>> {
    std::vector<std::pair<int, int>> non_intersections;

    double minDist = std::numeric_limits<double>::infinity();
    for (int i = 0; i < 2; i++) {
      for (int j = 0; j < 2; j++) {
        double dy, dx;
        dy = m[i].lat() - t[j].lat();
        dx = m[i].lon() - t[j].lon();
        
        if (dx * dx + dy * dy >= rr * 4) {
          double dist = distance(t[j], m[i]);
          if (dist < minDist) {
            non_intersections.insert(non_intersections.begin(), {j, i});
            minDist = dist;
          } else {
            non_intersections.push_back({ j, i });
          }
        }
      }
    }

    return non_intersections;
  };

  auto non_intersections = get_non_intersections(target_cs, my_cs);
  if (non_intersections.size() > 0) {
    const auto& p = non_intersections[0];
    const auto& tcenter = target_cs[p.first];
    const auto& mcenter = my_cs[p.second];

    double x1, x2, x3, x4, y1, y2, y3, y4;
    x1 = mcenter.lon();
    y1 = mcenter.lat();
    x2 = tcenter.lon();
    y2 = tcenter.lat();


    std::vector<std::array<double, 2>> preTangent;
    std::vector<std::array<double, 2>> postTangent;

    //double startangle = _vel.heading;
    //double endangle = tVel.heading;

    if (p.first == p.second) {
      // po tej samej stronie, sytuacja A

      double gamma = -std::atan2(y2 - y1, x2 - x1);
      double beta = 0; // ??? u nas oba kola maja ten sam radius. pelen wzor to:
      // asin((R-r)/sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1))),
      // gdzie R i r to oba promienie
      double alpha = fixAngle(gamma - beta);
      std::cout << rad2dgr(alpha) << std::endl;

      double sign = (p.first == 1) ? 1 : -1;

      x3 = x1 + sign * r * std::sin(alpha);
      y3 = y1 + sign * r * std::cos(alpha);

      x4 = x2 + sign * r * std::sin(alpha);
      y4 = y2 + sign * r * std::cos(alpha);

      GeoPos<double> mtangent = { {y3, x3, tPos.alt()} };
      GeoPos<double> ttangent = { {y4, x4, tPos.alt()} };

      double startangle = alpha - std::acos((2 * rr - std::pow(distance(_pos, mtangent), 2)) / (2 * rr));
      double endangle   = alpha + std::acos((2 * rr - std::pow(distance(tPos, ttangent), 2)) / (2 * rr));
      double deltaAngle = (2 * PI) / std::ceil(r / 25);

      startangle = fixAngle(startangle);
      endangle = fixAngle(endangle);

      while (std::abs(startangle - alpha) >= 1.1 * deltaAngle) {
        std::cout << "start: " << rad2dgr(startangle) << std::endl;
        preTangent.push_back({ y1 + sign * r * std::cos(startangle), x1 + sign * r * std::sin(startangle) });
        std::cout << preTangent[preTangent.size() - 1][0] << " " << preTangent[preTangent.size() - 1][1] << std::endl;
        startangle += sign * deltaAngle;
        startangle = fixAngle(startangle);
      } 

      double tmpAlpha = alpha;
      while (std::abs(endangle - tmpAlpha) >= 1.1 * deltaAngle) {
        std::cout << "end: " << rad2dgr(tmpAlpha) << std::endl;
        postTangent.push_back({ y2 + sign * r * std::cos(tmpAlpha), x2 + sign * r * std::sin(tmpAlpha) });
        std::cout << postTangent[postTangent.size() - 1][0] << " " << postTangent[postTangent.size() - 1][1] << std::endl;
        tmpAlpha += sign * deltaAngle;
        tmpAlpha = fixAngle(tmpAlpha);
      }

    }
    else {
      // po przeciwnych stronach
      double C = 4 * rr + x1 * (x2 - x1) + y1 * (y2 - y1);
      double a = -((x2 - x1) * x2 + y2 * (y2 - y1));
      double b = x2 * C + (x2 - x1) * x2 * y2 + y2 * y2 * (y2 - y1);
      double c = -x2 * y2 * C;

      double tempx, tempy;

      tempx = (-b - std::sqrt(b * b - 4 * a * c)) / 2 * a; 
      // funkcja kwadratowa, wiadomo powinno być +/- ale narazie jest sam -
      tempy = (C - tempx * (x2 - x1)) / (y2 - y1);

      double alpha = fixAngle( -std::atan2(tempy, tempx));

      if (p.first != 1) {
        std::cout << "W lewo" << std::endl;
        x3 = x1 - r * std::sin(-alpha);
        y3 = y1 - r * std::cos(-alpha);

        x4 = x2 - r * std::sin(PI - alpha);
        y4 = y2 - r * std::cos(PI - alpha);

        GeoPos<double> mtangent = { {y3, x3, tPos.alt()} };
        GeoPos<double> ttangent = { {y4, x4, tPos.alt()} };

        double startangle = alpha - std::acos((2 * rr - std::pow(distance(_pos, mtangent), 2)) / (2 * rr));
        double endangle = alpha + std::acos((2 * rr - std::pow(distance(tPos, ttangent), 2)) / (2 * rr));
        double deltaAngle = (2 * PI) / std::ceil(r / 25);

        startangle = fixAngle(startangle);
        endangle = fixAngle(endangle);

        while (std::abs(startangle - alpha) >= 1.1 * deltaAngle) {
          std::cout << "start: " << rad2dgr(startangle) << std::endl;
          preTangent.push_back({ y1 -  r * std::cos(-startangle), x1 -  r * std::sin(-startangle) });
          startangle -= deltaAngle;
          startangle = fixAngle(startangle);
        }

        double tmpAlpha = alpha;
        while (std::abs(endangle - tmpAlpha) >= 1.1 * deltaAngle) {
          std::cout << "end: " << rad2dgr(tmpAlpha) << std::endl;
          postTangent.push_back({ y2 - r * std::cos(PI - tmpAlpha), x2 - r * std::sin(PI - tmpAlpha) });
          tmpAlpha -= deltaAngle;
          tmpAlpha = fixAngle(tmpAlpha);
        }

      }
      else {
        std::cout << "W prawo" << std::endl;
        x3 = x1 + r * std::sin(PI - alpha);
        y3 = y1 + r * std::cos(PI - alpha);

        x4 = x2 + r * std::sin(-alpha);
        y4 = y2 + r * std::cos(-alpha);

        GeoPos<double> mtangent = { {y3, x3, tPos.alt()} };
        GeoPos<double> ttangent = { {y4, x4, tPos.alt()} };

        double startangle = alpha - std::acos((2 * rr - std::pow(distance(_pos, mtangent), 2)) / (2 * rr));
        double endangle = alpha + std::acos((2 * rr - std::pow(distance(tPos, ttangent), 2)) / (2 * rr));
        double deltaAngle = (2 * PI) / std::ceil(r / 25);

        startangle = fixAngle(startangle);
        endangle = fixAngle(endangle);

        while (std::abs(startangle - alpha) >= 1.1 * deltaAngle) {
          std::cout << "start: " << rad2dgr(startangle) << std::endl;
          preTangent.push_back({ y1 + r * std::cos(PI - startangle), x1 + r * std::sin(PI - startangle) });
          startangle += deltaAngle;
          startangle = fixAngle(startangle);
        }

        double tmpAlpha = alpha;
        while (std::abs(endangle - tmpAlpha) >= 1.1 * deltaAngle) {
          std::cout << "end: " << rad2dgr(tmpAlpha) << std::endl;
          postTangent.push_back({ y2 + r * std::cos(- tmpAlpha), x2 + r * std::sin(- tmpAlpha) });
          tmpAlpha -= deltaAngle;
          tmpAlpha = fixAngle(tmpAlpha);
        }

      }
    }

    GeoPos<double> mtangent = {{y3, x3, tPos.alt()}};
    GeoPos<double> ttangent = {{y4, x4, tPos.alt()}};

    if (preTangent.size()) {
      addWaypoint({ {{preTangent[0][0],preTangent[0][1],tPos.alt()}}, tVel, false, false}, true);
      for (int i = 1; i < preTangent.size(); i++) {
        addWaypoint({ {{preTangent[i][0],preTangent[i][1],tPos.alt()}}, tVel, false, false });
      }
    }

    std::cout << "DEBUG - 1: " << mtangent << " 2: " << ttangent << std::endl;

    addWaypoint({ mtangent, tVel, false, false });
    addWaypoint({ ttangent, tVel, false, false });

    for (int i = 0; i < postTangent.size(); i++) {
      addWaypoint({ {{postTangent[i][0],postTangent[i][1],tPos.alt()}}, tVel, false, false });
    }

    addWaypoint(targetSegment);
    
    this->_vaildPathFound = true;

  } else {
    std::cout << "DEBUG: Path not found\n";
    addWaypoint(targetSegment, true);
    this->_vaildPathFound = false;
  }
}

void Plane::generateLandingWaypoints(bool orientation, double slopeAngle = 3,
                                     double distance = 5000) {
  FlightSegment landingA = {
      {{52.423969, 16.81117,  0}}, {0.0, hdg2rad(100) - MAGNETIC_NORTH_DIFF}, false, true };
  FlightSegment landingB = {
      {{52.418973, 16.837063, 0}}, {0.0, hdg2rad(280) - MAGNETIC_NORTH_DIFF}, false, true };

  FlightSegment choice = (orientation) ? landingB : landingA;
  choice.pos = geo2xy(choice.pos);
  double dir = choice.vel.heading - PI;
  double alt = distance * std::tan(dgr2rad(slopeAngle));

  GeoPos<double> waypoint = {{choice.pos.lat() + sin(dir) * distance,
                              choice.pos.lon() + cos(dir) * distance, alt}};
  generateHelperWaypoints({ waypoint, {config->landingSpeed, choice.vel.heading}, false, false });
  choice.vel.value = config->landingSpeed;
  addWaypoint(choice);
  std::cout << choice.pos << std::endl;
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
    _auxParam.vel.value = _target.vel.value;
    _auxParam.alt = _target.pos.alt();
    _auxParam.altMode = ALT_MODE::SET;
}

// order handling
void Plane::setAltitude(float altitude) {
  if (_mode == MODE::AUTO) {
    _target.pos.alt() = ft2meter(altitude);
  } else {
    _auxParam.alt = ft2meter(altitude);
  }
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
  if (_mode == MODE::AUTO) {
    _target.vel.value = kts2ms(vel);
  } else {
    _auxParam.vel.value = kts2ms(vel);
  }
}

void Plane::setSquawk(const std::string &sq) { this->_info.squawk = sq; }

void Plane::followFlightPlan() {
  _flightPlan.auxiliary.clear();
  setModeAuto();
}

void Plane::enterHolding() {}
void Plane::landing() {
  setModeAux();
  generateLandingWaypoints(true);
}
void Plane::touchAndGo() {
  setModeAux();
  generateLandingWaypoints(false);
}
void Plane::enterAirportLoop() {}
// order handling
