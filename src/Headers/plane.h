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
  GeoPos<double> targetPos;
};

inline void to_json(json &j, const PlaneData &p) {
  j = json{
      {"id", p.info.id},
      {"sim_id", p.info.sim_id},
      {"isGrounded", p.info.isGrounded},
      {"airline", p.info.airline},
      {"flight_number", p.info.flightNumber},
      {"plane_number", p.info.planeNumber},
      {"callSign", p.info.callSign},
      {"squawk", p.info.squawk},
      {"model", p.info.model},
      {"velocity",
       {{"direction", p.vel.heading},
        {"value", p.vel.value}}},
      {"position",
       {{"latitude", p.pos.lat()},
        {"longitude", p.pos.lon()},
        {"altitude", p.pos.alt()}}},
      {"target",
       {{"latitude", p.targetPos.lat()},
        {"longitude", p.targetPos.lon()},
        {"altitude", p.targetPos.alt()}}}};
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
  vel.at("direction").get_to(p.vel.heading);
  vel.at("value").get_to(p.vel.value);

  const auto &pos = j.at("position");
  pos.at("latitude").get_to(p.pos.lat());
  pos.at("longitude").get_to(p.pos.lon());
  pos.at("altitude").get_to(p.pos.alt());

  const auto &tar = j.at("target");
  pos.at("latitude").get_to(p.targetPos.lat());
  pos.at("longitude").get_to(p.targetPos.lon());
  pos.at("altitude").get_to(p.targetPos.alt());
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
                {"direction", p.vel.heading},
                {"value", p.vel.value},
            }},
           {"position",
            {{"latitude", p.pos.lat()},
             {"longitude", p.pos.lon()},
             {"altitude", p.pos.alt()}}},
           {"target",
            {{"latitude", p.targetPos.lat()},
             {"longitude", p.targetPos.lon()},
             {"altitude", p.targetPos.alt()}}}};
}

// from_json function
inline void from_json(const json &j, PlaneFlightData &p) {
  j.at("squawk").get_to(p.squawk);
  j.at("id").get_to(p.id);

  const auto &vel = j.at("velocity");
  vel.at("direction").get_to(p.vel.heading);
  vel.at("value").get_to(p.vel.value);

  const auto &pos = j.at("position");
  pos.at("latitude").get_to(p.pos.lat());
  pos.at("longitude").get_to(p.pos.lon());
  pos.at("altitude").get_to(p.pos.alt());

  const auto &targetPos = j.at("target");
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

  FlightSegment target;
  double setClimbSpeed;

  bool declaredEmergency = false;
  bool ignoreFlightPlan = false;
  FlightPlan flightPlan;
  const PlaneConfig* config;

  void setData(const data::PlaneData &pd) {
    // TODO: maybe implement a better function of setting the data
    this->info = pd.info;
    this->vel = pd.vel;
    this->pos = pd.pos;
    this->target.pos = pd.targetPos;
    // data::PlaneFlightData pd2;
  }
  data::PlaneData getData() const {
    return data::PlaneData{this->info, 
                          {ms2kts(this->vel.value), -this->vel.heading},
                           xy2geo(this->pos)}; // coordinate conversion
  }

  void setFlightData(const data::PlaneFlightData &pd) {
    // TODO: maybe implement a better function of setting the data
    this->pos = geo2xy(pd.pos);
    this->vel.heading = hdg2rad(pd.vel.heading);
    this->vel.value = kts2ms(pd.vel.value);
    this->target.pos = geo2xy(pd.targetPos);
  }
  data::PlaneFlightData getFlightData() const {
    return data::PlaneFlightData{this->info.id, this->info.squawk, 
                                {ms2kts(this->vel.value), -this->vel.heading},
                                 xy2geo(this->pos), xy2geo(this->target.pos) };
  }

  Plane(const data::PlaneData &data, const FlightPlan &flightplan,
      const PlaneConfig* configPointer) {
    this->info = data.info;
    this->vel = data.vel;
    this->vel.value = std::min(std::max(vel.value, configPointer->minSpeed),
                               configPointer->maxSpeed);
    this->pos = geo2xy(data.pos);
    this->pos.alt() =
        std::min(std::max(pos.alt(), 0.0), configPointer->maxAltitude);
    this->flightPlan = flightplan;

    this->config = configPointer;
    setClimbSpeed = config->deafultClimbingSpeed;

    updateFlightPlan(true);
  }

  void generateLandingWaypoints(bool orientation, double slopeAngle = 3, double distance = 5000) {
      FlightSegment landingA = { {{52.423969, 16.81117, 0}}, {0.0, hdg2rad(100)}, true };
      FlightSegment landingB = { {{52.418973, 16.837063,0}}, {0.0, hdg2rad(280)}, true };

      FlightSegment choice = (orientation) ? landingB : landingA;
      double dir = choice.useHeading - PI;
      double alt = distance * std::tan(dgr2rad(slopeAngle));

      GeoPos<double> waypoint = { {choice.pos.lat() + sin(dir) * distance,
                                  choice.pos.lon() + cos(dir) * distance, alt} };
      generateHelperWaypoints(FlightSegment{ waypoint, config->landingSpeed, false });
      choice.vel.value = config->landingSpeed;
      flightPlan.auxiliary.push_back(choice);
  }

  void update(float timeDelta) {
    // Debug
    //std::cout << info.callSign << " target: " << target.pos
    //          << " dist: " << distance(pos, target.pos) << std::endl;
    //std::cout << "Hdg: " << rad2hdg(vel.heading) << " Speed: " << vel.value
    //          << std::endl;
    //std::cout << "Pos(GEO): " << xy2geo(pos) << std::endl;
    //std::cout << "Pos (XY): " << pos << std::endl << std::endl;
    updateFlightPlan();
    updateVelocity(timeDelta);
    updatePosition(timeDelta);
  }

private:
  void updateVelocity(float timeDelta) {
    double velDelta = target.vel.value - vel.value;
    vel.value += sgn(velDelta) *
                 std::min(std::abs(velDelta),
                          std::pow(config->accelerationFactor, 2) / vel.value) *
                 timeDelta;
    vel.value =
        std::min(std::max(vel.value, config->minSpeed), config->maxSpeed);

    double hdgDelta = findHeadingDelta(pos, target.pos);
    vel.heading += sgn(hdgDelta) *
                   std::min(std::abs(hdgDelta), getTurnFactor() * timeDelta);
    vel.heading = fixAngle(vel.heading);
  }

  void updatePosition(float timeDelta) {
    pos.lat() += std::sin(vel.heading) * vel.value * timeDelta;
    pos.lon() += std::cos(vel.heading) * vel.value * timeDelta;
    double altitudeDelta = target.pos.alt() - pos.alt();
    pos.alt() += sgn(altitudeDelta) *
                 std::min(std::abs(altitudeDelta), setClimbSpeed * timeDelta);
    pos.alt() = std::max(std::min(pos.alt(), config->maxAltitude), 0.0);
  }

  void updateFlightPlan(bool force = false, double margin = 10) {
    if (force || distance(pos, target.pos) < margin) {
      if (flightPlan.route.size() == 0 && flightPlan.auxiliary.size() == 0 &&
          !info.isGrounded) {
        ignoreFlightPlan = true;
        return;
      }
      FlightSegment next;
      if (flightPlan.auxiliary.size()) {
        next = flightPlan.auxiliary.front();
        flightPlan.auxiliary.pop_front();
      } else {
        next = flightPlan.route.front();
        flightPlan.route.pop_front();
      }

      if (next.useHeading) {
        std::cout << "Changing Target " << std::endl;
        vaildPathFound = false;
        generateHelperWaypoints(next);
      } else {
        this->target = next;
        // maybe coord transformation should be moved to scenerio loading
      }
    }
  }

  double getTurnFactor() {
    double n = (declaredEmergency) ? config->maxLoad : config->normalLoad;
    return 9.81 / (vel.value * std::sqrt(std::pow(n, 2) - 1));
  }

  double findHeadingDelta(GeoPos<double> pos, GeoPos<double> targetPos) {
    double targetHeading = fixAngle(
        std::atan2(targetPos.lat() - pos.lat(), targetPos.lon() - pos.lon()));
    // std::cout << "Target Hdg: " << rad2dgr(targetHeading) << std::endl;

    //Check if turn is possible 
    if (checkMinRadius())  return 0;
    
    //Check if advanced pathfinding requires repositioning 
    if (!vaildPathFound) {
      generateHelperWaypoints(target);
      return 0;
    }

    double hdgDelta = targetHeading - vel.heading;
    double altDelta = (2 * PI - std::abs(hdgDelta)) * -sgn(hdgDelta);
    if (std::abs(altDelta) < std::abs(hdgDelta))
      hdgDelta = altDelta;
    return hdgDelta;
  }

  bool checkMinRadius() {
    double n = (declaredEmergency) ? config->maxLoad : config->normalLoad;
    double r = std::sqrt(-std::pow(target.vel.value, 4) /
                         (std::pow(G, 2) * (1 - std::pow(n, 2))));
    GeoPos<double> A = {{pos.lat() + std::sin(vel.heading + PI / 2) * r,
                         pos.lon() + std::cos(vel.heading + PI / 2) * r,
                         target.pos.alt()}};
    GeoPos<double> B = {{pos.lat() - std::sin(vel.heading + PI / 2) * r,
                         pos.lon() - std::cos(vel.heading + PI / 2) * r,
                         target.pos.alt()}};
    return (distance(target.pos, A) < r || distance(target.pos, B) < r);
  }

  void generateHelperWaypoints(FlightSegment targetSegment) {
    // TO DO: check 4 circle non-overlaping match combinations and find
    // coresponding 4 valid tangent lines if match exist stop ignoring heading
    // delta -> vaildPathFound = true;

    GeoPos<double> tPos = targetSegment.pos;
    Velocity tVel = targetSegment.vel;

    double n = (declaredEmergency) ? config->maxLoad : config->normalLoad;
    double rr =
        -std::pow(tVel.value, 4) / (std::pow(G, 2) * (1 - std::pow(n, 2)));
    double r = std::sqrt(rr);
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

    angle = this->vel.heading + PI / 2;
    my_cs[0] = {{std::sin(angle) * r + this->pos.lat(),
                 std::cos(angle) * r + this->pos.lon(), this->pos.alt()}};

    angle = this->vel.heading - PI / 2;
    my_cs[1] = {{std::sin(angle) * r + this->pos.lat(),
                 std::cos(angle) * r + this->pos.lon(), this->pos.alt()}};

    auto get_non_intersections = [&](const std::array<GeoPos<double>, 2> &t,
                                     const std::array<GeoPos<double>, 2> &m)
        -> std::vector<std::pair<int, int>> {
      std::vector<std::pair<int, int>> non_intersections;

      for (int i = 0; i < 2; i++) {
        for (int j = 0; j < 2; j++) {
          double dy, dx;
          dy = m[i].lat() - t[j].lat();
          dx = m[i].lon() - t[j].lon();

          if (dx * dx + dy * dy >= rr * 4) {
            non_intersections.push_back({j, i});
          }
        }
      }

      return non_intersections;
    };

    auto non_intersections = get_non_intersections(target_cs, my_cs);
    if (non_intersections.size() > 0) {
      const auto &p = non_intersections[0];
      const auto &tcenter = target_cs[p.first];
      const auto &mcenter = my_cs[p.second];

      double x1, x2, x3, x4, y1, y2, y3, y4;
      x1 = mcenter.lon();
      y1 = mcenter.lat();
      x2 = tcenter.lon();
      y2 = tcenter.lat();

      if (p.first == p.second) {
        // po tej samej stronie, sytuacja A

        double gamma = -std::atan2(y2 - y1, x2 - x1);
        double beta =
            0; // ??? u nas oba kola maja ten sam radius. pelen wzor to:
               // asin((R-r)/sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1))), gdzie R i r
               // to oba promienie
        double alpha = gamma - beta;

        if (p.first == 1) {
          x3 = x1 + r * std::sin(alpha);
          y3 = y1 + r * std::cos(alpha);

          x4 = x2 + r * std::sin(alpha);
          y4 = y2 + r * std::cos(alpha);
        } else {
          x3 = x1 - r * std::sin(alpha);
          y3 = y1 - r * std::cos(alpha);

          x4 = x2 - r * std::sin(alpha);
          y4 = y2 - r * std::cos(alpha);
        }

      } else {
        // po przeciwnych stronach
        double C = 4 * rr + x1 * (x2 - x1) + y1 * (y2 - y1);
        double a = -((x2 - x1) * x2 + y2 * (y2 - y1));
        double b = x2 * C + (x2 - x1) * x2 * y2 + y2 * y2 * (y2 - y1);
        double c = -x2 * y2 * C;

        double tempx, tempy;

        tempx = (-b + std::sqrt(b * b - 4 * a * c)) / 2 *
                a; // funkcja kwadratowa, wiadomo powinno być +/- ale narazie
                   // jest sam +
        tempy = (C - tempx * (x2 - x1)) / (y2 - y1);

        double alpha = -std::atan2(tempy, tempx);

        if (p.first == 1) {
          x3 = x1 - r * std::sin(-alpha);
          y3 = y1 - r * std::cos(-alpha);

          x4 = x2 - r * std::sin(PI - alpha);
          y4 = y2 - r * std::cos(PI - alpha);
        } else {
          x3 = x1 + r * std::sin(PI - alpha);
          y3 = y1 + r * std::cos(PI - alpha);

          x4 = x2 + r * std::sin(-alpha);
          y4 = y2 + r * std::cos(-alpha);
        }
      }

      GeoPos<double> mtangent = {{y3, x3, tPos.alt()}};
      GeoPos<double> ttangent = {{y4, x4, tPos.alt()}};

      std::cout << "DEBUG - 1: " << mtangent << " 2: " << ttangent << std::endl;

      this->target = FlightSegment{mtangent, tVel, false};
      this->flightPlan.auxiliary.push_back({ttangent, tVel, false});
      targetSegment.useHeading = false;
      this->flightPlan.auxiliary.push_back(targetSegment);
      this->vaildPathFound = true;

    } else {
      std::cout << "DEBUG: Path not found\n";
      this->vaildPathFound = false;
      this->target = targetSegment;
    }
  }
};
