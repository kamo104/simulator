#include "scenario.h"

namespace data {
// ScenarioWaypoint parsing
void to_json(json &j, const ScenarioWaypoint &p) {
  j = json{{"id", p.id},
           {"statusId", p.statusId},
           {"time", p.time},
           {"is_entrypoint", p.is_entrypoint},
           {"is_exit_point", p.is_exit_point},
           {"fuel_state", p.fuel_state},
           {"weather", p.weather},
           {"events", p.events},
           {"position", p.position},
           {"velocity", p.velocity}};
};
void from_json(const json &j, ScenarioWaypoint &p) {
  j.at("id").get_to(p.id);
  j.at("statusId").get_to(p.statusId);
  j.at("time").get_to(p.time);
  j.at("is_entrypoint").get_to(p.is_entrypoint);
  j.at("is_exit_point").get_to(p.is_exit_point);
  j.at("fuel_state").get_to(p.fuel_state);
  j.at("weather").get_to(p.weather);
  j.at("events").get_to(p.events);
  j.at("position").get_to(p.position);
  j.at("velocity").get_to(p.velocity);
};
// ScenarioWaypoint parsing

// ScenarioFlight parsing
void to_json(json &j, const ScenarioFlight &p) {
  j = json{{"type_of_aircraft", p.type_of_aircraft},
           {"registrationnumber", p.registrationnumber},
           {"destination", p.destination},
           {"alternative_airports", p.alternative_airports},
           {"weather", p.weather}};
  j.merge_patch(json(p.planeData));
};
void from_json(const json &j, ScenarioFlight &p) {
  j.get_to(p.planeData);
  j["type_of_aircraft"].get_to(p.type_of_aircraft);
  j["registrationnumber"].get_to(p.registrationnumber);
  j["destination"].get_to(p.destination);
  j["alternative_airports"].get_to(p.alternative_airports);
  j["weather"].get_to(p.weather);
  return;
};
// ScenarioFlight parsing

// Scenario parsing
void to_json(json &j, const Scenario &p) { j = json(p.flights); };
void from_json(const json &j, Scenario &p) { j.get_to(p.flights); };
// Scenario parsing

} // namespace data
