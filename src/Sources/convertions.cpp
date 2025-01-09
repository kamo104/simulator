#include "convertions.h"

double dgr2rad(double degrees) { return degrees * (PI / 180.0); }

double rad2dgr(double radian) { return radian * (180.0 / PI); }

double meter2lat(double distance) { return distance * 0.0000089831117; }

double meter2long(double distance, double latitude) {
  return distance * 0.00000898315658 / std::cos(latitude);
}

double distance(GeoPos<double> posA, GeoPos<double> posB) {
  return std::sqrt(std::pow(posA.lat() - posB.lat(), 2) +
                   std::pow(posA.lon() - posB.lon(), 2) +
                   std::pow(posA.alt() - posB.alt(), 2));
}

double fixAngle(double angle) {
  while (angle < 0)
    angle += 2 * PI;
  while (angle > 2 * PI)
    angle -= 2 * PI;
  return angle;
}

double ms2kts(double value) { return value * 1.943844; }
double kts2ms(double value) { return value / 1.943844; }

GeoPos<double> geo2xy(GeoPos<double> pos) {
  return GeoPos<double>{
      {pos.lat() * matrix[0][0] + pos.lon() * matrix[0][1] + matrixConst[0],
       pos.lat() * matrix[1][0] + pos.lon() * matrix[1][1] + matrixConst[1],
       pos.alt()}};
}

GeoPos<double> xy2geo(GeoPos<double> pos) {
  return GeoPos<double>{{(pos.lat() - matrixConst[0]) * matrixInv[0][0] +
                             (pos.lon() - matrixConst[1]) * matrixInv[0][1],
                         (pos.lat() - matrixConst[0]) * matrixInv[1][0] +
                             (pos.lon() - matrixConst[1]) * matrixInv[1][1],
                         pos.alt()}};
}
