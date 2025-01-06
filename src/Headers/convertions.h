#pragma once
#include "geoPos.h"
#include <cmath>

const double PI = 3.1415;
const double G = 9.81;

inline double fixAngle(double angle) {
    while (angle < 0)
        angle += 2 * PI;
    while (angle > 2 * PI)
        angle -= 2 * PI;
    return angle;
}

inline double dgr2rad(double degrees) { return degrees * (PI / 180.0); }
inline double hdg2rad(double degrees) { return dgr2rad(fixAngle(450 - degrees)); }

inline double rad2dgr(double radian) { return radian * (180.0 / PI); }
inline double rad2hdg(double degrees) { return fmod(450 - rad2dgr(degrees), 360.0); }

inline double meter2lat(double distance) { return distance * 0.0000089831117; }

inline double meter2long(double distance, double latitude) {
  return distance * 0.00000898315658 / std::cos(latitude);
}

template <typename T> int sgn(T val) { return (T(0) < val) - (val < T(0)); }

inline double distance(GeoPos<double> posA, GeoPos<double> posB) {
  return std::sqrt(std::pow(posA.lat() - posB.lat(), 2) +
                   std::pow(posA.lon() - posB.lon(), 2) +
                   std::pow(posA.alt() - posB.alt(), 2));
}

inline double ms2kts(double value) { return value * 1.943844; }
inline double kts2ms(double value) { return value / 1.943844; }

const double matrix[2][2] = { {1.11272107e+05,  1.19500542e+00}, {9.67595934e+01,  6.80570555e+04} };
const double matrixInv[2][2] = { {8.98697832e-06,  -1.57801241e-10}, {-1.27771671e-08, 1.46935539e-05} };
const double matrixConst[2] = {-5.83292455e+06, -1.15021977e+06};

inline GeoPos<double> geo2xy(GeoPos<double> pos) {
  return GeoPos<double> {{ pos.lat() * matrix[0][0] + pos.lon() * matrix[0][1] + matrixConst[0],
	     pos.lat() * matrix[1][0] + pos.lon() * matrix[1][1] + matrixConst[1],
	     pos.alt() }};
}

inline GeoPos<double> xy2geo(GeoPos<double> pos) {
  return GeoPos<double> {{ (pos.lat() - matrixConst[0]) * matrixInv[0][0] + (pos.lon() - matrixConst[1]) * matrixInv[0][1],
	     (pos.lat() - matrixConst[0]) * matrixInv[1][0] + (pos.lon() - matrixConst[1]) * matrixInv[1][1],
	      pos.alt() }};
}

