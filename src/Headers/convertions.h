#pragma once
#include "geoPos.h"
#include <cmath>

const double PI = 3.1415;
const double G = 9.81;
const double MAGNETIC_NORTH_DIFF = 0.10192422222;

double fixAngle(double angle);

double dgr2rad(double degrees);
double hdg2rad(double degrees);

double rad2dgr(double radian);
double rad2hdg(double degrees);

double meter2ft(double alt);
double ft2meter(double alt);

double meter2lat(double distance);

double meter2long(double distance, double latitude);

template <typename T> int sgn(T val) { return (T(0) < val) - (val < T(0)); }

double distance(GeoPos<double> posA, GeoPos<double> posB);

double ms2kts(double value);
double kts2ms(double value);

const double matrix[2][2] = {{1.11272107e+05, 1.19500542e+00},
                             {9.67595934e+01, 6.80570555e+04}};
const double matrixInv[2][2] = {{8.98697832e-06, -1.57801241e-10},
                                {-1.27771671e-08, 1.46935539e-05}};
const double matrixConst[2] = {-5.83292455e+06, -1.15021977e+06};

GeoPos<double> geo2xy(GeoPos<double> pos);

GeoPos<double> xy2geo(GeoPos<double> pos);

double parseDMS(const std::string &dms);
