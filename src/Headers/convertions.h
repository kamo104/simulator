#pragma once
#include <cmath>

const double PI = 3.1415;

double dgr2rad(double degrees) {
    return degrees * (PI / 180.0);
}

double rad2dgr(double radian) {
    return radian * (180.0 / PI);
}

double meter2lat(double distance) {
    return distance * 0.0000089831117;
}

double meter2long(double distance, double latitude) {
    return distance * 0.00000898315658 / std::cos(latitude);
}

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

double distance(GeoPos<double> posA, GeoPos<double> posB) {
    return std::sqrt(std::pow(posA.lat() - posB.lat(), 2) + std::pow(posA.lon() - posB.lon(), 2) + std::pow(meter2lat(posA.alt()) - meter2lat(posB.alt()), 2));
}

double fixAngle(double angle) {
    while (angle < 0) angle += 2 * PI;
    while (angle > 2 * PI) angle -= 2 * PI;
    return angle;
}