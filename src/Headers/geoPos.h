#pragma once
#include "common.h"
#include "vec.h"

template <typename T> class GeoPos : public Vec<T, 3> {
public:
    T& lat() { return (*this)[0]; }  // Use index 0 for latitude
    T& lon() { return (*this)[1]; }  // Use index 1 for longitude
    T& alt() { return (*this)[2]; }  // Use index 2 for altitude
    const T& lat() const { return (*this)[0]; }
    const T& lon() const { return (*this)[1]; }
    const T& alt() const { return (*this)[2]; }
};