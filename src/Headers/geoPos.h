#pragma once
#include "common.h"
#include "vec.h"

template <typename T> class GeoPos : public Vec<T, 3> {
  // Synonyms for geographic coordinates
  T &lat() { return this->x(); }
  T &lon() { return this->y(); }
  T &alt() { return this->z(); }
  const T &lat() const { return this->x(); }
  const T &lon() const { return this->y(); }
  const T &alt() const { return this->z(); }
};
