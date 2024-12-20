#pragma once
#include "common.h"

template <typename T, uint N> class Vec {
  std::array<T, N> components;

public:
  // Constructors
  Vec() : components{} {}
  Vec(std::initializer_list<T> init) {
    if (init.size() != N) {
      throw std::invalid_argument(
          "Initializer list size must match the dimension N");
    }
    std::copy(init.begin(), init.end(), components.begin());
  }

  // Accessors
  template <uint M = N, typename std::enable_if<(M >= 1), int>::type = 0>
  T x() const {
    return components[0];
  }
  template <uint M = N, typename std::enable_if<(M >= 1), int>::type = 0>
  const T &x() const {
    return components[0];
  }
  template <uint M = N, typename std::enable_if<(M >= 2), int>::type = 0>
  T y() const {
    return components[1];
  }
  template <uint M = N, typename std::enable_if<(M >= 2), int>::type = 0>
  const T &y() const {
    return components[1];
  }
  template <uint M = N, typename std::enable_if<(M >= 3), int>::type = 0>
  T z() const {
    return components[2];
  }
  template <uint M = N, typename std::enable_if<(M >= 3), int>::type = 0>
  const T &z() const {
    return components[2];
  }

  // Array access operator
  T &operator[](size_t i) { return components[i]; }
  const T &operator[](size_t i) const { return components[i]; }

  // Arithmetic operators
  Vec &operator+=(const Vec &rhs) {
    for (size_t i = 0; i < N; ++i) {
      components[i] += rhs.components[i];
    }
    return *this;
  }
  Vec &operator-=(const Vec &rhs) {
    for (size_t i = 0; i < N; ++i) {
      components[i] -= rhs.components[i];
    }
    return *this;
  }
  Vec operator+(const Vec &rhs) const {
    Vec result;
    for (size_t i = 0; i < N; ++i) {
      result.components[i] = components[i] + rhs.components[i];
    }
    return result;
  }
  Vec operator-(const Vec &rhs) const {
    Vec result;
    for (size_t i = 0; i < N; ++i) {
      result.components[i] = components[i] - rhs.components[i];
    }
    return result;
  }

  // Output stream
  friend std::ostream &operator<<(std::ostream &os, const Vec &vec) {
    os << '[';
    for (size_t i = 0; i < N; ++i) {
      os << vec.components[i];
      if (i < N - 1) {
        os << ", ";
      }
    }
    os << ']';
    return os;
  }
};
