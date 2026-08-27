#include <cmath>
#include "MathUtils.hpp"

// NOLINTBEGIN(readability-identifier-length)

float length(const Coord& coord)
{
  return std::hypot(coord.x, coord.y);
}

Coord normalizeCoord(const Coord& coord)
{
  return coord / length(coord);
}

float normalizeAngle(float angle)
{
  while (angle > M_PI) {
    angle -= 2.0f * M_PI;
  }

  while (angle < -M_PI) {
    angle += 2.0f * M_PI;
  }
  return angle;
};

float getDirectionFromTo(const Coord& from, const Coord& to)
{
  return atan2f(to.y - from.y, to.x - from.x);
}

// NOLINTEND(readability-identifier-length)