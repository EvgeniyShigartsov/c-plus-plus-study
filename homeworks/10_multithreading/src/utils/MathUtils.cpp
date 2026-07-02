#include <cmath>
#include "MathUtils.hpp"

// NOLINTBEGIN(readability-identifier-length)

Coord interpolatePos(const float frac, const Coord& currentTargetPos, const Coord& nextTargetPos)
{
  return currentTargetPos + (nextTargetPos - currentTargetPos) * frac;
}

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

InterpolationIndex getInterpolationIndex(const float t, const float arrayTimeStep, const int targetMovesCount)
{
  const int idx = (int)(floorf(t / arrayTimeStep)) % targetMovesCount;
  const int next = (idx + 1) % targetMovesCount;
  const float frac = (t - static_cast<float>(idx) * arrayTimeStep) / arrayTimeStep;

  return {frac, idx, next};
}

float getDirectionFromTo(const Coord& from, const Coord& to)
{
  return atan2f(to.y - from.y, to.x - from.x);
}

// NOLINTEND(readability-identifier-length)