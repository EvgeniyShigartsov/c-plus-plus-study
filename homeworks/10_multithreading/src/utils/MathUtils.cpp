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

Coord movePos(const Coord& pos, const float dir, const float speed, const float deltaTime)
{
  return pos + Coord{cosf(dir), sinf(dir)} * speed * deltaTime;
}

float rotateToward(const float currentDir, const float targetDir, const float angularSpeed, const float deltaTime)
{
  const float diff = normalizeAngle(targetDir - currentDir);
  const float maxStep = angularSpeed * deltaTime;

  if (diff > maxStep) {
    return currentDir + maxStep;
  }
  if (diff < -maxStep) {
    return currentDir - maxStep;
  }
  return targetDir;
}

// NOLINTEND(readability-identifier-length)