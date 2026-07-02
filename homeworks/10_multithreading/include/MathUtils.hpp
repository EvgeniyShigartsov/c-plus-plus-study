#pragma once
#include <cstring>
#include "types.hpp"

// NOLINTBEGIN(readability-identifier-length)

Coord interpolatePos(const float frac, const Coord& currentTargetPos, const Coord& nextTargetPos);
float length(const Coord& coord);

Coord normalizeCoord(const Coord& coord);

InterpolationIndex getInterpolationIndex(const float t, const float arrayTimeStep, const int targetMovesCount);

float getDirectionFromTo(const Coord& from, const Coord& to);

float normalizeAngle(float angle);

// NOLINTEND(readability-identifier-length)