#pragma once
#include <cstring>
#include "types.hpp"

// NOLINTBEGIN(readability-identifier-length)

Coord interpolatePos(const float frac, const Coord& currentTargetPos, const Coord& nextTargetPos);
float length(const Coord& coord);

Coord normalizeCoord(const Coord& coord);

InterpolationIndex getInterpolationIndex(const float t, const float arrayTimeStep, const int targetMovesCount);

float getDirectionFromTo(const Coord& from, const Coord& to);

// Нова позиція після руху за напрямком dir зі швидкістю speed протягом deltaTime.
Coord movePos(const Coord& pos, const float dir, const float speed, const float deltaTime);

float normalizeAngle(float angle);

// Зміна курсу плавно, не швидше за angularSpeed.
float rotateToward(const float currentDir, const float targetDir, const float angularSpeed, const float deltaTime);

// NOLINTEND(readability-identifier-length)