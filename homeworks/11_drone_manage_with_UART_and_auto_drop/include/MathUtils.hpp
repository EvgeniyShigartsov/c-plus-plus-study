#pragma once
#include <cstring>
#include "types.hpp"

// NOLINTBEGIN(readability-identifier-length)

float length(const Coord& coord);

Coord normalizeCoord(const Coord& coord);

float getDirectionFromTo(const Coord& from, const Coord& to);

float normalizeAngle(float angle);

// NOLINTEND(readability-identifier-length)