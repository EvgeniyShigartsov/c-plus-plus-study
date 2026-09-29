#pragma once
#include <cstddef>
#include <cstdint>

#include "Reading.hpp"

namespace StatusLine {

bool format(char* buf, const size_t capacity, const uint32_t tMs, const Reading& reading);

}  // namespace StatusLine
