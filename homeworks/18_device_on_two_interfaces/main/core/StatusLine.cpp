#include "StatusLine.hpp"

#include <cstdio>

namespace StatusLine {

bool format(char* buf, const size_t capacity, const uint32_t tMs, const Reading& reading)
{
  const int written = std::snprintf(buf, capacity, "t=%u ms  value=%.3f", tMs, reading.value);

  return written > 0 && static_cast<size_t>(written) < capacity;
}

}  // namespace StatusLine
