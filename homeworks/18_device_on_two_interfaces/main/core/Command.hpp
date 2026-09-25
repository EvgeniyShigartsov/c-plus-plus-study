#pragma once
#include <cstdint>
#include <string_view>

struct Command {
  enum class Kind { SetPeriod, Unknown } kind = Kind::Unknown;
  uint32_t periodMs = 0;
};

Command parseCommand(const std::string_view line);
