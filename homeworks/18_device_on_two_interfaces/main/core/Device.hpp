#pragma once
#include <cstddef>
#include <cstdint>
#include <string_view>

#include "Reading.hpp"

class Device {
public:
  bool onTick(char* buf, const size_t capacity, const uint32_t tMs, const Reading& reading) const;

  bool onCommand(const std::string_view line, char* buf, const size_t capacity);

  uint32_t periodMs() const { return periodMsValue; }

private:
  uint32_t periodMsValue = 500;
};
