#pragma once
#include <common/mavlink.h>

#include "types.hpp"

// Куди DroneNode віддає результати роботи, залежно від платформи вивід різний:
// UDP на хості (drone_sim), UART на ESP32
// NOLINTNEXTLINE(cppcoreguidelines-special-member-functions)
class IDroneOutput {
public:
  virtual void send(const mavlink_message_t& msg) = 0;
  virtual void onDrop(const Coord& aimPoint, const float timeSinceStart) = 0;
  virtual void onMissionComplete() = 0;
  virtual ~IDroneOutput() = default;
};
