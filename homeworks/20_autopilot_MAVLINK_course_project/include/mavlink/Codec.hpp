#pragma once
#include <cstdint>

#include <common/mavlink.h>

#include "mavlink/Identity.hpp"
#include "types.hpp"

namespace mav {

struct Heartbeat {
  uint8_t type = 0;       // MAV_TYPE_*
  uint8_t autopilot = 0;  // MAV_AUTOPILOT_*
  uint8_t base_mode = 0;  // MAV_MODE_FLAG_*
  uint32_t custom_mode = 0;
  uint8_t system_status = 0;  // MAV_STATE_*

  bool operator==(const Heartbeat&) const = default;
};

struct LocalPositionNed {
  uint32_t time_boot_ms = 0;  // час від старту, мс

  // де я
  float north = 0.0f;
  float east = 0.0f;
  float down = 0.0f;  // висота зі знаком мінус

  // швидкість руху
  float velocity_north = 0.0f;
  float velocity_east = 0.0f;
  float velocity_down = 0.0f;

  bool operator==(const LocalPositionNed&) const = default;
};

struct Attitude {
  uint32_t time_boot_ms = 0;  // час від старту, мс
  float yaw = 0.0f;           // курс, куди дивиться ніс, у системі NED (за годинниковою)

  bool operator==(const Attitude&) const = default;
};

mavlink_message_t pack_heartbeat(const Identity& from, const Heartbeat& heartbeat);
Heartbeat parse_heartbeat(const mavlink_message_t& msg);

mavlink_message_t pack_local_position_ned(const Identity& from, const VehicleState& state);
LocalPositionNed parse_local_position_ned(const mavlink_message_t& msg);

mavlink_message_t pack_attitude(const Identity& from, const VehicleState& state);
Attitude parse_attitude(const mavlink_message_t& msg);

mavlink_message_t pack_global_position_int(const Identity& from, const VehicleState& state);

VehicleState to_vehicle_state(const LocalPositionNed& position, const Attitude& attitude);
}  // namespace mav
