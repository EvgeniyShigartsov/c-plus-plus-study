#pragma once
#include <cstdint>

#include <common/mavlink.h>

#include "mavlink/Identity.hpp"

namespace mav {

struct Heartbeat {
  Identity from;
  uint8_t type = 0;           // MAV_TYPE_*
  uint8_t autopilot = 0;      // MAV_AUTOPILOT_*
  uint8_t base_mode = 0;      // MAV_MODE_FLAG_*
  uint32_t custom_mode = 0;   // у autopilot — стан арбітражу
  uint8_t system_status = 0;  // MAV_STATE_*

  bool operator==(const Heartbeat&) const = default;
};

mavlink_message_t pack_heartbeat(const Heartbeat& heartbeat);

Heartbeat parse_heartbeat(const mavlink_message_t& msg);

}  // namespace mav
