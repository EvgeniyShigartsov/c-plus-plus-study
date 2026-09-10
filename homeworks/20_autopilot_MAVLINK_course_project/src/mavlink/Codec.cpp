#include "mavlink/Codec.hpp"

namespace mav {

mavlink_message_t pack_heartbeat(const Heartbeat& heartbeat)
{
  mavlink_message_t msg{};
  mavlink_msg_heartbeat_pack(heartbeat.from.sysid,
                             heartbeat.from.compid,
                             &msg,
                             heartbeat.type,
                             heartbeat.autopilot,
                             heartbeat.base_mode,
                             heartbeat.custom_mode,
                             heartbeat.system_status);
  return msg;
}

Heartbeat parse_heartbeat(const mavlink_message_t& msg)
{
  mavlink_heartbeat_t raw{};
  mavlink_msg_heartbeat_decode(&msg, &raw);

  return Heartbeat{
    .from = {.sysid = msg.sysid, .compid = msg.compid},
    .type = raw.type,
    .autopilot = raw.autopilot,
    .base_mode = raw.base_mode,
    .custom_mode = raw.custom_mode,
    .system_status = raw.system_status,
  };
}

}  // namespace mav
