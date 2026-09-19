#include "bridge/SerialBridge.hpp"

#include "mavlink/Codec.hpp"

SerialBridge::SerialBridge(const mav::MavlinkEndpoint& drone,
                           const mav::MavlinkEndpoint& fromApps,
                           const mav::MavlinkEndpoint& toAutopilot,
                           const mav::MavlinkEndpoint& toGcs)
  : drone(drone)
  , fromApps(fromApps)
  , toAutopilot(toAutopilot)
  , toGcs(toGcs)
{
}

void SerialBridge::pump()
{
  // Від автопілота та оператора до дрона
  for (const mavlink_message_t& msg : fromApps.poll()) {
    drone.send(msg);

    if (msg.msgid == MAVLINK_MSG_ID_COMMAND_LONG && mavlink_msg_command_long_get_command(&msg) == mav::kMissionCompleteCommandId) {
      MISSION_COMPLETE = true;
    }
  }

  // Від дрона до автопілота та GCS
  for (const mavlink_message_t& msg : drone.poll()) {
    toAutopilot.send(msg);
    toGcs.send(msg);
  }
}

bool SerialBridge::isMissionComplete() const
{
  return MISSION_COMPLETE;
}
