#pragma once
#include <cstdint>

#include <common/mavlink.h>

class UdpLink;

namespace mavlinkUtil {

constexpr uint8_t SYSID = 1;
constexpr uint8_t COMPID = MAV_COMP_ID_AUTOPILOT1;

void send(const UdpLink& udp, const mavlink_message_t& msg);

}  // namespace mavlinkUtil
