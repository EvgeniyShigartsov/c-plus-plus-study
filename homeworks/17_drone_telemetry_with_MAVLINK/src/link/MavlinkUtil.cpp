#include "link/MavlinkUtil.hpp"

#include <array>

#include "link/UdpLink.hpp"

namespace mavlinkUtil {

void send(const UdpLink& udp, const mavlink_message_t& msg)
{
  std::array<uint8_t, MAVLINK_MAX_PACKET_LEN> buffer{};
  const uint16_t len = mavlink_msg_to_send_buffer(buffer.data(), &msg);
  udp.sendFrame(buffer.data(), len);
}

}  // namespace mavlinkUtil
