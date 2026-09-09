#include "link/MavlinkUtil.hpp"

#include <array>
#include <cmath>

#include "MathUtils.hpp"
#include "link/UdpLink.hpp"

namespace mavlinkUtil {

void send(const UdpLink& udp, const mavlink_message_t& msg)
{
  std::array<uint8_t, MAVLINK_MAX_PACKET_LEN> buffer{};
  const uint16_t len = mavlink_msg_to_send_buffer(buffer.data(), &msg);
  udp.sendFrame(buffer.data(), len);
}

// Конвертація математичного курсу (0 = схід, наліво) у компасний (0 = північ, направо) + нормалізація кута
float toNedYaw(float dir)
{
  return normalizeAngle(static_cast<float>(M_PI / 2.0) - dir);
}

}  // namespace mavlinkUtil
