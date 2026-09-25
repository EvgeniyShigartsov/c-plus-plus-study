#include "mavlink/Endpoint.hpp"
#include <sys/types.h>

#include <array>
#include <cstdint>

namespace mav {

MavlinkEndpoint::MavlinkEndpoint(const ILink& link, const mavlink_channel_t rxChannel)
  : link(link)
  , rxChannel(rxChannel)
{
}

void MavlinkEndpoint::send(const mavlink_message_t& msg) const
{
  std::array<uint8_t, MAVLINK_MAX_PACKET_LEN> buffer{};
  const uint16_t length = mavlink_msg_to_send_buffer(buffer.data(), &msg);
  link.sendFrame(buffer.data(), length);
}

std::vector<mavlink_message_t> MavlinkEndpoint::poll() const
{
  std::vector<mavlink_message_t> received;
  std::array<uint8_t, MAVLINK_MAX_PACKET_LEN> buffer{};

  while (true) {
    const ssize_t bytesRead = link.receive(buffer.data(), buffer.size());
    if (bytesRead <= 0) {
      break;
    }

    mavlink_message_t msg{};
    mavlink_status_t status{};
    for (ssize_t i = 0; i < bytesRead; i++) {
      if (mavlink_parse_char(rxChannel, buffer[i], &msg, &status) != 0) {
        received.push_back(msg);
      }
    }
  }

  return received;
}

}  // namespace mav
