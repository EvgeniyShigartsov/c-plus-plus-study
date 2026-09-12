#pragma once
#include <vector>

#include <common/mavlink.h>

#include "link/UdpLink.hpp"

namespace mav {

class MavlinkEndpoint {
public:
  MavlinkEndpoint(const UdpLink& udp, const mavlink_channel_t rxChannel);

  void send(const mavlink_message_t& msg) const;

  [[nodiscard]] std::vector<mavlink_message_t> poll() const;

private:
  // NOLINTNEXTLINE(cppcoreguidelines-avoid-const-or-ref-data-members)
  const UdpLink& udp;
  mavlink_channel_t rxChannel;
};

}  // namespace mav
