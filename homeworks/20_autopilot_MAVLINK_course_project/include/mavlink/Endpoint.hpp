#pragma once
#include <vector>

#include <common/mavlink.h>

#include "link/ILink.hpp"

namespace mav {

class MavlinkEndpoint {
public:
  MavlinkEndpoint(const ILink& link, const mavlink_channel_t rxChannel);

  // Відправляємо mavlink повідомлення
  void send(const mavlink_message_t& msg) const;

  // Вичитуємо mavlink повідомлення
  [[nodiscard]] std::vector<mavlink_message_t> poll() const;

private:
  // NOLINTNEXTLINE(cppcoreguidelines-avoid-const-or-ref-data-members)
  const ILink& link;
  mavlink_channel_t rxChannel;
};

}  // namespace mav
