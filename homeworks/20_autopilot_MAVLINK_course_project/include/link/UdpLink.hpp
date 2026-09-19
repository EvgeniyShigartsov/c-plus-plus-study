#pragma once
#include <netinet/in.h>
#include <sys/types.h>
#include <cstdint>
#include <string>

#include "link/ILink.hpp"

// NOLINTNEXTLINE(cppcoreguidelines-special-member-functions)
class UdpLink : public ILink {
public:
  UdpLink(const std::string& host, const uint16_t port, const uint16_t ownPort = 0);
  ~UdpLink() override;

  [[nodiscard]] bool isOpen() const override;

  void sendFrame(const uint8_t* buf, const size_t len) const override;

  ssize_t receive(uint8_t* buf, const size_t capacity) const override;

private:
  int fileDescriptor = -1;
  sockaddr_in destination{};
  void cleanup();
};
