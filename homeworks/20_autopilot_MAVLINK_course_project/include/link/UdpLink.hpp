#pragma once
#include <netinet/in.h>
#include <sys/types.h>
#include <cstdint>
#include <string>

// NOLINTNEXTLINE(cppcoreguidelines-special-member-functions)
class UdpLink {
public:
  UdpLink(const std::string& host, const uint16_t port, const uint16_t ownPort = 0);
  ~UdpLink();

  [[nodiscard]] bool isOpen() const;

  void sendFrame(const uint8_t* buf, const size_t len) const;

  ssize_t receive(uint8_t* buf, const size_t capacity) const;

private:
  int fileDescriptor = -1;
  sockaddr_in destination{};
  void cleanup();
};
