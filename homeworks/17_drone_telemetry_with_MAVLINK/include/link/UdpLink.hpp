#pragma once
#include <cstdint>
#include <string>

// NOLINTNEXTLINE(cppcoreguidelines-special-member-functions)
class UdpLink {
public:
  UdpLink(const std::string& host, const uint16_t port);
  ~UdpLink();

  [[nodiscard]] bool isOpen() const;

  void sendFrame(const uint8_t* buf, const size_t len) const;

  int receive(uint8_t* buf, const size_t capacity) const;

private:
  int fileDescriptor = -1;
};
