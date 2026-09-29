#pragma once
#include <sys/types.h>
#include <cstdint>
#include <string>

#include "link/ILink.hpp"

// NOLINTNEXTLINE(cppcoreguidelines-special-member-functions)
class SerialLink : public ILink {
public:
  SerialLink(const std::string& device, const int baudRate);
  ~SerialLink() override;

  [[nodiscard]] bool isOpen() const override;

  void sendFrame(const uint8_t* buf, const size_t len) const override;

  ssize_t receive(uint8_t* buf, const size_t capacity) const override;

private:
  int fileDescriptor = -1;
  void cleanup();
};
