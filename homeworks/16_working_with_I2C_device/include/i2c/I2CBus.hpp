#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

// NOLINTNEXTLINE(cppcoreguidelines-special-member-functions)
class I2CBus {
public:
  I2CBus(const std::string& device, const uint8_t address);
  ~I2CBus();

  bool isOpen() const;

  bool writeByte(const uint8_t reg, const uint8_t value) const;

  bool readBlock(const uint8_t reg, uint8_t* data, const size_t len) const;

private:
  int fd = -1;
};
