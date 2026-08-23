#include "i2c/I2CBus.hpp"

#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <unistd.h>

#include <array>
#include <cerrno>
#include <cstring>

#include "Logger.hpp"

// NOLINTBEGIN(cppcoreguidelines-pro-type-vararg)

I2CBus::I2CBus(const std::string& device, const uint8_t address)
  : fd(open(device.c_str(), O_RDWR))
{
  if (fd < 0) {
    LOG("I2C: failed to open " << device << ": " << std::strerror(errno));
    return;
  }

  if (ioctl(fd, I2C_SLAVE, address) < 0) {
    LOG("I2C: failed to select address " << address << " on " << device << ": " << std::strerror(errno));
    close(fd);
    fd = -1;
  }
}

I2CBus::~I2CBus()
{
  if (fd >= 0) {
    close(fd);
  }
}

bool I2CBus::isOpen() const
{
  return fd >= 0;
}

bool I2CBus::writeByte(const uint8_t reg, const uint8_t value) const
{
  if (!isOpen()) {
    return false;
  }

  const std::array<uint8_t, 2> out{reg, value};

  const ssize_t res = write(fd, out.data(), out.size());

  if (res != static_cast<ssize_t>(out.size())) {
    LOG("I2C: write to register " << reg << " failed: " << std::strerror(errno));
    return false;
  }

  return true;
}

bool I2CBus::readBlock(const uint8_t reg, uint8_t* data, const size_t len) const
{
  if (!isOpen()) {
    return false;
  }

  const ssize_t res = write(fd, &reg, 1);

  if (res != 1) {
    LOG("I2C: device did not respond to register select " << reg << ", : " << std::strerror(errno));
    return false;
  }

  const ssize_t got = read(fd, data, len);
  if (got != static_cast<ssize_t>(len)) {
    LOG("I2C: read from register " << reg << " was cut (got " << got << " of " << len << " bytes)");
    return false;
  }

  return true;
}

// NOLINTEND(cppcoreguidelines-pro-type-vararg)
