#include <cstdint>
#include <cstdlib>
#include <string>

#include "Logger.hpp"
#include "i2c/I2CBus.hpp"
#include "sensors/Mpu6050.hpp"

// NOLINTBEGIN(cppcoreguidelines-pro-bounds-pointer-arithmetic)
int main(int argc, char* argv[])
{
  if (argc < 3) {
    LOG("Usage: " << argv[0] << " no device or address provided.");
    LOG("Valid example: " << argv[0] << " /dev/i2c-1 0x68");
    return 1;
  }

  const std::string device = argv[1];
  const auto address = static_cast<uint8_t>(std::strtol(argv[2], nullptr, 0));

  I2CBus bus(device, address);
  if (!bus.isOpen()) {
    return 1;
  }

  Mpu6050 sensor(bus);
  if (!sensor.checkWhoAmI()) {
    return 1;
  }

  LOG("MPU-6050 found on " << device << ", address " << static_cast<int>(address));

  return 0;
}

// NOLINTEND(cppcoreguidelines-pro-bounds-pointer-arithmetic)
