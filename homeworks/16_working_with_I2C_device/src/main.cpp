#include <unistd.h>

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
    LOG("Unable to check WHO_AM_I");
    return 1;
  }

  if (!sensor.wake()) {
    LOG("Unable to wakeup MPU-6050");
    return 1;
  }

  LOG("MPU-6050 found on " << device << ", address " << static_cast<int>(address));

  while (true) {
    Mpu6050::Readings r{};
    if (!sensor.readAll(r)) {
      LOG("Reading from MPU-6050 failed.");
      return 1;
    }

    LOG("accel g: " << r.accelX << ", " << r.accelY << ", " << r.accelZ << " | gyro dps: " << r.gyroX << ", " << r.gyroY << ", " << r.gyroZ
                    << " | temp C: " << r.tempC);

    usleep(200000);
  }
}

// NOLINTEND(cppcoreguidelines-pro-bounds-pointer-arithmetic)
