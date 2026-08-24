#include "sensors/Mpu6050.hpp"

#include "Logger.hpp"

Mpu6050::Mpu6050(I2CBus& bus)
  : bus(bus)
{
}

bool Mpu6050::checkWhoAmI() const
{
  uint8_t id = 0;
  if (!bus.readBlock(Mpu6050::REG_WHO_AM_I, &id, 1)) {
    return false;
  }

  if (id != Mpu6050::EXPECTED_WHO_AM_I) {
    LOG("MPU-6050: unexpected WHO_AM_I: got " << static_cast<int>(id) << ", expected " << static_cast<int>(Mpu6050::EXPECTED_WHO_AM_I));
    return false;
  }

  return true;
}

bool Mpu6050::wake() const
{
  return bus.writeByte(Mpu6050::REG_PWR_MANAGEMENT_1, 0x00);
}
