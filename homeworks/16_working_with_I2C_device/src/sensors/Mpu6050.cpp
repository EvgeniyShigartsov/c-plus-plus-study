#include "sensors/Mpu6050.hpp"

#include <array>

#include "Logger.hpp"

namespace {

int16_t combine(const uint8_t hi, const uint8_t lo)
{
  return static_cast<int16_t>((static_cast<uint16_t>(hi) << 8) | lo);
}
}  // namespace

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

bool Mpu6050::readAll(Readings& out) const
{
  std::array<uint8_t, 14> raw{};

  if (!bus.readBlock(Mpu6050::REG_MEASUREMENTS_START, raw.data(), raw.size())) {
    return false;
  }

  out.accelX = static_cast<float>(combine(raw[0], raw[1])) / Mpu6050::ACCEL_LSB_PER_G;
  out.accelY = static_cast<float>(combine(raw[2], raw[3])) / Mpu6050::ACCEL_LSB_PER_G;
  out.accelZ = static_cast<float>(combine(raw[4], raw[5])) / Mpu6050::ACCEL_LSB_PER_G;

  out.tempC = static_cast<float>(combine(raw[6], raw[7])) / 340.0f + 36.53f;

  out.gyroX = static_cast<float>(combine(raw[8], raw[9])) / Mpu6050::GYRO_LSB_PER_DPS;
  out.gyroY = static_cast<float>(combine(raw[10], raw[11])) / Mpu6050::GYRO_LSB_PER_DPS;
  out.gyroZ = static_cast<float>(combine(raw[12], raw[13])) / Mpu6050::GYRO_LSB_PER_DPS;

  return true;
}
