#pragma once

#include <cstdint>

#include "i2c/I2CBus.hpp"

// NOLINTNEXTLINE(cppcoreguidelines-special-member-functions)
class Mpu6050 {
public:
  Mpu6050(I2CBus& bus);
  ~Mpu6050() = default;

  static constexpr uint8_t DEFAULT_ADDRESS = 0x68;
  static constexpr uint8_t REG_WHO_AM_I = 0x75;
  static constexpr uint8_t EXPECTED_WHO_AM_I = 0x68;
  static constexpr uint8_t REG_PWR_MANAGEMENT_1 = 0x6B;
  // Перший регістр суцільного 14-байтного блоку accel+temp+gyro (ACCEL_XOUT_H у даташиті)
  static constexpr uint8_t REG_MEASUREMENTS_START = 0x3B;

  // Чутливість за замовчуванням з даташиту
  // https://cdn.sparkfun.com/datasheets/Sensors/Accelerometers/RM-MPU-6000A.pdf
  static constexpr float ACCEL_LSB_PER_G = 16384.0f;
  static constexpr float GYRO_LSB_PER_DPS = 131.0f;

  struct Readings {
    float accelX, accelY, accelZ;  // g, прискорення
    float gyroX, gyroY, gyroZ;     // °/с, швидкість обертання навколо осі x/y/z
    float tempC;                   // °C
  };

  bool checkWhoAmI() const;

  bool wake() const;

  bool readAll(Readings& out) const;

private:
  // NOLINTNEXTLINE(cppcoreguidelines-avoid-const-or-ref-data-members)
  I2CBus& bus;
};
