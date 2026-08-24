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

  bool checkWhoAmI() const;

  bool wake() const;

private:
  // NOLINTNEXTLINE(cppcoreguidelines-avoid-const-or-ref-data-members)
  I2CBus& bus;
};
