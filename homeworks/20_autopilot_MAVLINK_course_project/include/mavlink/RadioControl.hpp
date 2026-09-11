#pragma once
#include <algorithm>
#include <cstdint>

// PWM-кодування радіокерування — формат пульта
// 1000 = мінімум, 1500 = нейтраль, 2000 = максимум, мікросекунди.
namespace mav {

constexpr uint16_t kPwmMin = 1000;
constexpr uint16_t kPwmNeutral = 1500;
constexpr uint16_t kPwmMax = 2000;
constexpr uint16_t kPwmHalfRange = 500;  // відстань від нейтралі до краю
constexpr uint16_t kPwmReleased = 0;     // канал лишається оператору

// Нормалізація [-1..1] -> PWM [1000..2000]
inline uint16_t normalized_to_pwm(float value)
{
  const float clamped = std::clamp(value, -1.0f, 1.0f);
  return static_cast<uint16_t>(kPwmNeutral + clamped * kPwmHalfRange);
}

// Нормалізація PWM [1000..2000] -> [-1..1]
inline float pwm_to_normalized(uint16_t pwm)
{
  // Команди нема
  if (pwm == kPwmReleased) {
    return 0.0f;
  }

  const uint16_t clamped = std::clamp(pwm, kPwmMin, kPwmMax);

  return (static_cast<float>(clamped) - kPwmNeutral) / kPwmHalfRange;
}

}  // namespace mav
