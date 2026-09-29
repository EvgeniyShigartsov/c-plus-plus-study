#pragma once
#include <cmath>
#include <numbers>

// Конвенції систем координат, тут усі перетворення між звичною системою координат і MAVLink.
namespace mav {

// Нормалізація кута в [-pi, pi]
inline float wrap_pi(float angle)
{
  constexpr float pi = std::numbers::pi_v<float>;
  while (angle > pi) {
    angle -= 2.0f * pi;
  }
  while (angle < -pi) {
    angle += 2.0f * pi;
  }
  return angle;
}

// Математичний курс -> компасний курс
inline float ned_yaw(float dir)
{
  return wrap_pi(std::numbers::pi_v<float> * 0.5f - dir);
}

// Опорна точка прив'язки локальних метрів до GPS
constexpr double kRefLat = 50.4501;
constexpr double kRefLon = 30.5234;
constexpr double kMetersPerDeg = 111320.0;

struct Gps {
  double lat;
  double lon;
};

// Метри -> градуси.
inline Gps to_gps(float east, float north)
{
  return {
    .lat = kRefLat + static_cast<double>(north) / kMetersPerDeg,
    .lon = kRefLon + static_cast<double>(east) / (kMetersPerDeg * std::cos(kRefLat * std::numbers::pi / 180.0)),
  };
}

}  // namespace mav
