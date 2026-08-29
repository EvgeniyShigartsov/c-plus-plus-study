#pragma once
#include <chrono>
#include <cstdint>

class UdpLink;

class MavlinkDrop {
public:
  explicit MavlinkDrop(const UdpLink& udp);

  // Старт скиду
  void begin(const double lat, const double lon, const float altitude);

  // Відправляти повідомлення про скид повторно якщо не дійшло з першої спроби, але не більше ніж MAX_ATTEMPTS разів.
  void resendIfFailed();

  // Обробка байтів які прийшли по UDP, пошук COMMAND_ACK
  void feed(const uint8_t* buf, const int len);

  [[nodiscard]] bool started() const;
  [[nodiscard]] bool finished() const;
  [[nodiscard]] bool acked() const;

private:
  void sendCommand();

  // NOLINTNEXTLINE(cppcoreguidelines-avoid-const-or-ref-data-members)
  const UdpLink& udp;

  double dropLat = 0.0;
  double dropLon = 0.0;
  float dropAltitude = 0.0f;

  int attempts = 0;
  bool isFinished = false;
  bool isAcked = false;
  std::chrono::steady_clock::time_point lastSend;
};
