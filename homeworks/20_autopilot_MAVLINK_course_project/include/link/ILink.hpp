#pragma once
#include <sys/types.h>
#include <cstddef>
#include <cstdint>

// Транспорт для MAVLink-кадрів, UDP на хості, UART на залізі
// NOLINTNEXTLINE(cppcoreguidelines-special-member-functions)
class ILink {
public:
  [[nodiscard]] virtual bool isOpen() const = 0;

  virtual void sendFrame(const uint8_t* buf, const size_t len) const = 0;

  // результат: -1 - даних поки нема, 0 - лінк закрито або помилка, >0 - скільки байтів прочитано
  virtual ssize_t receive(uint8_t* buf, const size_t capacity) const = 0;

  virtual ~ILink() = default;
};
