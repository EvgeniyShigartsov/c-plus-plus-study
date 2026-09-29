#pragma once
#include <sys/types.h>
#include <cstddef>
#include <cstdint>

#include "driver/uart.h"
#include "link/ILink.hpp"

// NOLINTNEXTLINE(cppcoreguidelines-special-member-functions)
class EspUartLink : public ILink {
public:
  EspUartLink(const uart_port_t port, const int txPin, const int rxPin, const int baudRate);
  ~EspUartLink() override;

  [[nodiscard]] bool isOpen() const override;

  // Кладе кадр у буфер передачі драйвера; якщо буфер повний - драйвер чекає, поки звільниться місце
  void sendFrame(const uint8_t* buf, const size_t len) const override;

  // Неблокуюче читання: -1 - даних поки нема, 0 - помилка, >0 - скільки байтів прочитано
  ssize_t receive(uint8_t* buf, const size_t capacity) const override;

private:
  uart_port_t port;
  bool opened = false;
};
