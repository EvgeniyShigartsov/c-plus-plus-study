#include "EspUartLink.hpp"

namespace {

constexpr int driverBufferSize = 1024;  // окремо на прийом і на передачу по 1кб

}  // namespace

EspUartLink::EspUartLink(const uart_port_t port, const int txPin, const int rxPin, const int baudRate)
  : port(port)
{
  uart_config_t config{};
  config.baud_rate = baudRate;          // Швидкість передачі даних
  config.data_bits = UART_DATA_8_BITS;  // Розмір кадру, 8 біт = 1 байт, + 1 старт біт на початку, + 1 стоп біт в кінці, всього 10 біт
  config.parity = UART_PARITY_DISABLE;  // Вимкнути перевірку парності бітів
  config.stop_bits = UART_STOP_BITS_1;  // Стоп біти в кінці кадру, на цьому біті лінія переходить у "спокійний стан" та може
                                        // "підготуватись" до наступного кадру
  config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;  // Вимкнути апаратне керування потоком
  config.source_clk = UART_SCLK_DEFAULT;        // Джерело тактування UART, UART_SCLK_DEFAULT = обирає ESP-IDF

  const esp_err_t installResult = uart_driver_install(port, driverBufferSize, driverBufferSize, 0, nullptr, 0);
  opened = installResult == ESP_OK;
  if (!opened) {
    return;
  }

  const esp_err_t paramsLoadResult = uart_param_config(port, &config);
  const esp_err_t setPinResult = uart_set_pin(port, txPin, rxPin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

  if (paramsLoadResult != ESP_OK || setPinResult != ESP_OK) {
    uart_driver_delete(port);
    opened = false;
  }
}

EspUartLink::~EspUartLink()
{
  if (opened) {
    uart_driver_delete(port);
  }
}

bool EspUartLink::isOpen() const
{
  return opened;
}

void EspUartLink::sendFrame(const uint8_t* buf, const size_t len) const
{
  uart_write_bytes(port, buf, len);
}

ssize_t EspUartLink::receive(uint8_t* buf, const size_t capacity) const
{
  const TickType_t timeout = 0;  // Нульовий тайм-аут - забрати те, що вже є в буфері драйвера, і не чекати
  const int bytesRead = uart_read_bytes(port, buf, static_cast<uint32_t>(capacity), timeout);

  if (bytesRead < 0) {
    return 0;
  }
  return bytesRead == 0 ? -1 : bytesRead;
}
