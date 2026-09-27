#include <sys/types.h>
#include <cstdint>
#include <cstdio>
#include <string_view>

#include "core/Device.hpp"
#include "core/Reading.hpp"
#include "driver/usb_serial_jtag.h"
#include "EspAdcReader.hpp"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "hal/adc_types.h"

constexpr adc_channel_t kSoundSensorChannel = ADC_CHANNEL_3;
constexpr uint32_t kDefaultPeriodMs = 500;
constexpr uint32_t kTicksToWait = 0;  // Скільки чекати якщо даних у UART каналі ще немає
constexpr uint32_t kMaxBytesReadPerCall = 1;

uint32_t microsecondsToMilliseconds(const int64_t microseconds)
{
  return static_cast<uint32_t>(microseconds / 1000);
}

extern "C" void app_main()
{
  usb_serial_jtag_driver_config_t jtagConfig = USB_SERIAL_JTAG_DRIVER_CONFIG_DEFAULT();
  usb_serial_jtag_driver_install(&jtagConfig);  // Відкриття UART каналу з консоллю

  std::printf("Device boot");

  Device device(kDefaultPeriodMs);
  const EspAdcReader soundSensor(kSoundSensorChannel);
  char lineBuf[64];

  char commandBuf[64];
  size_t commandLength = 0;

  TickType_t lastWakeTime = xTaskGetTickCount();

  while (true) {
    const uint32_t tMs = microsecondsToMilliseconds(esp_timer_get_time());
    const Reading reading = soundSensor.read();

    if (device.onTick(lineBuf, sizeof(lineBuf), tMs, reading)) {
      std::printf("%s\n", lineBuf);
    }

    uint8_t rxByte = 0;

    while (usb_serial_jtag_read_bytes(&rxByte, kMaxBytesReadPerCall, kTicksToWait) > 0) {
      const char character = static_cast<char>(rxByte);

      if (character == '\n') {
        if (commandLength > 0) {
          char ackBuf[64];

          const bool isAckReady = device.onCommand(std::string_view(commandBuf, commandLength), ackBuf, sizeof(ackBuf));
          if (isAckReady) {
            std::printf("%s\n", ackBuf);
          }
          commandLength = 0;
        }
      }
      else if (commandLength < sizeof(commandBuf)) {
        commandBuf[commandLength] = character;
        commandLength++;
      }
    }

    // Період тепер керований командою p N (Device::onCommand), а не сталою величиною
    xTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(device.periodMs()));
  }
}
