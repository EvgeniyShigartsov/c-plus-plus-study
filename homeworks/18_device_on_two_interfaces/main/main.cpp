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
constexpr uint32_t kTicksToWait = 0;  // Скільки чекати якщо даних у каналі ще немає
constexpr uint32_t kMaxBytesReadPerCall = 1;

uint32_t microsecondsToMilliseconds(const int64_t microseconds)
{
  return static_cast<uint32_t>(microseconds / 1000);
}

uint64_t millisecondsToMicroseconds(const uint32_t milliseconds)
{
  return static_cast<uint64_t>(milliseconds) * 1000;
}

void onTick(void* arg)
{
  xTaskNotifyGive(static_cast<TaskHandle_t>(arg));  // Розбудити головну задачу
}

extern "C" void app_main()
{
  usb_serial_jtag_driver_config_t jtagConfig = USB_SERIAL_JTAG_DRIVER_CONFIG_DEFAULT();
  usb_serial_jtag_driver_install(&jtagConfig);  // Відкриття каналу з консоллю

  std::printf("Device boot\n");

  Device device(kDefaultPeriodMs);
  const EspAdcReader soundSensor(kSoundSensorChannel);
  char lineBuf[64];

  char commandBuf[64];
  size_t commandLength = 0;

  const TaskHandle_t mainTask = xTaskGetCurrentTaskHandle();

  esp_timer_create_args_t timerArgs{};
  timerArgs.callback = onTick;  // Колбек який буде виконаний
  timerArgs.arg = mainTask;     // номер (дескриптор) поточної задачі
  timerArgs.name = "tick";

  esp_timer_handle_t timerHandle = nullptr;
  esp_timer_create(&timerArgs, &timerHandle);
  esp_timer_start_periodic(timerHandle, millisecondsToMicroseconds(device.periodMs()));

  while (true) {
    // Заснути до наступного спрацювання таймера
    ulTaskNotifyTake(pdTRUE,  // обнулити внутрішній лічильник до 0 після опрацювання сповіщення
                     portMAX_DELAY  // чекати без таймауту
    );

    const uint32_t tMs = microsecondsToMilliseconds(esp_timer_get_time());
    const Reading reading = soundSensor.read();

    if (device.onTick(lineBuf, sizeof(lineBuf), tMs, reading)) {
      std::printf("%s\n", lineBuf);
    }

    uint8_t rxByte = 0;

    while (usb_serial_jtag_read_bytes(&rxByte, kMaxBytesReadPerCall, kTicksToWait) > 0) {
      const char character = static_cast<char>(rxByte);

      if (character == '\n' || character == '\r') {
        if (commandLength > 0) {
          char ackBuf[64];

          const uint32_t periodBeforeCommand = device.periodMs();
          const bool isAckReady = device.onCommand(std::string_view(commandBuf, commandLength), ackBuf, sizeof(ackBuf));
          if (isAckReady) {
            std::printf("%s\n", ackBuf);
          }

          // Перезапуск таймеру після зміни періоду
          if (device.periodMs() != periodBeforeCommand) {
            esp_timer_restart(timerHandle, millisecondsToMicroseconds(device.periodMs()));
          }

          commandLength = 0;
        }
      }
      else if (commandLength < sizeof(commandBuf)) {
        commandBuf[commandLength] = character;
        commandLength++;
      }
    }
  }
}
