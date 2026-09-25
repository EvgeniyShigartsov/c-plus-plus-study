#include <cstdarg>
#include <cstdint>
#include <cstdio>

#include "core/Device.hpp"
#include "core/Reading.hpp"
#include "EspAdcReader.hpp"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "hal/adc_types.h"

namespace {

constexpr TickType_t kLoopPeriod = pdMS_TO_TICKS(500);  // 2гц

constexpr adc_channel_t kPhotoresistorChannel = ADC_CHANNEL_3;

void deviceLog(const char* format, ...)
{
  std::printf("[LOG]: ");

  va_list args;
  va_start(args, format);
  std::vprintf(format, args);
  va_end(args);

  std::printf("\n");
}

}  // namespace

extern "C" void app_main()
{
  deviceLog("boot");

  Device device;
  const EspAdcReader photoresistor(kPhotoresistorChannel);
  char lineBuf[64];

  TickType_t lastWakeTime = xTaskGetTickCount();

  while (true) {
    const uint32_t tMs = static_cast<uint32_t>(esp_timer_get_time() / 1000);
    const Reading reading = photoresistor.read();

    if (device.onTick(lineBuf, sizeof(lineBuf), tMs, reading)) {
      std::printf("%s\n", lineBuf);
    }

    xTaskDelayUntil(&lastWakeTime, kLoopPeriod);
  }
}
