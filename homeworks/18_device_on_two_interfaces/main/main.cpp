#include <cstdarg>
#include <cstdint>
#include <cstdio>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace {

constexpr TickType_t kLoopPeriod = pdMS_TO_TICKS(500);

void deviceLog(const char* format, ...)
{
  std::printf("[LOG]: [HM18 DEVICE]: ");

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

  TickType_t lastWakeTime = xTaskGetTickCount();
  uint32_t tick = 0;

  while (true) {
    // Тимчасова заглушка замість реального виміру
    deviceLog("tick %u", static_cast<unsigned>(tick++));

    xTaskDelayUntil(&lastWakeTime, kLoopPeriod);
  }
}
