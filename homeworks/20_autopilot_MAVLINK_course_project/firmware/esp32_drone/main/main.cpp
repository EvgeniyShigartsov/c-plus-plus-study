#include <cstdarg>
#include <cstdint>
#include <cstdio>

#include "drone/DroneNode.hpp"
#include "drone/EmbeddedConfigs.hpp"
#include "EspUartLink.hpp"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "interfaces/IDroneOutput.hpp"
#include "mavlink/Codec.hpp"
#include "mavlink/Endpoint.hpp"

namespace {

// Номер конфігу (1..10) задається при збірці: idf.py build -DDRONE_CONFIG=N, див. main/CMakeLists.txt
static_assert(DRONE_CONFIG >= 1 && DRONE_CONFIG <= static_cast<int>(kEmbeddedDroneConfigs.size()), "DRONE_CONFIG поза межами");

constexpr const EmbeddedDroneConfig& kEmbeddedConfig = kEmbeddedDroneConfigs[DRONE_CONFIG - 1];
constexpr const DroneConfig& kConfig = kEmbeddedConfig.drone;
constexpr float kPhysicsTimeStep = kEmbeddedConfig.physicsTimeStep;
constexpr float kTimeScale = 1.0f;  // на залізі реальний час

constexpr uart_port_t kLinkUart = UART_NUM_1;  // Вільний UART канал
constexpr int kLinkTxPin = 1;                  // Фізичний пін 1
constexpr int kLinkRxPin = 2;                  // Фізичний пін 2
constexpr int kLinkBaudRate = 115200;          // Швидкість передачі даних аналогічна до PI

// Частота головного циклу = крок фізики (10 мс): команди від оператора мають оброблятись за <= 10 мс
constexpr TickType_t kLoopPeriod = pdMS_TO_TICKS(10);

// Лог телеметрії раз на 10 секунд (летить 10 на секунду)
constexpr uint32_t kTelemetryLogEvery = 100;

constexpr uint32_t kRestartDelayMs = 500;

void droneLog(const char* format, ...)
{
  std::printf("[LOG]: [ESP_32 DRONE]: ");

  va_list args;
  va_start(args, format);
  std::vprintf(format, args);
  va_end(args);

  std::printf("\n");
}

class DroneOutput : public IDroneOutput {
public:
  explicit DroneOutput(const mav::MavlinkEndpoint& endpoint)
    : endpoint(endpoint)
  {
  }

  void send(const mavlink_message_t& msg) override
  {
    endpoint.send(msg);

    if (msg.msgid != MAVLINK_MSG_ID_LOCAL_POSITION_NED) {
      return;
    }

    // Друк позиції в лог раз на 10 секунд (про всяк випадок)
    if (telemetryCount++ % kTelemetryLogEvery == 0) {
      const mav::LocalPositionNed position = mav::parse_local_position_ned(msg);
      droneLog("telemetry t=%u ms pos=(%.2f,%.2f) velocity=(%.2f,%.2f)",
               static_cast<unsigned>(position.time_boot_ms),
               static_cast<double>(position.north),
               static_cast<double>(position.east),
               static_cast<double>(position.velocity_north),
               static_cast<double>(position.velocity_east));
    }
  }

  void onDrop(const Coord& aimPoint, const float timeSinceStart) override
  {
    droneLog(
      "DROP t=%.4f aim=(%.3f,%.3f)", static_cast<double>(timeSinceStart), static_cast<double>(aimPoint.x), static_cast<double>(aimPoint.y));
  }

  void onMissionComplete() override { droneLog("mission complete notification received"); }

private:
  const mav::MavlinkEndpoint& endpoint;  // NOLINT(cppcoreguidelines-avoid-const-or-ref-data-members)
  uint32_t telemetryCount = 0;
};

float microsecondsToSeconds(const int64_t microseconds)
{
  return static_cast<float>(microseconds) / 1'000'000.0f;
}

}  // namespace

extern "C" void app_main()
{
  const EspUartLink link(kLinkUart, kLinkTxPin, kLinkRxPin, kLinkBaudRate);
  if (!link.isOpen()) {
    droneLog("Failed to open UART %d", static_cast<int>(kLinkUart));
    return;
  }
  const mav::MavlinkEndpoint endpoint(link, MAVLINK_COMM_1);

  DroneOutput output(endpoint);
  DroneNode droneNode(kConfig, kPhysicsTimeStep, kTimeScale, output);

  droneLog("MISSION STARTED with config %d %s position=(%.0f,%.0f)",
           DRONE_CONFIG,
           kEmbeddedConfig.name,
           static_cast<double>(kConfig.startPos.x),
           static_cast<double>(kConfig.startPos.y));

  int64_t lastUpdateMicroseconds = esp_timer_get_time();
  TickType_t lastWakeTime = xTaskGetTickCount();

  while (!droneNode.isMissionComplete()) {
    for (const mavlink_message_t& msg : endpoint.poll()) {
      droneNode.onMessage(msg);
    }

    const int64_t nowMicroseconds = esp_timer_get_time();  // Реальний час що минув

    const float realDeltaSec = microsecondsToSeconds(nowMicroseconds - lastUpdateMicroseconds);
    droneNode.update(realDeltaSec);
    lastUpdateMicroseconds = nowMicroseconds;

    // Приспати процес до наступної точки розкладу (обробки даних)
    xTaskDelayUntil(&lastWakeTime, kLoopPeriod);
  }

  droneLog("mission complete, restarting");
  vTaskDelay(pdMS_TO_TICKS(kRestartDelayMs));

  // Перезавантаження заліза для наступного тесту
  esp_restart();
}
