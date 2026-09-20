#include <cstdint>
#include <cstdio>

#include "drone/DroneNode.hpp"
#include "drone/EmbeddedConfigs.hpp"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "interfaces/IDroneOutput.hpp"
#include "mavlink/Codec.hpp"
#include "mavlink/Identity.hpp"

namespace {

// Номер конфігу (1..10) задається при збірці: idf.py build -DDRONE_CONFIG=N, див. main/CMakeLists.txt
static_assert(DRONE_CONFIG >= 1 && DRONE_CONFIG <= static_cast<int>(kEmbeddedDroneConfigs.size()), "DRONE_CONFIG поза межами");

constexpr const EmbeddedDroneConfig& kEmbeddedConfig = kEmbeddedDroneConfigs[DRONE_CONFIG - 1];
constexpr const DroneConfig& kConfig = kEmbeddedConfig.drone;
constexpr float kPhysicsTimeStep = kEmbeddedConfig.physicsTimeStep;
constexpr float kTimeScale = 1.0f;  // на залізі реальний час

// Друкування в консоль повідомлень ядра, замість відправки по UART
class ConsoleOutput : public IDroneOutput {
public:
  void send(const mavlink_message_t& msg) override
  {
    if (msg.msgid != MAVLINK_MSG_ID_LOCAL_POSITION_NED) {
      return;
    }

    if (telemetryCount++ % 10 == 0) {
      const mav::LocalPositionNed position = mav::parse_local_position_ned(msg);
      std::printf("time %u ms  position %.2f %.2f  velocity %.2f %.2f\n",
                  static_cast<unsigned>(position.time_boot_ms),
                  static_cast<double>(position.north),
                  static_cast<double>(position.east),
                  static_cast<double>(position.velocity_north),
                  static_cast<double>(position.velocity_east));
    }
  }

  void onDrop(const Coord& aimPoint, const float timeSinceStart) override
  {
    std::printf("drop at %.2f s  aim %.2f %.2f\n",
                static_cast<double>(timeSinceStart),
                static_cast<double>(aimPoint.x),
                static_cast<double>(aimPoint.y));
  }

  void onMissionComplete() override { std::printf("mission complete notification received\n"); }

private:
  uint32_t telemetryCount = 0;
};

}  // namespace

extern "C" void app_main()
{
  ConsoleOutput output;
  DroneNode droneNode(kConfig, kPhysicsTimeStep, kTimeScale, output);

  std::printf("esp32_drone started  config %d %s  position %.0f %.0f\n",
              DRONE_CONFIG,
              kEmbeddedConfig.name,
              static_cast<double>(kConfig.startPos.x),
              static_cast<double>(kConfig.startPos.y));

  // Даємо газ, як автопілот, щоб у консолі було видно рух
  droneNode.onMessage(mav::pack_radio_control_channels_override(mav::kAutopilot, mav::kVehicle, {.accel = 1.0f, .turnRate = 0.0f}));

  for (;;) {
    droneNode.update(0.1f);
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}
