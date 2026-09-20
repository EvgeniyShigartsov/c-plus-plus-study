#pragma once
#include <common/mavlink.h>

#include "interfaces/IDroneOutput.hpp"
#include "mavlink/Codec.hpp"
#include "mavlink/RadioControl.hpp"
#include "sim/DronePhysics.hpp"
#include "types.hpp"

// Загальна логіка дрону окремо, щоб можна було використати її на залізі, і просто на linux.
class DroneNode {
public:
  DroneNode(const DroneConfig& config, const float physicsTimeStep, const float timeScale, IDroneOutput& output);

  // Обробити одне вхідне повідомлення
  void onMessage(const mavlink_message_t& msg);

  // Просунути час, realDeltaSec - реальні секунди від попереднього виклику
  void update(const float realDeltaSec);

  [[nodiscard]] bool isMissionComplete() const;

  // Прийшла команда перезавантажитись, саме перезавантаження робить дрон
  [[nodiscard]] bool isRebootRequested() const;

private:
  DronePhysics physics;
  IDroneOutput& output;  // NOLINT(cppcoreguidelines-avoid-const-or-ref-data-members)
  float altitude;
  float simTimeStep;
  float physicsTimeStep;
  float timeScale;

  mav::RadioControlOverride lastAutopilotOverride{.roll = mav::kPwmNeutral, .throttle = mav::kPwmNeutral};
  mav::RadioControlOverride lastOperatorOverride{.roll = mav::kPwmNeutral, .throttle = mav::kPwmNeutral};

  bool dropped = false;
  bool MISSION_COMPLETE = false;
  bool REBOOT_REQUESTED = false;

  // Симульований час (реальні секунди * timeScale), який уже минув, але фізика його ще не відкрокувала
  float pendingPhysicsTimeSec = 0.0f;
  float secondsSinceHeartbeat;
  float nextTelemetryTime = 0.0f;

  void sendHeartbeat() const;
  void sendTelemetry() const;
};
