#pragma once

#include <cstdint>

#include "antidrone_turret/actuator_model.hpp"

namespace antidrone_turret {

enum class TargetState : std::uint8_t {
  kNone = 0,           // ціль не видима
  kLowConfidence = 1,  // ціль є але confidence нижче порога
  kLocked = 2,         // ціль видима і розпізнавання достатньо надійне
};

enum class ActionState : std::uint8_t {
  kIdle = 0,   // не публікувати нові команди наведення
  kTrack = 1,  // публікувати GimbalCommand і ServoCommand
};

enum class TriggerState : std::uint8_t {
  kSkip = 0,       // сервіс пострілу не викликати
  kRequested = 1,  // ціль близько і актуатор READY -> викликати сервіс
  kReloading = 2,  // ціль близько, але актуатор ще перезаряджається
};

enum class GimbalDirection : std::int8_t {
  kDown = -1,
  kCenter = 0,
  kUp = 1,
};

enum class ServoDirection : std::int8_t {
  kLeft = -1,
  kCenter = 0,
  kRight = 1,
};

// Спрощена модель кадру = 640x480, центр = 320/240
inline constexpr float kFrameCenterX = 320.0f;
inline constexpr float kFrameCenterY = 240.0f;

struct TargetInput {
  bool visible = false;
  float x = 0.0f;
  float y = 0.0f;
  float distance_m = 0.0f;
  float confidence = 0.0f;
};

struct TurretConfig {
  float confidence_threshold = 0.80f;
  float max_distance_m = 30.0f;
};

struct ServoCommandData {
  ServoDirection direction = ServoDirection::kCenter;
  float target_x = 0.0f;
  float error_x = 0.0f;
};

struct GimbalCommandData {
  GimbalDirection direction = GimbalDirection::kCenter;
  float target_y = 0.0f;
  float error_y = 0.0f;
};

struct TurretStatusData {
  TargetState target_state = TargetState::kNone;
  ActionState action = ActionState::kIdle;
  TriggerState trigger_state = TriggerState::kSkip;
  float confidence = 0.0f;
  float distance_m = 0.0f;
};

struct TurretDecision {
  TurretStatusData status = {};
  GimbalCommandData gimbal = {};
  ServoCommandData servo = {};
  bool should_aim = false;       // чи публікувати gimbal/servo команди
  bool request_trigger = false;  // чи викликати /actuator/trigger
};

// Оцінка якості цілі.
[[nodiscard]] inline TargetState evaluate_target(const TargetInput& target, const TurretConfig& config)
{
  if (!target.visible) {
    return TargetState::kNone;
  }
  if (target.confidence < config.confidence_threshold) {
    return TargetState::kLowConfidence;
  }
  return TargetState::kLocked;
}

// Команда yaw-серво з горизонтальної координати цілі.
[[nodiscard]] inline ServoCommandData compute_servo_command(const float x)
{
  const float error_x = x - kFrameCenterX;

  ServoDirection direction = ServoDirection::kCenter;
  if (error_x > 0.0f) {
    direction = ServoDirection::kRight;
  }
  else if (error_x < 0.0f) {
    direction = ServoDirection::kLeft;
  }

  return ServoCommandData{direction, x, error_x};
}

// Команда гімбала з вертикальної координати цілі.
[[nodiscard]] inline GimbalCommandData compute_gimbal_command(const float y)
{
  const float error_y = kFrameCenterY - y;

  GimbalDirection direction = GimbalDirection::kCenter;
  if (error_y > 0.0f) {
    direction = GimbalDirection::kUp;
  }
  else if (error_y < 0.0f) {
    direction = GimbalDirection::kDown;
  }

  return GimbalCommandData{direction, y, error_y};
}

// Рішення щодо пострілу за дистанцією та останнім станом актуатора.
[[nodiscard]] inline TriggerState decide_trigger(const float distance_m, const float max_distance_m, const ActuatorState actuator_state)
{
  if (distance_m > max_distance_m) {
    return TriggerState::kSkip;
  }
  return actuator_state == ActuatorState::kReady ? TriggerState::kRequested : TriggerState::kReloading;
}

// Cкладання повного рішення на один кадр /perception/target
[[nodiscard]] inline TurretDecision decide(const TargetInput& target, const ActuatorState actuator_state, const TurretConfig& config)
{
  TurretDecision decision = {};
  decision.status.confidence = target.confidence;
  decision.status.distance_m = target.distance_m;

  const TargetState target_state = evaluate_target(target, config);
  decision.status.target_state = target_state;

  // Ціль не придатна або не видима - без наведення, без пострілу
  if (target_state != TargetState::kLocked) {
    decision.status.action = ActionState::kIdle;
    decision.status.trigger_state = TriggerState::kSkip;
    return decision;
  }

  // Ціль захоплена: наводимо гімбал і серво
  decision.status.action = ActionState::kTrack;
  decision.should_aim = true;
  decision.servo = compute_servo_command(target.x);
  decision.gimbal = compute_gimbal_command(target.y);

  // Окремо вирішуємо щодо пострілу
  const TriggerState trigger_state = decide_trigger(target.distance_m, config.max_distance_m, actuator_state);
  decision.status.trigger_state = trigger_state;
  decision.request_trigger = (trigger_state == TriggerState::kRequested);

  return decision;
}

}  // namespace antidrone_turret
