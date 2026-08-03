#pragma once

namespace antidrone_turret {

inline constexpr auto kTargetTopic = "/perception/target";
inline constexpr auto kActuatorStatusTopic = "/actuator/status";
inline constexpr auto kGimbalCommandTopic = "/gimbal/cmd";
inline constexpr auto kServoCommandTopic = "/servo/cmd";
inline constexpr auto kTurretStatusTopic = "/turret/status";
inline constexpr auto kTriggerService = "/actuator/trigger";

}  // namespace antidrone_turret
