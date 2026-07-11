#include "control/DroneController.hpp"

#include <algorithm>
#include <cmath>

#include "MathUtils.hpp"

DroneController::DroneController(const DroneConfig& config)
  : v0(config.v0)
  , angularSpeed(config.angularSpeed)
  , droneAcceleration(powf(config.v0, 2) / (2 * config.accelerationPath))
  , timeStep(config.simTimeStep)
{
}

ControlSignal DroneController::compute(const DroneCommand& cmd, const DroneTelemetry& telemetry) const
{
  const bool wantsMove = (cmd.state == DroneState::Accelerating) || (cmd.state == DroneState::Moving);
  const float desiredSpeed = wantsMove ? v0 : 0.0f;

  const float speedError = desiredSpeed - telemetry.speed;
  const float accel = std::clamp(speedError / (droneAcceleration * timeStep), -1.0f, 1.0f);

  float turnRate = 0.0f;

  if (cmd.state != DroneState::Decelerating) {
    const float dirError = normalizeAngle(cmd.targetDir - telemetry.dir);
    turnRate = std::clamp(dirError / (angularSpeed * timeStep), -1.0f, 1.0f);
  }

  return {accel, turnRate};
}
