#include "sim/DronePhysics.hpp"

#include <algorithm>
#include <cmath>

namespace {

Coord movePos(const Coord& pos, float dir, float speed, float deltaTime)
{
  return pos + Coord{.x = std::cos(dir), .y = std::sin(dir)} * speed * deltaTime;
}
}  // namespace

DronePhysics::DronePhysics(const DroneConfig& config)
  : CURRENT_POS(config.startPos)
  , CURRENT_DIR(config.initialDir)
  , config(config)
  , droneAcceleration(config.v0 * config.v0 / (2.0f * config.accelerationPath))
{
}

void DronePhysics::setControl(const ControlSignal& control)
{
  currentControl = control;
}

void DronePhysics::stepPhysics(const float deltaTime)
{
  CURRENT_SPEED = std::clamp(CURRENT_SPEED + currentControl.accel * droneAcceleration * deltaTime, 0.0f, config.v0);
  CURRENT_DIR += currentControl.turnRate * config.angularSpeed * deltaTime;
  CURRENT_POS = movePos(CURRENT_POS, CURRENT_DIR, CURRENT_SPEED, deltaTime);
  timeSinceStart += deltaTime;
}

DroneTelemetry DronePhysics::getTelemetry() const
{
  return {.pos = CURRENT_POS, .speed = CURRENT_SPEED, .dir = CURRENT_DIR, .timeSinceStart = timeSinceStart};
}
