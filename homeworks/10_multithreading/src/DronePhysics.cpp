#include "DronePhysics.hpp"
#include <cmath>
#include "types.hpp"

DronePhysics::DronePhysics(const DroneConfig& config)
  : CURRENT_POS(config.startPos)
  , CURRENT_DIR(config.initialDir)
  , config(config)
  , droneAcceleration(powf(config.v0, 2) / (2 * config.accelerationPath))

{
}

void DronePhysics::pushCommand(const DroneCommand& cmd)
{
  currentCommand = cmd;
}

void DronePhysics::stepPhysics(const float deltaTime)
{
  switch (currentCommand.state) {
    case DroneState::Stopped:
      CURRENT_DIR = currentCommand.targetDir;
      timeSinceStart += deltaTime;
      break;

    case DroneState::Turning:
      CURRENT_DIR += currentCommand.angleSpeed * deltaTime;
      timeSinceStart += deltaTime;
      break;

    case DroneState::Accelerating:
      updateDroneXY(deltaTime);

      CURRENT_DIR = currentCommand.targetDir;
      CURRENT_SPEED += droneAcceleration * deltaTime;
      if (CURRENT_SPEED >= config.v0) {
        CURRENT_SPEED = config.v0;
      }
      timeSinceStart += deltaTime;
      break;

    case DroneState::Moving:
      CURRENT_DIR = currentCommand.targetDir;
      updateDroneXY(deltaTime);
      timeSinceStart += deltaTime;
      break;

    case DroneState::Decelerating:
      updateDroneXY(deltaTime);

      CURRENT_SPEED -= droneAcceleration * deltaTime;
      if (CURRENT_SPEED <= 0.0f) {
        CURRENT_SPEED = 0.0f;
      }
      timeSinceStart += deltaTime;
      break;
  }
}

DroneTelemetry DronePhysics::getTelemetry() const
{
  return {
    .pos = CURRENT_POS,
    .speed = CURRENT_SPEED,
    .dir = CURRENT_DIR,
    .timeSinceStart = timeSinceStart,
  };
}

void DronePhysics::updateDroneXY(const float deltaTime)
{
  CURRENT_POS = CURRENT_POS + Coord{cosf(CURRENT_DIR), sinf(CURRENT_DIR)} * CURRENT_SPEED * deltaTime;
}