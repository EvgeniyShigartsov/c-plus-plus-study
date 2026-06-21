#pragma once
#include "types.hpp"

class DronePhysics {
public:
  DronePhysics(const DroneConfig& config);

  void pushCommand(const DroneCommand& cmd);
  void stepPhysics(const float deltaTime);

  DroneTelemetry getTelemetry() const;

private:
  Coord CURRENT_POS;
  float CURRENT_SPEED = 0.0f;
  float CURRENT_DIR = 0.0f;
  float timeSinceStart = 0.0f;

  DroneCommand currentCommand{};
  DroneConfig config;

  float droneAcceleration = 0.0f;

  void updateDroneXY(const float deltaTime);
};