#pragma once
#include "types.hpp"

class DronePhysics {
public:
  explicit DronePhysics(const DroneConfig& config);

  void setControl(const ControlSignal& control);
  void stepPhysics(const float deltaTime);

  [[nodiscard]] DroneTelemetry getTelemetry() const;

private:
  Coord CURRENT_POS;
  float CURRENT_SPEED = 0.0f;
  float CURRENT_DIR = 0.0f;
  float timeSinceStart = 0.0f;

  DroneConfig config;
  float droneAcceleration = 0.0f;
  ControlSignal currentControl{};
};
