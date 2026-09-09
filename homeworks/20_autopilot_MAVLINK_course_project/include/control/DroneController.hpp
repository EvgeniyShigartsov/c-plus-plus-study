#pragma once
#include "types.hpp"

class DroneController {
public:
  DroneController(const DroneConfig& config);

  ControlSignal compute(const DroneCommand& cmd, const DroneTelemetry& telemetry) const;

private:
  float v0 = 0.0f;
  float angularSpeed = 0.0f;
  float droneAcceleration = 0.0f;
  float timeStep = 0.0f;
};
