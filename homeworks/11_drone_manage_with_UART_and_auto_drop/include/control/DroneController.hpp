#pragma once
#include "types.hpp"

struct ControlSignal {
  float accel = 0.0f;     // прискорення вздовж курсу, [-1..1] (1 = повний газ, -1 = гальмо)
  float turnRate = 0.0f;  // швидкість повороту, [-1..1] (1 = макс. вліво, -1 = вправо)
};

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
