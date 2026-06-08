#pragma once
#include "interfaces/IDroneState.hpp"
#include "states/StateStopped.hpp"
#include "types.hpp"

class StateDecelerating : public IDroneState {
public:
  std::unique_ptr<IDroneState> execute(Simulation& sim) override
  {
    sim.CURRENT_SPEED -= sim.droneAcceleration * sim.dc.simTimeStep;
    sim.updateDroneXY();

    if (sim.CURRENT_SPEED <= 0) {
      sim.CURRENT_SPEED = 0.0f;
      sim.turningTimeLeft = sim.deltaAngle / sim.dc.angularSpeed;

      return std::make_unique<StateStopped>();
    }

    return nullptr;
  }

  float getManeuverReadyTime(const Simulation& sim) override { return sim.CURRENT_SPEED / sim.droneAcceleration; }

  [[nodiscard]] const char* name() const override { return "Decelerating"; }
};
