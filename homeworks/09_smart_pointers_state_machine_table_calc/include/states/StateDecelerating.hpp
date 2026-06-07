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
      sim.CURRENT_STATE = STOPPED;
      sim.turningTimeLeft = sim.deltaAngle / sim.dc.angularSpeed;

      return std::make_unique<StateStopped>();
    }

    return nullptr;
  }
  [[nodiscard]] const char* name() const override { return "Decelerating"; }
};
