#pragma once
#include "interfaces/IDroneState.hpp"
#include "states/StateDecelerating.hpp"
#include "types.hpp"

class StateMoving : public IDroneState {
public:
  std::unique_ptr<IDroneState> execute(Simulation &sim) override
  {
    if (sim.deltaAngle > sim.dc.turnThreshold) {
      return std::make_unique<StateDecelerating>();
    }

    sim.CURRENT_DIR = sim.dirToFire;
    sim.updateDroneXY();

    return nullptr;
  }
  float getManeuverReadyTime(const Simulation &sim) override { return sim.dc.v0 / sim.droneAcceleration; }
  [[nodiscard]] const char *name() const override { return "Moving"; }
};
