#pragma once
#include "interfaces/IDroneState.hpp"
#include "states/StateDecelerating.hpp"
#include "states/StateMoving.hpp"
#include "types.hpp"

class StateAccelerating : public IDroneState {
public:
  std::unique_ptr<IDroneState> execute(Simulation& sim) override
  {
    if (sim.deltaAngle > sim.dc.turnThreshold) {
      return std::make_unique<StateDecelerating>();
    }

    sim.CURRENT_DIR = sim.dirToFire;
    sim.CURRENT_SPEED += sim.droneAcceleration * sim.dc.simTimeStep;

    if (sim.CURRENT_SPEED >= sim.dc.v0) {
      sim.CURRENT_SPEED = sim.dc.v0;
      sim.updateDroneXY();
      return std::make_unique<StateMoving>();
    }
    sim.updateDroneXY();

    return nullptr;
  }
  float getManeuverReadyTime(const Simulation& sim) override { return sim.CURRENT_SPEED / sim.droneAcceleration; }
  [[nodiscard]] const char* name() const override { return "Accelerating"; }
};
