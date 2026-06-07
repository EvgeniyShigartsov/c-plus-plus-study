#pragma once
#include "interfaces/IDroneState.hpp"
#include "states/StateMoving.hpp"
#include "types.hpp"

class StateAccelerating : public IDroneState {
public:
  std::unique_ptr<IDroneState> execute(Simulation& sim) override
  {
    sim.CURRENT_SPEED += sim.droneAcceleration * sim.dc.simTimeStep;

    if (sim.CURRENT_SPEED >= sim.dc.v0) {
      sim.CURRENT_SPEED = sim.dc.v0;
      sim.CURRENT_STATE = MOVING;
      sim.updateDroneXY();
      return std::make_unique<StateMoving>();
    }
    sim.updateDroneXY();

    return nullptr;
  }
  float getManeuverReadyTime(const Simulation& sim) override { return sim.CURRENT_SPEED / sim.droneAcceleration; }
  [[nodiscard]] const char* name() const override { return "Accelerating"; }
};
