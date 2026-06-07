#pragma once
#include "interfaces/IDroneState.hpp"
#include "types.hpp"
#include "states/StateAccelerating.hpp"
#include "states/StateTurning.hpp"

class StateStopped : public IDroneState {
public:
  std::unique_ptr<IDroneState> execute(Simulation& sim) override
  {
    if (sim.deltaAngle > sim.dc.turnThreshold) {
      sim.CURRENT_STATE = TURNING;
      return std::make_unique<StateTurning>();
    }

    sim.CURRENT_STATE = ACCELERATING;
    return std::make_unique<StateAccelerating>();
  }
  [[nodiscard]] const char* name() const override { return "Stopped"; }
};
