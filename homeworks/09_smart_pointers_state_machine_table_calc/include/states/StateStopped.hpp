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
      return std::make_unique<StateTurning>();
    }

    return std::make_unique<StateAccelerating>();
  }
  float getManeuverReadyTime(const Simulation& /* sim */) override { return 0.0f; }
  [[nodiscard]] const char* name() const override { return "Stopped"; }
};
