#pragma once
#include <memory>
#include "interfaces/IDroneState.hpp"
#include "states/StateAccelerating.hpp"
#include "types.hpp"

class StateTurning : public IDroneState {
public:
  std::unique_ptr<IDroneState> execute(Simulation& sim) override
  {
    sim.dirToFire > sim.CURRENT_DIR ? sim.CURRENT_DIR += sim.dc.angularSpeed* sim.dc.simTimeStep
                                    : sim.CURRENT_DIR -= sim.dc.angularSpeed * sim.dc.simTimeStep;

    sim.turningTimeLeft -= sim.dc.simTimeStep;

    if (sim.turningTimeLeft <= 0) {
      sim.CURRENT_DIR = sim.dirToFire;
      sim.CURRENT_STATE = ACCELERATING;
      return std::make_unique<StateAccelerating>();
    }

    return nullptr;
  }
  float getManeuverReadyTime(const Simulation& sim) override { return sim.turningTimeLeft; }
  [[nodiscard]] const char* name() const override { return "Turning"; }
};
