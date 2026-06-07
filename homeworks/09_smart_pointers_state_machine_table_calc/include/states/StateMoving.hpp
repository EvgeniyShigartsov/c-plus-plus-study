#pragma once
#include "interfaces/IDroneState.hpp"
#include "types.hpp"

class StateMoving : public IDroneState {
public:
  std::unique_ptr<IDroneState> execute(Simulation& sim) override
  {
    sim.updateDroneXY();

    return nullptr;
  }
  [[nodiscard]] const char* name() const override { return "Moving"; }
};
