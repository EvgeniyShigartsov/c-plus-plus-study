#pragma once
#include "interfaces/IDroneState.hpp"
#include "types.hpp"

class StateMoving : public IDroneState {
public:
  std::unique_ptr<IDroneState> execute(Simulation& simulation) override
  {
    //

    return std::make_unique<StateMoving>();
  }
  [[nodiscard]] const char* name() const override { return "Moving"; }
};
