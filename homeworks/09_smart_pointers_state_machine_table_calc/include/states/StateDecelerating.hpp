#pragma once
#include "interfaces/IDroneState.hpp"
#include "types.hpp"

class StateDecelerating : public IDroneState {
public:
  std::unique_ptr<IDroneState> execute(Simulation& simulation) override
  {
    //

    return std::make_unique<StateDecelerating>();
  }
  [[nodiscard]] const char* name() const override { return "Decelerating"; }
};
