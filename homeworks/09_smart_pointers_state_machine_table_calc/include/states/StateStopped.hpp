#pragma once
#include "interfaces/IDroneState.hpp"
#include "types.hpp"
#include "states/StateAccelerating.hpp"

class StateStopped : public IDroneState {
public:
  std::unique_ptr<IDroneState> execute(Simulation& simulation) override
  {
    //

    return std::make_unique<StateAccelerating>();
  }
  [[nodiscard]] const char* name() const override { return "Stopped"; }
};
