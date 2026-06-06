#pragma once
#include "interfaces/IDroneState.hpp"
#include "types.hpp"

class StateTurning : public IDroneState {
public:
  std::unique_ptr<IDroneState> execute(Simulation& simulation) override
  {
    //

    return std::make_unique<StateTurning>();
  }
  [[nodiscard]] const char* name() const override { return "Turning"; }
};
