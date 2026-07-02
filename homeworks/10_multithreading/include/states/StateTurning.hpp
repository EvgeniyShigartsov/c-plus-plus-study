#pragma once
#include <memory>
#include "interfaces/IDroneState.hpp"
#include "types.hpp"

class StateTurning : public IDroneState {
public:
  std::unique_ptr<IDroneState> execute(Simulation& sim) override;
  float getManeuverReadyTime(const Simulation& sim) override;
  [[nodiscard]] const char* name() const override;
};
