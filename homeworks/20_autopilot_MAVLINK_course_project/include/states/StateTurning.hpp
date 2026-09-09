#pragma once
#include <memory>
#include <utility>
#include "interfaces/IDroneState.hpp"
#include "types.hpp"

class StateTurning : public IDroneState {
public:
  std::pair<std::unique_ptr<IDroneState>, DroneCommand> execute(Simulation& sim) override;
  float getManeuverReadyTime(const Simulation& sim) override;
  [[nodiscard]] const char* name() const override;
};
