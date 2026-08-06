#pragma once
#include <utility>
#include "interfaces/IDroneState.hpp"
#include "types.hpp"

class StateStopped : public IDroneState {
public:
  std::pair<std::unique_ptr<IDroneState>, DroneCommand> execute(Simulation &sim) override;
  float getManeuverReadyTime(const Simulation &sim) override;
  [[nodiscard]] const char *name() const override;
};
