#pragma once
#include <memory>
#include <utility>
#include "types.hpp"

// NOLINTNEXTLINE(cppcoreguidelines-special-member-functions)
class IDroneState {
public:
  virtual ~IDroneState() = default;

  virtual std::pair<std::unique_ptr<IDroneState>, DroneCommand> execute(Simulation& simulation) = 0;
  virtual float getManeuverReadyTime(const Simulation& sim) = 0;

  [[nodiscard]] virtual const char* name() const = 0;
};
