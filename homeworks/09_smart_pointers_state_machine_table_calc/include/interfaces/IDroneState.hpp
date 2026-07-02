#pragma once
#include <memory>
#include "types.hpp"

// NOLINTNEXTLINE(cppcoreguidelines-special-member-functions)
class IDroneState {
public:
  virtual ~IDroneState() = default;

  virtual std::unique_ptr<IDroneState> execute(Simulation& simulation) = 0;
  virtual float getManeuverReadyTime(const Simulation& sim) = 0;

  [[nodiscard]] virtual const char* name() const = 0;
};
