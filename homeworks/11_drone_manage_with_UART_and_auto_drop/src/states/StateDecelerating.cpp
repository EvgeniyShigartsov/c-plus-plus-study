#include <memory>
#include <utility>
#include "states/StateDecelerating.hpp"
#include "interfaces/IDroneState.hpp"
#include "states/StateStopped.hpp"
#include "types.hpp"

std::pair<std::unique_ptr<IDroneState>, DroneCommand> StateDecelerating::execute(Simulation& sim)
{
  DroneCommand command{
    .state = DroneState::Decelerating,
    .targetDir = sim.dirToFire,
  };

  if (sim.CURRENT_SPEED <= 0) {
    sim.CURRENT_SPEED = 0.0f;

    command.state = DroneState::Stopped;
    return {std::make_unique<StateStopped>(), command};
  }

  return {nullptr, command};
}

float StateDecelerating::getManeuverReadyTime(const Simulation& sim)
{
  return sim.CURRENT_SPEED / sim.droneAcceleration;
}

const char* StateDecelerating::name() const
{
  return "Decelerating";
}