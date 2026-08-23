#include <memory>
#include <utility>
#include "interfaces/IDroneState.hpp"
#include "states/StateDecelerating.hpp"
#include "types.hpp"
#include "states/StateMoving.hpp"

std::pair<std::unique_ptr<IDroneState>, DroneCommand> StateMoving::execute(Simulation &sim)
{
  DroneCommand command{
    .state = DroneState::Moving,
    .targetDir = sim.dirToFire,
  };

  if (sim.deltaAngle > sim.dc.turnThreshold) {
    command.state = DroneState::Decelerating;
    return {std::make_unique<StateDecelerating>(), command};
  }

  return {nullptr, command};
}

float StateMoving::getManeuverReadyTime(const Simulation &sim)
{
  return sim.dc.v0 / sim.droneAcceleration;
}

const char *StateMoving::name() const
{
  return "Moving";
}