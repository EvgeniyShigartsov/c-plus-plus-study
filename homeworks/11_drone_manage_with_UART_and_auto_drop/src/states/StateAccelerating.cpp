#include <memory>
#include <utility>
#include "states/StateAccelerating.hpp"
#include "interfaces/IDroneState.hpp"
#include "states/StateDecelerating.hpp"
#include "states/StateMoving.hpp"
#include "types.hpp"

std::pair<std::unique_ptr<IDroneState>, DroneCommand> StateAccelerating::execute(Simulation& sim)
{
  DroneCommand command{
    .state = DroneState::Accelerating,
    .targetDir = sim.dirToFire,
  };

  if (sim.deltaAngle > sim.dc.turnThreshold) {
    command.state = DroneState::Decelerating;

    return {std::make_unique<StateDecelerating>(), command};
  }

  if (sim.CURRENT_SPEED >= sim.dc.v0) {
    command.state = DroneState::Moving;
    return {std::make_unique<StateMoving>(), command};
  }

  return {nullptr, command};
}

float StateAccelerating::getManeuverReadyTime(const Simulation& sim)
{
  return sim.CURRENT_SPEED / sim.droneAcceleration;
}

const char* StateAccelerating::name() const
{
  return "Accelerating";
}