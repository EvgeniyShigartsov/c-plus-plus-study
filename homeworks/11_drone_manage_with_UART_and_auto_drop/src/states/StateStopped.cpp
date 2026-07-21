#include <memory>
#include <utility>
#include "states/StateStopped.hpp"
#include "interfaces/IDroneState.hpp"
#include "types.hpp"
#include "states/StateAccelerating.hpp"
#include "states/StateTurning.hpp"

std::pair<std::unique_ptr<IDroneState>, DroneCommand> StateStopped::execute(Simulation &sim)
{
  DroneCommand command{
    .state = DroneState::Stopped,
    .angleSpeed = 0.0f,
    .targetDir = sim.dirToFire,
  };

  if (sim.deltaAngle > sim.dc.turnThreshold) {
    command.state = DroneState::Turning;
    return {std::make_unique<StateTurning>(), command};
  }

  command.state = DroneState::Accelerating;
  return {std::make_unique<StateAccelerating>(), command};
}

float StateStopped::getManeuverReadyTime(const Simulation & /* sim */)
{
  return 0.0f;
}

const char *StateStopped::name() const
{
  return "Stopped";
}