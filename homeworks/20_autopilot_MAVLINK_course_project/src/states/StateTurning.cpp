#include <memory>
#include <utility>
#include "states/StateTurning.hpp"
#include "interfaces/IDroneState.hpp"
#include "states/StateAccelerating.hpp"
#include "types.hpp"

std::pair<std::unique_ptr<IDroneState>, DroneCommand> StateTurning::execute(Simulation& sim)
{
  DroneCommand command{
    .state = DroneState::Turning,
    .targetDir = sim.dirToFire,
  };

  if (sim.deltaAngle <= sim.dc.turnThreshold) {
    command.state = DroneState::Accelerating;

    return {std::make_unique<StateAccelerating>(), command};
  }

  return {nullptr, command};
}

float StateTurning::getManeuverReadyTime(const Simulation& sim)
{
  return sim.deltaAngle / sim.dc.angularSpeed;
}

const char* StateTurning::name() const
{
  return "Turning";
}