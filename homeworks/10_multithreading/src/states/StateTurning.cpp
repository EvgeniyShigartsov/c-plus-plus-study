#include <memory>
#include <utility>
#include "states/StateTurning.hpp"
#include "interfaces/IDroneState.hpp"
#include "states/StateAccelerating.hpp"
#include "MathUtils.hpp"
#include "types.hpp"

std::pair<std::unique_ptr<IDroneState>, DroneCommand> StateTurning::execute(Simulation& sim)
{
  const float turnDelta = normalizeAngle(sim.dirToFire - sim.CURRENT_DIR);

  DroneCommand command{
    .state = DroneState::Turning,
    .angleSpeed = turnDelta > 0.0f ? sim.dc.angularSpeed : -sim.dc.angularSpeed,
    .targetDir = sim.dirToFire,
  };

  sim.turningTimeLeft -= sim.dc.simTimeStep;

  if (sim.turningTimeLeft <= 0.0f) {
    command.state = DroneState::Accelerating;
    command.angleSpeed = 0.0f;

    return {std::make_unique<StateAccelerating>(), command};
  }

  return {nullptr, command};
}

float StateTurning::getManeuverReadyTime(const Simulation& sim)
{
  return sim.turningTimeLeft;
}

const char* StateTurning::name() const
{
  return "Turning";
}