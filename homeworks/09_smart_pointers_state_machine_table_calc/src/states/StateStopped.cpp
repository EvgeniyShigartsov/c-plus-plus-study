#include <memory>
#include "states/StateStopped.hpp"
#include "interfaces/IDroneState.hpp"
#include "types.hpp"
#include "states/StateAccelerating.hpp"
#include "states/StateTurning.hpp"

std::unique_ptr<IDroneState> StateStopped::execute(Simulation &sim)
{
  if (sim.deltaAngle > sim.dc.turnThreshold) {
    sim.turningTimeLeft = sim.deltaAngle / sim.dc.angularSpeed;
    return std::make_unique<StateTurning>();
  }

  sim.CURRENT_DIR = sim.dirToFire;
  return std::make_unique<StateAccelerating>();
}

float StateStopped::getManeuverReadyTime(const Simulation & /* sim */)
{
  return 0.0f;
}

const char *StateStopped::name() const
{
  return "Stopped";
}