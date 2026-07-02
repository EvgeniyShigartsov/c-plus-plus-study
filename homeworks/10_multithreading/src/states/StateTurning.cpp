#include <memory>
#include "states/StateTurning.hpp"
#include "interfaces/IDroneState.hpp"
#include "states/StateAccelerating.hpp"
#include "MathUtils.hpp"
#include "types.hpp"

std::unique_ptr<IDroneState> StateTurning::execute(Simulation& sim)
{
  const float turnDelta = normalizeAngle(sim.dirToFire - sim.CURRENT_DIR);
  const float turnRate = sim.dc.angularSpeed * sim.dc.simTimeStep;

  sim.CURRENT_DIR += (turnDelta > 0.0f ? turnRate : -turnRate);
  sim.turningTimeLeft -= sim.dc.simTimeStep;

  if (sim.turningTimeLeft <= 0.0f) {
    sim.CURRENT_DIR = sim.dirToFire;
    return std::make_unique<StateAccelerating>();
  }

  return nullptr;
}

float StateTurning::getManeuverReadyTime(const Simulation& sim)
{
  return sim.turningTimeLeft;
}

const char* StateTurning::name() const
{
  return "Turning";
}