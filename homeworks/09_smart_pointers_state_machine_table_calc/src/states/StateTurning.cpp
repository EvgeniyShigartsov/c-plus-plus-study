#include <memory>
#include "states/StateTurning.hpp"
#include "interfaces/IDroneState.hpp"
#include "states/StateAccelerating.hpp"
#include "types.hpp"

std::unique_ptr<IDroneState> StateTurning::execute(Simulation& sim)
{
  sim.dirToFire > sim.CURRENT_DIR ? sim.CURRENT_DIR += sim.dc.angularSpeed* sim.dc.simTimeStep
                                  : sim.CURRENT_DIR -= sim.dc.angularSpeed * sim.dc.simTimeStep;

  sim.turningTimeLeft -= sim.dc.simTimeStep;

  if (sim.turningTimeLeft <= 0) {
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