#include <memory>
#include "interfaces/IDroneState.hpp"
#include "states/StateDecelerating.hpp"
#include "types.hpp"
#include "states/StateMoving.hpp"

std::unique_ptr<IDroneState> StateMoving::execute(Simulation &sim)
{
  if (sim.deltaAngle > sim.dc.turnThreshold) {
    sim.updateDroneXY();

    return std::make_unique<StateDecelerating>();
  }

  sim.CURRENT_DIR = sim.dirToFire;
  sim.updateDroneXY();

  return nullptr;
}

float StateMoving::getManeuverReadyTime(const Simulation &sim)
{
  return sim.dc.v0 / sim.droneAcceleration;
}

const char *StateMoving::name() const
{
  return "Moving";
}