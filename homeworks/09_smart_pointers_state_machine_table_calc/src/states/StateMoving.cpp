#include "interfaces/IDroneState.hpp"
#include "types.hpp"
#include "states/StateMoving.hpp"

std::unique_ptr<IDroneState> StateMoving::execute(Simulation &sim)
{
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