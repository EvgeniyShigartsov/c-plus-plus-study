#include "states/StateAccelerating.hpp"
#include "interfaces/IDroneState.hpp"
#include "states/StateMoving.hpp"
#include "types.hpp"

std::unique_ptr<IDroneState> StateAccelerating::execute(Simulation& sim)
{
  sim.CURRENT_SPEED += sim.droneAcceleration * sim.dc.simTimeStep;

  if (sim.CURRENT_SPEED >= sim.dc.v0) {
    sim.CURRENT_SPEED = sim.dc.v0;
    sim.updateDroneXY();
    return std::make_unique<StateMoving>();
  }
  sim.updateDroneXY();

  return nullptr;
}

float StateAccelerating::getManeuverReadyTime(const Simulation& sim)
{
  return sim.CURRENT_SPEED / sim.droneAcceleration;
}

const char* StateAccelerating::name() const
{
  return "Accelerating";
}