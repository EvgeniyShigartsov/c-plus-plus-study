#include <memory>
#include "states/StateDecelerating.hpp"
#include "interfaces/IDroneState.hpp"
#include "states/StateStopped.hpp"
#include "types.hpp"

std::unique_ptr<IDroneState> StateDecelerating::execute(Simulation& sim)
{
  sim.updateDroneXY();
  sim.CURRENT_SPEED -= sim.droneAcceleration * sim.dc.simTimeStep;

  if (sim.CURRENT_SPEED <= 0) {
    sim.CURRENT_SPEED = 0.0f;
    sim.turningTimeLeft = sim.deltaAngle / sim.dc.angularSpeed;

    return std::make_unique<StateStopped>();
  }

  return nullptr;
}

float StateDecelerating::getManeuverReadyTime(const Simulation& sim)
{
  return sim.CURRENT_SPEED / sim.droneAcceleration;
}

const char* StateDecelerating::name() const
{
  return "Decelerating";
}