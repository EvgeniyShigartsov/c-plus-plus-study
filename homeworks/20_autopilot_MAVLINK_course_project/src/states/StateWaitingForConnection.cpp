#include <memory>
#include <utility>
#include "states/StateWaitingForConnection.hpp"
#include "interfaces/IDroneState.hpp"
#include "states/StateStopped.hpp"
#include "types.hpp"

std::pair<std::unique_ptr<IDroneState>, DroneCommand> StateWaitingForConnection::execute(Simulation &sim)
{
  DroneCommand command{
    .state = DroneState::WaitingForConnection,
    .targetDir = sim.CURRENT_DIR,  // тримаємо поточний курс, при втраті зв'язку стара ціль вже не актуальна.
  };

  if (!sim.connectionLost) {
    command.state = DroneState::Stopped;
    return {std::make_unique<StateStopped>(), command};
  }

  return {nullptr, command};
}

float StateWaitingForConnection::getManeuverReadyTime(const Simulation & /* sim */)
{
  return 0.0f;
}

const char *StateWaitingForConnection::name() const
{
  return "WaitingForConnection";
}
