#pragma once
#include <memory>
#include "interfaces/IDroneState.hpp"
#include "types.hpp"

class IBallisticSolver;
class ITargetProvider;

// NOLINTNEXTLINE(cppcoreguidelines-special-member-functions)
class MissionProcessor {
private:
  Simulation sim;
  std::shared_ptr<ITargetProvider> targetProvider;
  std::unique_ptr<IBallisticSolver> ballisticSolver;

  DroneConfig droneInitialConfig{};
  float bombFlightTime = 0.0f;
  float h = 0.0f;
  int targetsCount = 0;
  std::unique_ptr<IDroneState> currentState;
  DroneCommand lastCommand{};

public:
  MissionProcessor(std::shared_ptr<ITargetProvider> provider, std::unique_ptr<IBallisticSolver> solver);
  [[nodiscard]] bool init(const DroneConfig& config);
  [[nodiscard]] bool hasNext() const;
  SimStep step(const DroneTelemetry& telemetry);
  DroneCommand getLastCommand() const;
  void changeSolver(std::unique_ptr<IBallisticSolver> solver);
  void reset();

  virtual ~MissionProcessor();
};