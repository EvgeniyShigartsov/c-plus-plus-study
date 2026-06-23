#pragma once
#include <memory>
#include <vector>
#include "DronePhysics.hpp"
#include "interfaces/IDroneState.hpp"
#include "types.hpp"
#include "interfaces/IBallisticSolver.hpp"
#include "interfaces/ITargetProvider.hpp"
#include "interfaces/IConfigLoader.hpp"

// NOLINTNEXTLINE(cppcoreguidelines-special-member-functions)
class MissionProcessor {
private:
  Simulation sim;
  std::shared_ptr<ITargetProvider> targetProvider;
  std::unique_ptr<IBallisticSolver> ballisticSolver;
  std::unique_ptr<DronePhysics> dronePhysics;

  DroneConfig droneInitialConfig{};
  float bombFlightTime = 0.0f;
  float h = 0.0f;
  float droneAcceleration = 0.0f;  // TODO: Remove it when sim.droneAcceleration used
  int targetsCount = 0;
  std::unique_ptr<IDroneState> currentState;

  std::vector<SimStep> stepsLog;

  void syncSimulationWithDronePhysics();

public:
  MissionProcessor(std::shared_ptr<ITargetProvider> provider, std::unique_ptr<IBallisticSolver> solver);
  [[nodiscard]] bool init(std::unique_ptr<IConfigLoader> configLoader);
  [[nodiscard]] bool hasNext() const;
  SimStep step();
  void changeSolver(std::unique_ptr<IBallisticSolver> solver);
  void reset();
  std::vector<SimStep> getStepsLog();
  virtual ~MissionProcessor();
};