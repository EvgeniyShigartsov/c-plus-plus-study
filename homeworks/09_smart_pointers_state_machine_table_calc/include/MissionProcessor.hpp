#pragma once
#include <memory>
#include <vector>
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

  DroneConfig dc{};
  float bombFlightTime = 0.0f;
  float h = 0.0f;
  float droneAcceleration = 0.0f;
  int targetsCount = 0;

  std::vector<SimStep> stepsLog;

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