#pragma once
#include <atomic>
#include <memory>
#include <vector>
#include "DronePhysics.hpp"
#include "interfaces/IDroneState.hpp"
#include "types.hpp"

class IConfigLoader;
class IBallisticSolver;
class ITargetProvider;

// NOLINTNEXTLINE(cppcoreguidelines-special-member-functions)
class MissionProcessor {
private:
  Simulation sim;
  std::shared_ptr<ITargetProvider> targetProvider;
  std::unique_ptr<IBallisticSolver> ballisticSolver;
  std::shared_ptr<DronePhysics> dronePhysics;

  DroneConfig droneInitialConfig{};
  float bombFlightTime = 0.0f;
  float h = 0.0f;
  int targetsCount = 0;
  std::unique_ptr<IDroneState> currentState;

  std::vector<SimStep> stepsLog;

  std::atomic<bool> startFlag{false};
  std::atomic<bool> stopFlag{false};
  std::atomic<bool> threadReady{false};

public:
  MissionProcessor(std::shared_ptr<ITargetProvider> provider,
                   std::shared_ptr<DronePhysics> dronePhysics,
                   std::unique_ptr<IBallisticSolver> solver);
  [[nodiscard]] bool init(std::unique_ptr<IConfigLoader> configLoader);
  [[nodiscard]] bool hasNext() const;
  SimStep step(const DroneTelemetry& telemetry);
  void changeSolver(std::unique_ptr<IBallisticSolver> solver);
  void reset();
  std::vector<SimStep> getStepsLog();

  void run();
  void start();
  void stop();
  bool isThreadReady() const;

  virtual ~MissionProcessor();
};