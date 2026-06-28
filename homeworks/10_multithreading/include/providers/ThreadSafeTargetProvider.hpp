#pragma once
#include <string>
#include <atomic>
#include <vector>
#include "types.hpp"
#include "interfaces/ITargetProvider.hpp"

// NOLINTNEXTLINE(cppcoreguidelines-special-member-functions)
class ThreadSafeTargetProvider : public ITargetProvider {
public:
  ThreadSafeTargetProvider(const std::string& pathToTargets, const DroneConfig& config);

  int getTargetCount() override;
  Target getTarget(const int targetIndex) override;
  bool isLoadSucces() override;

  bool isThreadReady() const override;
  void start() override;
  void stop() override;
  void run() override;

  virtual ~ThreadSafeTargetProvider();

private:
  bool isSuccessfullyLoaded = false;

  std::vector<std::vector<Coord>> targetsInTime;
  std::vector<Target> currentTargets;

  int TARGETS_COUNT = 0;
  int TARGET_MOVES_COUNT = 0;
  int currentNodeIndex = 0;

  float arrayTimeStep = 0.0f;
  float timeScale = 1;

  std::atomic<bool> startFlag{false};
  std::atomic<bool> stopFlag{false};
  std::atomic<bool> threadReady{false};

  std::mutex targetsMutex;

  void advance();
};
