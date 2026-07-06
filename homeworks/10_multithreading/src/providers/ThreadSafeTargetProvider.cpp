#include <fstream>
#include <thread>
#include "third_party/json.hpp"
#include "types.hpp"
#include "Logger.hpp"
#include "providers/ThreadSafeTargetProvider.hpp"

using json = nlohmann::json;

ThreadSafeTargetProvider::ThreadSafeTargetProvider(const std::string& pathToTargets, const DroneConfig& droneConfig)
  : arrayTimeStep(droneConfig.arrayTimeStep)
  , timeScale(droneConfig.timeScale)
{
  std::ifstream targetsFile(pathToTargets);

  if (!targetsFile.is_open()) {
    LOG("targets.json was not found.");
    return;
  }

  json targetsData;
  targetsFile >> targetsData;

  TARGETS_COUNT = targetsData["targetCount"];
  TARGET_MOVES_COUNT = targetsData["timeSteps"];

  targetsInTime.reserve(TARGETS_COUNT);

  try {
    for (int target = 0; target < TARGETS_COUNT; target++) {
      std::vector<Coord> targetInTime;
      targetInTime.reserve(TARGET_MOVES_COUNT);

      for (int move = 0; move < TARGET_MOVES_COUNT; move++) {
        targetInTime.push_back({
          .x = targetsData["targets"][target]["positions"][move]["x"],
          .y = targetsData["targets"][target]["positions"][move]["y"],
        });
      }

      targetsInTime.push_back(std::move(targetInTime));
    }

    currentTargets.resize(TARGETS_COUNT);

    updateCurrentTargets(0);
    isSuccessfullyLoaded = true;
  }
  catch (const json::exception& parseError) {
    LOG("targets.json parse error: " << parseError.what());
  }
}

void ThreadSafeTargetProvider::updateCurrentTargets(const int nodeIndex)
{
  std::lock_guard<std::mutex> lock(targetsMutex);
  for (int i = 0; i < TARGETS_COUNT; i++) {
    const int nextIndex = (nodeIndex + 1) % TARGET_MOVES_COUNT;

    const Coord pos = targetsInTime[i][nodeIndex];
    const Coord nextPos = targetsInTime[i][nextIndex];
    const Coord velocity = (nextPos - pos) / arrayTimeStep;

    currentTargets[i] = {.pos = pos, .velocity = velocity};
  }
}

Target ThreadSafeTargetProvider::getTarget(const int targetIndex) const
{
  std::lock_guard<std::mutex> lock(targetsMutex);
  return currentTargets[targetIndex];
}

void ThreadSafeTargetProvider::advance()
{
  currentNodeIndex = (currentNodeIndex + 1) % TARGET_MOVES_COUNT;
  updateCurrentTargets(currentNodeIndex);
}

bool ThreadSafeTargetProvider::isLoadSucces() const
{
  return isSuccessfullyLoaded;
}
bool ThreadSafeTargetProvider::isThreadReady() const
{
  return threadReady;
}

void ThreadSafeTargetProvider::start()
{
  startFlag = true;
}
void ThreadSafeTargetProvider::stop()
{
  stopFlag = true;
}
void ThreadSafeTargetProvider::run()
{
  threadReady = true;

  LOG("TargetProvider thread ready");

  while (!startFlag) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  LOG("TargetProvider started");

  while (!stopFlag) {
    std::this_thread::sleep_for(std::chrono::duration<float>(arrayTimeStep / timeScale));
    advance();
  }

  LOG("TargetProvider stopped");
}

int ThreadSafeTargetProvider::getTargetCount() const
{
  std::lock_guard<std::mutex> lock(targetsMutex);
  return TARGETS_COUNT;
}

ThreadSafeTargetProvider::~ThreadSafeTargetProvider() = default;