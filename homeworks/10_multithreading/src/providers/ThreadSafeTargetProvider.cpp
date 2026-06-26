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

    isSuccessfullyLoaded = true;

    for (int i = 0; i < TARGETS_COUNT; i++) {
      const int nextIndex = 1;
      const Coord pos = targetsInTime[i][0];
      const Coord nextPos = targetsInTime[i][nextIndex];
      const Coord velocity = (nextPos - pos) / arrayTimeStep;
      currentTargets[i] = {.pos = pos, .velocity = velocity};
    }
  }
  catch (const json::exception& parseError) {
    LOG("targets.json parse error: " << parseError.what());
  }
}

Target ThreadSafeTargetProvider::getTarget(const int targetIndex)
{
  std::lock_guard<std::mutex> lock(targetsMutex);
  return currentTargets[targetIndex];
}

void ThreadSafeTargetProvider::advance()
{
  currentNodeIndex = (currentNodeIndex + 1) % TARGET_MOVES_COUNT;

  std::lock_guard<std::mutex> lock(targetsMutex);
  for (int i = 0; i < TARGETS_COUNT; i++) {
    const int nextIndex = (currentNodeIndex + 1) % TARGET_MOVES_COUNT;

    const Coord pos = targetsInTime[i][currentNodeIndex];
    const Coord nextPos = targetsInTime[i][nextIndex];
    const Coord velocity = (nextPos - pos) / arrayTimeStep;

    currentTargets[i] = {.pos = pos, .velocity = velocity};
  }
}

bool ThreadSafeTargetProvider::isLoadSucces()
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
  thread.join();
}
void ThreadSafeTargetProvider::run()
{
  threadReady = true;

  while (!startFlag) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  while (!stopFlag) {
    advance();
    std::this_thread::sleep_for(std::chrono::duration<float>(arrayTimeStep / timeScale));
  }
}

void ThreadSafeTargetProvider::setThread(std::thread t)
{
  thread = std::move(t);
}

int ThreadSafeTargetProvider::getTargetCount()
{
  return TARGETS_COUNT;
}

ThreadSafeTargetProvider::~ThreadSafeTargetProvider() = default;