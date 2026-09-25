#include "sim/JsonTargetProvider.hpp"

#include <cmath>
#include <fstream>

#include "Logger.hpp"
#include "third_party/json.hpp"

using json = nlohmann::json;

namespace {

// Портовано з MathUtils ДЗ-09 — тут єдиний споживач, окремий файл не виправданий.
struct InterpolationIndex {
  float frac;
  int idx;
  int next;
};

Coord interpolatePos(const float frac, const Coord& currentTargetPos, const Coord& nextTargetPos)
{
  return currentTargetPos + (nextTargetPos - currentTargetPos) * frac;
}

InterpolationIndex getInterpolationIndex(const float t, const float arrayTimeStep, const int targetMovesCount)
{
  const int idx = (int)(floorf(t / arrayTimeStep)) % targetMovesCount;
  const int next = (idx + 1) % targetMovesCount;
  const float frac = (t - static_cast<float>(idx) * arrayTimeStep) / arrayTimeStep;

  return {frac, idx, next};
}

}  // namespace

JsonTargetProvider::JsonTargetProvider(const std::string& pathToConfig, float arrayTimeStep, float simTimeStep)
  : arrayTimeStep(arrayTimeStep)
  , simTimeStep(simTimeStep)
{
  std::ifstream targetsFile(pathToConfig);

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
    isSuccesFullyLoaded = true;
  }
  catch (const json::exception& parseError) {
    LOG("targets.json parse error: " << parseError.what());
  }
}

Target JsonTargetProvider::getTarget(const float simCurrentTime, const int targetIndex) const
{
  const InterpolationIndex currentIndex = getInterpolationIndex(simCurrentTime, arrayTimeStep, TARGET_MOVES_COUNT);

  const Coord targetCurrentXY =
    interpolatePos(currentIndex.frac, targetsInTime[targetIndex][currentIndex.idx], targetsInTime[targetIndex][currentIndex.next]);

  // 2. Обчислити швидкість цілі (targetVx, targetVy) через кінцеві різниці
  const InterpolationIndex nextIndex = getInterpolationIndex(simCurrentTime + simTimeStep, arrayTimeStep, TARGET_MOVES_COUNT);

  const Coord targetNextXY =
    interpolatePos(nextIndex.frac, targetsInTime[targetIndex][nextIndex.idx], targetsInTime[targetIndex][nextIndex.next]);
  const Coord targetVelocity = (targetNextXY - targetCurrentXY) / simTimeStep;

  return {.pos = targetCurrentXY, .velocity = targetVelocity};
}

int JsonTargetProvider::getTargetCount() const
{
  return TARGETS_COUNT;
}

bool JsonTargetProvider::isLoadSucces() const
{
  return isSuccesFullyLoaded;
}

JsonTargetProvider::~JsonTargetProvider() = default;
