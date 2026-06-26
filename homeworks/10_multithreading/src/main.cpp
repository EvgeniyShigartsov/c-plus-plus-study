#include <iostream>
#include <fstream>
#include <memory>
#include <string>
#include <thread>
#include <vector>
#include "DronePhysics.hpp"
#include "interfaces/IBallisticSolver.hpp"
#include "interfaces/ITargetProvider.hpp"
#include "third_party/json.hpp"
#include "types.hpp"
#include "Logger.hpp"
#include "config/ComponentFactory.hpp"
#include "MissionProcessor.hpp"

using json = nlohmann::json;

const std::string CONFIG_FILE_PATH = "homeworks/10_multithreading/data/config.json";
const std::string AMMO_FILE_PATH = "homeworks/10_multithreading/data/ammo.json";
const std::string TARGETS_FILE_PATH = "homeworks/10_multithreading/data/targets.json";
const std::string BALLISTIC_TABLE_FILE_PATH = "homeworks/10_multithreading/data/ballistic_table.txt";

json toJsonXY(const Coord& coord)
{
  return {{"x", coord.x}, {"y", coord.y}};
}

void writeSimulationJson(const std::vector<SimStep>& stepsLog)
{
  json out;

  out["totalSteps"] = stepsLog.size();
  out["steps"] = json::array();

  for (const SimStep& step : stepsLog) {
    json outStep;

    outStep["position"] = toJsonXY(step.pos);
    outStep["direction"] = step.direction;
    outStep["state"] = step.state;
    outStep["targetIndex"] = step.targetIdx;
    outStep["dropPoint"] = toJsonXY(step.dropPoint);
    outStep["aimPoint"] = toJsonXY(step.aimPoint);
    outStep["predictedTarget"] = toJsonXY(step.predictedTarget);
    outStep["timeSecSinceStart"] = step.timeSecSinceStart;

    out["steps"].push_back(outStep);
  }

  std::ofstream outJsonFile("simulation.json");
  outJsonFile << out.dump(2);
}

int main()
{
  std::unique_ptr<IConfigLoader> configLoader = createLoader(LoaderType::FILE);

  if (configLoader == nullptr) {
    LOG("Config loader was not found.");
    return 1;
  }

  const bool isConfigLoadSuccess = configLoader->load(CONFIG_FILE_PATH, AMMO_FILE_PATH);

  const DroneConfig droneConfig = configLoader->getConfig();
  const BombParams bombParams = configLoader->getAmmoParams();

  std::shared_ptr<ITargetProvider> targetProvider = createProvider(ProviderType::JSON, TARGETS_FILE_PATH, droneConfig);

  if (targetProvider == nullptr) {
    LOG("Target provider was not found.");
    return 1;
  }

  std::unique_ptr<IBallisticSolver> solver = createSolver(SolverType::TABLE, BALLISTIC_TABLE_FILE_PATH, bombParams, droneConfig);

  if (solver == nullptr) {
    LOG("Ballistic solver was not found.");
    return 1;
  }

  auto physics = std::make_shared<DronePhysics>(droneConfig);
  auto missionProcessor = std::make_shared<MissionProcessor>(targetProvider, physics, std::move(solver));

  const bool isInitSucces = missionProcessor->init(std::move(configLoader));

  if (!isConfigLoadSuccess || !targetProvider->isLoadSucces() || !isInitSucces) {
    return 1;
  }

  std::thread providerThread(&ITargetProvider::run, targetProvider);
  std::thread physicsThread(&DronePhysics::run, physics);
  std::thread missionThread(&MissionProcessor::run, missionProcessor);

  while (!targetProvider->isThreadReady() || !physics->isThreadReady() || !missionProcessor->isThreadReady()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  targetProvider->start();
  physics->start();
  missionProcessor->start();

  missionThread.join();

  physics->stop();
  targetProvider->stop();

  providerThread.join();
  physicsThread.join();

  const std::vector<SimStep> stepsLog = missionProcessor->getStepsLog();

  writeSimulationJson(stepsLog);

  LOG("Simulation complete. Steps: " << stepsLog.size());

  return 0;
}
