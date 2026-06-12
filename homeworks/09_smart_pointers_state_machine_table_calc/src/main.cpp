#include <iostream>
#include <fstream>
#include <memory>
#include <string>
#include <vector>
#include "interfaces/IBallisticSolver.hpp"
#include "third_party/json.hpp"
#include "types.hpp"
#include "Logger.hpp"
#include "config/ComponentFactory.hpp"
#include "MissionProcessor.hpp"

using json = nlohmann::json;

const std::string CONFIG_FILE_PATH = "homeworks/09_smart_pointers_state_machine_table_calc/data/config.json";
const std::string AMMO_FILE_PATH = "homeworks/09_smart_pointers_state_machine_table_calc/data/ammo.json";
const std::string TARGETS_FILE_PATH = "homeworks/09_smart_pointers_state_machine_table_calc/data/targets.json";

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

  std::shared_ptr<ITargetProvider> targetProvider = createProvider(ProviderType::JSON, TARGETS_FILE_PATH, configLoader->getConfig());

  if (targetProvider == nullptr) {
    LOG("Target provider was not found.");
    return 1;
  }

  std::unique_ptr<IBallisticSolver> solver = createSolver(SolverType::ANALYTICAL);

  if (solver == nullptr) {
    LOG("Ballistic solver was not found.");
    return 1;
  }

  MissionProcessor missionProcessor{targetProvider, std::move(solver)};

  const bool isInitSucces = missionProcessor.init(std::move(configLoader));
  if (!isConfigLoadSuccess || !targetProvider->isLoadSucces() || !isInitSucces) {
    return 1;
  }

  while (missionProcessor.hasNext()) {
    missionProcessor.step();
  }

  const std::vector<SimStep> stepsLog = missionProcessor.getStepsLog();

  writeSimulationJson(stepsLog);

  LOG("Simulation complete. Steps: " << stepsLog.size());

  missionProcessor.reset();

  return 0;
}
