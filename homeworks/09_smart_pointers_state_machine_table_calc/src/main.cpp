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

void writeSimulation(const std::vector<float>& droneXHistory,
                     const std::vector<float>& droneYHistory,
                     const std::vector<float>& droneDirHistory,
                     const std::vector<int>& droneStateHistory,
                     const std::vector<int>& droneSelectedTargetHistory,
                     const size_t steps)
{
  std::ofstream simulation("simulation.txt");
  simulation << steps << std::endl;

  for (size_t i = 0; i < steps; i++) {
    simulation << droneXHistory[i] << ' ' << droneYHistory[i] << ' ';
  }
  simulation << std::endl;

  for (size_t i = 0; i < steps; i++) {
    simulation << droneDirHistory[i] << ' ';
  }
  simulation << std::endl;

  for (size_t i = 0; i < steps; i++) {
    simulation << droneStateHistory[i] << ' ';
  }
  simulation << std::endl;

  for (size_t i = 0; i < steps; i++) {
    simulation << droneSelectedTargetHistory[i] << ' ';
  }
  simulation << std::endl;

  simulation.close();
}

int main()
{
  std::unique_ptr<IConfigLoader> configLoader = createLoader(LoaderType::FILE);
  const bool isConfigLoadSuccess = configLoader->load(CONFIG_FILE_PATH, AMMO_FILE_PATH);

  std::shared_ptr<ITargetProvider> targetProvider = createProvider(ProviderType::JSON, TARGETS_FILE_PATH, configLoader->getConfig());
  std::unique_ptr<IBallisticSolver> solver = createSolver(SolverType::ANALYTICAL);

  MissionProcessor missionProcessor{targetProvider, std::move(solver)};

  const bool isInitSucces = missionProcessor.init(std::move(configLoader));
  if (!isConfigLoadSuccess || !targetProvider->isLoadSucces() || !isInitSucces) {
    return 1;
  }

  // Історія для зворотньої сумісності з .txt результатом симуляції.
  std::vector<float> droneXHistory;
  std::vector<float> droneYHistory;
  std::vector<float> droneDirHistory;
  std::vector<int> droneSelectedTargetHistory;
  std::vector<int> droneStateHistory;
  const std::map<std::string, int> stateNameToOldDroneStateInt = {
    {"Stopped", 0},
    {"Accelerating", 1},
    {"Decelerating", 2},
    {"Turning", 3},
    {"Moving", 4},
  };

  while (missionProcessor.hasNext()) {
    const SimStep stepResult = missionProcessor.step();

    droneXHistory.push_back(stepResult.pos.x);
    droneYHistory.push_back(stepResult.pos.y);
    droneDirHistory.push_back(stepResult.direction);
    droneSelectedTargetHistory.push_back(stepResult.targetIdx);
    const auto it = stateNameToOldDroneStateInt.find(stepResult.state);

    if (it != stateNameToOldDroneStateInt.end()) {
      droneStateHistory.push_back(it->second);
    }
    else {
      LOG("Unknown state name: " << stepResult.state);
    }
  }

  const std::vector<SimStep> stepsLog = missionProcessor.getStepsLog();

  writeSimulation(droneXHistory, droneYHistory, droneDirHistory, droneStateHistory, droneSelectedTargetHistory, stepsLog.size());

  writeSimulationJson(stepsLog);

  LOG("Simulation complete. Steps: " << stepsLog.size());

  missionProcessor.reset();

  return 0;
}
