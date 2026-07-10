#include <iostream>
#include <fstream>
#include <map>
#include <memory>
#include <string>
#include <thread>
#include <vector>
#include "DronePhysics.hpp"
#include "interfaces/IBallisticSolver.hpp"
#include "interfaces/ITargetProvider.hpp"
#include "interfaces/IConfigLoader.hpp"
#include "third_party/json.hpp"
#include "types.hpp"
#include "Logger.hpp"
#include "config/ComponentFactory.hpp"
#include "MissionProcessor.hpp"

using json = nlohmann::json;

const std::string CONFIG_FILE_PATH = "homeworks/11_drone_manage_with_UART_and_auto_drop/data/config.json";
const std::string AMMO_FILE_PATH = "homeworks/11_drone_manage_with_UART_and_auto_drop/data/ammo.json";
const std::string TARGETS_FILE_PATH = "homeworks/11_drone_manage_with_UART_and_auto_drop/data/targets.json";
const std::string BALLISTIC_TABLE_FILE_PATH = "homeworks/11_drone_manage_with_UART_and_auto_drop/data/ballistic_table.txt";

const std::string TESTING_DATA_DIR = "homeworks/11_drone_manage_with_UART_and_auto_drop/data/testing_data/";

struct TestPaths {
  std::string configPath;
  std::string targetsPath;
};

TestPaths getTestPaths(const std::string& key)
{
  static const std::map<std::string, TestPaths> testPaths = {
    {"1", {TESTING_DATA_DIR + "01_sample_circles/01_config.json", TESTING_DATA_DIR + "01_sample_circles/01_targets.json"}},
    {"2", {TESTING_DATA_DIR + "02_eliptic_trajectories/02_config.json", TESTING_DATA_DIR + "02_eliptic_trajectories/02_targets.json"}},
    {"3", {TESTING_DATA_DIR + "03_visimky_lissaju_1-2/03_config.json", TESTING_DATA_DIR + "03_visimky_lissaju_1-2/03_targets.json"}},
    {"4", {TESTING_DATA_DIR + "04_star_trajectories/04_config.json", TESTING_DATA_DIR + "04_star_trajectories/04_targets.json"}},
    {"5", {TESTING_DATA_DIR + "05_lissaju_complex_curves/05_config.json", TESTING_DATA_DIR + "05_lissaju_complex_curves/05_targets.json"}},
    {"6",
     {TESTING_DATA_DIR + "06_fast_drone_slow_targets/06_config.json", TESTING_DATA_DIR + "06_fast_drone_slow_targets/06_targets.json"}},
    {"7", {TESTING_DATA_DIR + "07_heavy_ammo/07_config.json", TESTING_DATA_DIR + "07_heavy_ammo/07_targets.json"}},
    {"8", {TESTING_DATA_DIR + "08_gliding_ammo/08_config.json", TESTING_DATA_DIR + "08_gliding_ammo/08_targets.json"}},
    {"9", {TESTING_DATA_DIR + "09_cardioids_eptirohoids/09_config.json", TESTING_DATA_DIR + "09_cardioids_eptirohoids/09_targets.json"}},
    {"10", {TESTING_DATA_DIR + "10_extreme_far_fast/10_config.json", TESTING_DATA_DIR + "10_extreme_far_fast/10_targets.json"}},
  };

  const auto it = testPaths.find(key);

  if (it == testPaths.end()) {
    return {CONFIG_FILE_PATH, TARGETS_FILE_PATH};
  }

  return it->second;
}

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

int main(int argc, char* argv[])
{
  // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-pointer-arithmetic)
  const std::string testKey = (argc > 1) ? argv[1] : "";
  const TestPaths testPaths = getTestPaths(testKey);

  std::unique_ptr<IConfigLoader> configLoader = createLoader(LoaderType::FILE);

  if (configLoader == nullptr) {
    LOG("Config loader was not found.");
    return 1;
  }

  const bool isConfigLoadSuccess = configLoader->load(testPaths.configPath, AMMO_FILE_PATH);

  const DroneConfig droneConfig = configLoader->getConfig();
  const BombParams bombParams = configLoader->getAmmoParams();

  std::shared_ptr<ITargetProvider> targetProvider = createProvider(ProviderType::JSON, testPaths.targetsPath, droneConfig);

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
