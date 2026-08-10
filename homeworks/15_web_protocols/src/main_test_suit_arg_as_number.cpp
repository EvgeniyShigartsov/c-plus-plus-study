#include <chrono>
#include <cstddef>
#include <iostream>
#include <map>
#include <memory>
#include <string>
#include <thread>
#include <vector>
#include "DronePhysics.hpp"
#include "HttpClient.hpp"
#include "interfaces/IBallisticSolver.hpp"
#include "interfaces/ITargetProvider.hpp"
#include "interfaces/IConfigLoader.hpp"
#include "third_party/json.hpp"
#include "types.hpp"
#include "Logger.hpp"
#include "config/ComponentFactory.hpp"
#include "MissionProcessor.hpp"

using json = nlohmann::json;

const std::string CONFIG_FILE_PATH = "homeworks/15_web_protocols/data/config.json";
const std::string AMMO_FILE_PATH = "homeworks/15_web_protocols/data/ammo.json";
const std::string TARGETS_FILE_PATH = "homeworks/15_web_protocols/data/targets.json";
const std::string BALLISTIC_TABLE_FILE_PATH = "homeworks/15_web_protocols/data/ballistic_table.txt";

const std::string TESTING_DATA_DIR = "homeworks/15_web_protocols/data/testing_data/";
const std::string STUDENT_ID = "2120";
const std::string DEFAULT_TEST_ID = "T01";
const size_t MAX_RETRIES = 5;

struct TestPaths {
  std::string configPath;
  std::string targetsPath;
  std::string testId;
};

TestPaths getTestPaths(const std::string& key)
{
  static const std::map<std::string, TestPaths> testPaths = {
    {"1", {TESTING_DATA_DIR + "01_sample_circles/01_config.json", TESTING_DATA_DIR + "01_sample_circles/01_targets.json", "T01"}},
    {"2",
     {TESTING_DATA_DIR + "02_eliptic_trajectories/02_config.json", TESTING_DATA_DIR + "02_eliptic_trajectories/02_targets.json", "T02"}},
    {"3", {TESTING_DATA_DIR + "03_visimky_lissaju_1-2/03_config.json", TESTING_DATA_DIR + "03_visimky_lissaju_1-2/03_targets.json", "T03"}},
    {"4", {TESTING_DATA_DIR + "04_star_trajectories/04_config.json", TESTING_DATA_DIR + "04_star_trajectories/04_targets.json", "T04"}},
    {"5",
     {TESTING_DATA_DIR + "05_lissaju_complex_curves/05_config.json",
      TESTING_DATA_DIR + "05_lissaju_complex_curves/05_targets.json",
      "T05"}},
    {"6",
     {TESTING_DATA_DIR + "06_fast_drone_slow_targets/06_config.json",
      TESTING_DATA_DIR + "06_fast_drone_slow_targets/06_targets.json",
      "T06"}},
    {"7", {TESTING_DATA_DIR + "07_heavy_ammo/07_config.json", TESTING_DATA_DIR + "07_heavy_ammo/07_targets.json", "T07"}},
    {"8", {TESTING_DATA_DIR + "08_gliding_ammo/08_config.json", TESTING_DATA_DIR + "08_gliding_ammo/08_targets.json", "T08"}},
    {"9",
     {TESTING_DATA_DIR + "09_cardioids_eptirohoids/09_config.json", TESTING_DATA_DIR + "09_cardioids_eptirohoids/09_targets.json", "T09"}},
    {"10", {TESTING_DATA_DIR + "10_extreme_far_fast/10_config.json", TESTING_DATA_DIR + "10_extreme_far_fast/10_targets.json", "T10"}},
  };

  const auto it = testPaths.find(key);

  if (it == testPaths.end()) {
    return {CONFIG_FILE_PATH, TARGETS_FILE_PATH, DEFAULT_TEST_ID};
  }

  return it->second;
}

json toJsonXY(const Coord& coord)
{
  return {{"x", coord.x}, {"y", coord.y}};
}

json buildSimulationJson(const std::vector<SimStep>& stepsLog)
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

  return out;
}

void sendResultsToServer(const std::string& testId, const json& simulationJson)
{
  HttpClient httpClient(STUDENT_ID);

  for (size_t attempt = 0; attempt < MAX_RETRIES; attempt++) {
    const httplib::Result res = httpClient.sendResults(testId, simulationJson);

    if (!res) {
      LOG(testId << ": No HTTP connection, retryable failure");
    }
    else if (res->status == 503) {
      LOG(testId << ": Save results failed, retryable failure (503)");
    }
    else if (res->status == 200 || res->status == 201) {
      const bool confirmed = httpClient.ensureResults(testId);

      if (confirmed) {
        LOG(testId << ": Results saved on server");
        return;
      }

      LOG(testId << ": POST request status is " << res->status << ", but no confirmation received");
    }
    else {
      LOG(testId << ": Save results permanent failure: status is " << res->status);
      return;
    }

    std::this_thread::sleep_for(std::chrono::duration(std::chrono::seconds(1)));
  }
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

  const json simulationJson = buildSimulationJson(stepsLog);

  LOG("Simulation complete. Steps: " << stepsLog.size());

  sendResultsToServer(testPaths.testId, simulationJson);

  return 0;
}
