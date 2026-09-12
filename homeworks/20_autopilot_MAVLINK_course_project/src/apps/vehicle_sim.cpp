#include <chrono>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

#include "Logger.hpp"
#include "sim/DronePhysics.hpp"
#include "sim/FileConfigLoader.hpp"
#include "sim/JsonTargetProvider.hpp"

// NOLINTBEGIN(cppcoreguidelines-pro-bounds-pointer-arithmetic)

struct CliOptions {
  std::string scenario = "data";
  std::string apEndpoint = "udp://127.0.0.1:14555";
  float timeScale = 1.0f;
};

CliOptions parseArgs(const std::vector<std::string>& args)
{
  CliOptions opts;

  for (size_t i = 0; i + 1 < args.size(); i += 2) {
    const std::string& key = args[i];
    const std::string& value = args[i + 1];

    if (key == "--scenario") {
      opts.scenario = value;
    }
    else if (key == "--ap-endpoint") {
      opts.apEndpoint = value;
    }
    else if (key == "--time-scale") {
      opts.timeScale = std::stof(value);
    }
    else {
      std::cerr << "Unknown argument at vehicle_sim.cpp: " << key << '\n';
    }
  }

  return opts;
}

constexpr std::string CONFIG_FILE_FILENAME = "config.json";
constexpr std::string AMMO_FILE_FILENAME = "ammo.json";
constexpr std::string TARGETS_FILE_FILENAME = "targets.json";

std::string makeScenarioPath(const std::string& scenario, const std::string& filename)
{
  return scenario + "/" + filename;
}

int main(int argc, char* argv[])
{
  std::vector<std::string> args;
  for (int i = 1; i < argc; i++) {
    args.emplace_back(argv[i]);
  }
  const CliOptions opts = parseArgs(args);

  LOG("vehicle_sim:\n"
      << "  scenario    = " << opts.scenario << '\n'
      << "  ap-endpoint = " << opts.apEndpoint << '\n'
      << "  time-scale  = " << opts.timeScale);

  FileConfigLoader loader;
  if (!loader.load(makeScenarioPath(opts.scenario, CONFIG_FILE_FILENAME), makeScenarioPath(opts.scenario, AMMO_FILE_FILENAME))) {
    LOG("Failed to load config or ammo");
    return 1;
  }

  const DroneConfig droneConfig = loader.getConfig();
  const float physicsTimeStep = loader.getPhysicsTimeStep();
  const float timeScale = opts.timeScale;

  DronePhysics physics = DronePhysics(droneConfig);
  const JsonTargetProvider targets =
    JsonTargetProvider(makeScenarioPath(opts.scenario, TARGETS_FILE_FILENAME), loader.getArrayTimeStep(), droneConfig.simTimeStep);

  if (!targets.isLoadSucces()) {
    return 1;
  }

  std::chrono::time_point last = std::chrono::steady_clock::now();
  float accumulator = 0.0f;
  float nextTelemetryLog = 0.0f;

  while (true) {
    const std::chrono::time_point now = std::chrono::steady_clock::now();
    accumulator += std::chrono::duration<float>(now - last).count() * timeScale;
    last = now;

    const float maxCatchUp = 1.0f;
    if (accumulator > maxCatchUp) {
      accumulator = maxCatchUp;
    }

    while (accumulator >= physicsTimeStep) {
      physics.stepPhysics(physicsTimeStep);
      accumulator -= physicsTimeStep;
    }

    const DroneTelemetry telemetry = physics.getTelemetry();
    if (telemetry.timeSinceStart >= nextTelemetryLog) {
      const Target firstTarget = targets.getTarget(telemetry.timeSinceStart, 0);
      LOG("t=" << telemetry.timeSinceStart << " pos=(" << telemetry.pos.x << "," << telemetry.pos.y << ") speed=" << telemetry.speed
               << " dir=" << telemetry.dir << " | target0=(" << firstTarget.pos.x << "," << firstTarget.pos.y << ")");
      nextTelemetryLog += droneConfig.simTimeStep;
    }

    std::this_thread::sleep_for(std::chrono::duration<float>(physicsTimeStep / timeScale));
  }
}

// NOLINTEND(cppcoreguidelines-pro-bounds-pointer-arithmetic)
