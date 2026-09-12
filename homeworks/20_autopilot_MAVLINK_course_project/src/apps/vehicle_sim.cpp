#include <chrono>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

#include "Logger.hpp"
#include "link/UdpLink.hpp"
#include "mavlink/Codec.hpp"
#include "mavlink/Endpoint.hpp"
#include "mavlink/RadioControl.hpp"
#include "sim/DronePhysics.hpp"
#include "sim/FileConfigLoader.hpp"
#include "sim/JsonTargetProvider.hpp"

// NOLINTBEGIN(cppcoreguidelines-pro-bounds-pointer-arithmetic)

struct CliOptions {
  std::string scenario = "data";
  std::string apHost = "127.0.0.1";  // куди шлемо телеметрію — хост autopilot
  uint16_t apPort = 14560;           // куди шлемо телеметрію — домашній порт autopilot
  uint16_t ownPort = 14555;          // домашній порт, сюди autopilot шле RC_CHANNELS_OVERRIDE
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
    else if (key == "--ap-host") {
      opts.apHost = value;
    }
    else if (key == "--ap-port") {
      opts.apPort = static_cast<uint16_t>(std::stoi(value));
    }
    else if (key == "--own-port") {
      opts.ownPort = static_cast<uint16_t>(std::stoi(value));
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

VehicleState toVehicleState(const DroneTelemetry& telemetry, const float altitude)
{
  return {
    .mission_time_ms = static_cast<uint32_t>(telemetry.timeSinceStart * 1000.0f),
    .x = telemetry.pos.x,
    .y = telemetry.pos.y,
    .z = altitude,
    .vx = telemetry.speed * std::cos(telemetry.dir),
    .vy = telemetry.speed * std::sin(telemetry.dir),
    .speed = telemetry.speed,
    .dir = telemetry.dir,
  };
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
      << "  scenario   = " << opts.scenario << '\n'
      << "  ap-host    = " << opts.apHost << '\n'
      << "  ap-port    = " << opts.apPort << '\n'
      << "  own-port   = " << opts.ownPort << '\n'
      << "  time-scale = " << opts.timeScale);

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

  const UdpLink udp(opts.apHost, opts.apPort, opts.ownPort);
  if (!udp.isOpen()) {
    LOG("Failed to open UDP link to " << opts.apHost << ":" << opts.apPort << " on own port " << opts.ownPort);
    return 1;
  }
  const mav::MavlinkEndpoint endpoint(udp, MAVLINK_COMM_1);

  constexpr std::chrono::seconds heartbeatPeriod = std::chrono::seconds(1);
  std::chrono::steady_clock::time_point lastHeartbeat;

  std::chrono::time_point last = std::chrono::steady_clock::now();
  float accumulator = 0.0f;
  float nextTelemetryLog = 0.0f;

  while (true) {
    for (const mavlink_message_t& msg : endpoint.poll()) {
      if (msg.msgid == MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE) {
        physics.setControl(mav::to_control_signal(mav::parse_radio_control_channels_override(msg)));
      }
    }

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

    if (now - lastHeartbeat >= heartbeatPeriod) {
      endpoint.send(mav::pack_heartbeat(mav::kVehicle, {.type = MAV_TYPE_QUADROTOR, .system_status = MAV_STATE_ACTIVE}));
      lastHeartbeat = now;
    }

    const DroneTelemetry telemetry = physics.getTelemetry();
    if (telemetry.timeSinceStart >= nextTelemetryLog) {
      const Target firstTarget = targets.getTarget(telemetry.timeSinceStart, 0);
      LOG("t=" << telemetry.timeSinceStart << " pos=(" << telemetry.pos.x << "," << telemetry.pos.y << ") speed=" << telemetry.speed
               << " dir=" << telemetry.dir << " | target0=(" << firstTarget.pos.x << "," << firstTarget.pos.y << ")");

      const VehicleState state = toVehicleState(telemetry, droneConfig.altitude);
      endpoint.send(mav::pack_local_position_ned(mav::kVehicle, state));
      endpoint.send(mav::pack_attitude(mav::kVehicle, state));
      endpoint.send(mav::pack_global_position_int(mav::kVehicle, state));

      // TODO: коли буде симуляція оператора, використати, а поки нейтраль
      endpoint.send(mav::pack_radio_control_channels(
        mav::kVehicle, {.time_boot_ms = state.mission_time_ms, .roll = mav::kPwmNeutral, .throttle = mav::kPwmNeutral}));

      nextTelemetryLog += droneConfig.simTimeStep;
    }

    std::this_thread::sleep_for(std::chrono::duration<float>(physicsTimeStep / timeScale));
  }
}

// NOLINTEND(cppcoreguidelines-pro-bounds-pointer-arithmetic)
