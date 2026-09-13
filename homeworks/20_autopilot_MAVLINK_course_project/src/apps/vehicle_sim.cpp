#include <chrono>
#include <cmath>
#include <cstdint>
#include <fstream>
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
#include "third_party/json.hpp"

using json = nlohmann::json;

// NOLINTBEGIN(cppcoreguidelines-pro-bounds-pointer-arithmetic)

struct CliOptions {
  std::string scenario = "data";
  std::string apHost = "127.0.0.1";   // куди шлемо телеметрію — хост autopilot
  uint16_t apPort = 14560;            // куди шлемо телеметрію — домашній порт autopilot
  uint16_t ownPort = 14555;           // домашній порт, сюди autopilot шле RC_CHANNELS_OVERRIDE
  std::string gcsHost = "127.0.0.1";  // куди дублюємо телеметрію -- для QGC/чекера
  uint16_t gcsPort = 14550;
  float timeScale = 1.0f;
  std::string simOutput = "simulation.json";
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
    else if (key == "--gcs-host") {
      opts.gcsHost = value;
    }
    else if (key == "--gcs-port") {
      opts.gcsPort = static_cast<uint16_t>(std::stoi(value));
    }
    else if (key == "--time-scale") {
      opts.timeScale = std::stof(value);
    }
    else if (key == "--sim-output") {
      opts.simOutput = value;
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

json toJsonXY(const Coord& coord)
{
  return {{"x", coord.x}, {"y", coord.y}};
}

void writeSimulationJson(const std::vector<SimStep>& stepsLog, const std::string& path)
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

  std::ofstream outJsonFile(path);
  outJsonFile << out.dump(2);
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
      << "  gcs-host   = " << opts.gcsHost << '\n'
      << "  gcs-port   = " << opts.gcsPort << '\n'
      << "  time-scale = " << opts.timeScale << '\n'
      << "  sim-output = " << opts.simOutput);

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

  // Дублювання телеметрії на GCS
  const UdpLink gcsUdp(opts.gcsHost, opts.gcsPort);
  const mav::MavlinkEndpoint gcsEndpoint(gcsUdp, MAVLINK_COMM_2);

  constexpr std::chrono::seconds heartbeatPeriod = std::chrono::seconds(1);
  std::chrono::steady_clock::time_point lastHeartbeat;

  std::chrono::time_point last = std::chrono::steady_clock::now();
  float accumulator = 0.0f;
  float nextTelemetryLog = 0.0f;

  std::vector<SimStep> stepsLog;
  bool dropped = false;

  mav::RadioControlOverride lastAutopilotOverride{.roll = mav::kPwmNeutral, .throttle = mav::kPwmNeutral};
  mav::RadioControlOverride lastOperatorOverride{.roll = mav::kPwmNeutral, .throttle = mav::kPwmNeutral};

  while (true) {
    for (const mavlink_message_t& msg : endpoint.poll()) {
      if (msg.msgid == MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE) {
        const mav::RadioControlOverride rc_override = mav::parse_radio_control_channels_override(msg);
        const mav::Identity from{.sysid = msg.sysid, .compid = msg.compid};

        if (from == mav::kGcs) {
          lastOperatorOverride = rc_override;
        }
        else {
          lastAutopilotOverride = rc_override;
        }
      }
      else if (msg.msgid == MAVLINK_MSG_ID_COMMAND_LONG && mavlink_msg_command_long_get_command(&msg) == mav::kDropNotificationCommandId &&
               !dropped) {
        dropped = true;

        const mav::DropNotification drop = mav::parse_drop_notification(msg);
        const Coord dropPoint{.x = static_cast<float>(drop.latitude), .y = static_cast<float>(drop.longitude)};

        const DroneTelemetry telemetryAtDrop = physics.getTelemetry();
        const Target targetAtDrop = targets.getTarget(telemetryAtDrop.timeSinceStart, 0);

        const float missDistance = std::hypot(dropPoint.x - targetAtDrop.pos.x, dropPoint.y - targetAtDrop.pos.y);
        const bool hit = missDistance <= droneConfig.hitRadius;

        LOG("DROP t=" << telemetryAtDrop.timeSinceStart << " point=(" << dropPoint.x << "," << dropPoint.y << ") target=("
                      << targetAtDrop.pos.x << "," << targetAtDrop.pos.y << ") miss=" << missDistance << " -> " << (hit ? "HIT" : "MISS"));

        stepsLog.push_back({
          .pos = telemetryAtDrop.pos,
          .dropPoint = dropPoint,
          .state = "Stopped",  // тимчасова заглушка
          .targetIdx = 0,      // тимчасова заглушка
          .step = static_cast<int>(stepsLog.size()),
          .timeSecSinceStart = telemetryAtDrop.timeSinceStart,
        });
        writeSimulationJson(stepsLog, opts.simOutput);
      }
    }

    const bool operatorActive =
      !mav::are_channels_in_deadband({.roll = lastOperatorOverride.roll, .throttle = lastOperatorOverride.throttle});

    physics.setControl(mav::to_control_signal(operatorActive ? lastOperatorOverride : lastAutopilotOverride));

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
      const mavlink_message_t heartbeat =
        mav::pack_heartbeat(mav::kVehicle, {.type = MAV_TYPE_QUADROTOR, .system_status = MAV_STATE_ACTIVE});
      endpoint.send(heartbeat);
      gcsEndpoint.send(heartbeat);
      lastHeartbeat = now;
    }

    const DroneTelemetry telemetry = physics.getTelemetry();
    if (telemetry.timeSinceStart >= nextTelemetryLog) {
      const Target firstTarget = targets.getTarget(telemetry.timeSinceStart, 0);
      LOG("t=" << telemetry.timeSinceStart << " pos=(" << telemetry.pos.x << "," << telemetry.pos.y << ") speed=" << telemetry.speed
               << " dir=" << telemetry.dir << " | target0=(" << firstTarget.pos.x << "," << firstTarget.pos.y << ")");

      if (!dropped) {
        stepsLog.push_back({
          .pos = telemetry.pos,
          .direction = telemetry.dir,
          .state = "Stopped",  // тимчасова зашлушка
          .targetIdx = 0,      // тимчасова зашлушка
          .step = static_cast<int>(stepsLog.size()),
          .timeSecSinceStart = telemetry.timeSinceStart,
        });
      }

      const VehicleState state = toVehicleState(telemetry, droneConfig.altitude);
      const mavlink_message_t localPositionNed = mav::pack_local_position_ned(mav::kVehicle, state);
      const mavlink_message_t attitude = mav::pack_attitude(mav::kVehicle, state);
      const mavlink_message_t globalPositionInt = mav::pack_global_position_int(mav::kVehicle, state);

      const mavlink_message_t radioControlChannels = mav::pack_radio_control_channels(
        mav::kVehicle,
        {.time_boot_ms = state.mission_time_ms, .roll = lastOperatorOverride.roll, .throttle = lastOperatorOverride.throttle});

      endpoint.send(localPositionNed);
      endpoint.send(attitude);
      endpoint.send(globalPositionInt);
      endpoint.send(radioControlChannels);

      gcsEndpoint.send(localPositionNed);
      gcsEndpoint.send(attitude);
      gcsEndpoint.send(globalPositionInt);
      gcsEndpoint.send(radioControlChannels);

      nextTelemetryLog += droneConfig.simTimeStep;
    }

    std::this_thread::sleep_for(std::chrono::duration<float>(physicsTimeStep / timeScale));
  }
}

// NOLINTEND(cppcoreguidelines-pro-bounds-pointer-arithmetic)
