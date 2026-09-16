#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
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

#define VEHICLE_LOG(msg) LOG("[VEHICLE]: " << msg)
#define VEHICLE_DEBUG(msg) DEBUG("[VEHICLE]: " << msg)

// NOLINTBEGIN(cppcoreguidelines-pro-bounds-pointer-arithmetic)

const std::string defaultDataDir = "homeworks/20_autopilot_MAVLINK_course_project/data";

struct CliOptions {
  std::string apHost = "127.0.0.1";   // куди шлемо телеметрію — хост autopilot
  uint16_t apPort = 14560;            // куди шлемо телеметрію — домашній порт autopilot
  uint16_t ownPort = 14555;           // домашній порт, сюди autopilot шле RC_CHANNELS_OVERRIDE
  std::string gcsHost = "127.0.0.1";  // куди дублюємо телеметрію -- для QGC/чекера
  uint16_t gcsPort = 14550;
  float timeScale = -1.0f;  // -1 = не задано явно, береться з config.json
  std::string configPath = defaultDataDir + "/config.json";
  std::string ammoPath = defaultDataDir + "/ammo.json";
};

CliOptions parseArgs(const std::vector<std::string>& args)
{
  CliOptions opts;

  for (size_t i = 0; i + 1 < args.size(); i += 2) {
    const std::string& key = args[i];
    const std::string& value = args[i + 1];

    if (key == "--ap-host") {
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
    else if (key == "--config-path") {
      opts.configPath = value;
    }
    else if (key == "--ammo-path") {
      opts.ammoPath = value;
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

int main(int argc, char* argv[])
{
  std::vector<std::string> args;
  for (int i = 1; i < argc; i++) {
    args.emplace_back(argv[i]);
  }
  const CliOptions opts = parseArgs(args);

  VEHICLE_LOG("vehicle_sim:\n"
              << "  ap-host    = " << opts.apHost << '\n'
              << "  ap-port    = " << opts.apPort << '\n'
              << "  own-port   = " << opts.ownPort << '\n'
              << "  gcs-host   = " << opts.gcsHost << '\n'
              << "  gcs-port   = " << opts.gcsPort);

  FileConfigLoader loader;
  if (!loader.load(opts.configPath, opts.ammoPath)) {
    VEHICLE_LOG("Failed to load config or ammo");
    return 1;
  }

  const DroneConfig droneConfig = loader.getConfig();
  const float physicsTimeStep = loader.getPhysicsTimeStep();
  const bool timeScaleFromCli = opts.timeScale > 0.0f;
  const float timeScale = timeScaleFromCli ? opts.timeScale : loader.getTimeScale();
  VEHICLE_LOG("  time-scale = " << timeScale << (timeScaleFromCli ? " (CLI)" : " (config.json)"));

  DronePhysics physics = DronePhysics(droneConfig);

  const UdpLink udp(opts.apHost, opts.apPort, opts.ownPort);
  if (!udp.isOpen()) {
    VEHICLE_LOG("Failed to open UDP link to " << opts.apHost << ":" << opts.apPort << " on own port " << opts.ownPort);
    return 1;
  }
  const mav::MavlinkEndpoint endpoint(udp, MAVLINK_COMM_1);

  // Дублювання телеметрії на GCS
  const UdpLink gcsUdp(opts.gcsHost, opts.gcsPort);
  if (!gcsUdp.isOpen()) {
    VEHICLE_LOG("Failed to open UDP link to " << opts.gcsHost << ":" << opts.gcsPort);
    return 1;
  }
  const mav::MavlinkEndpoint gcsEndpoint(gcsUdp, MAVLINK_COMM_2);

  constexpr std::chrono::seconds heartbeatPeriod = std::chrono::seconds(1);
  std::chrono::steady_clock::time_point lastHeartbeat;

  std::chrono::time_point last = std::chrono::steady_clock::now();
  float accumulator = 0.0f;
  float nextTelemetryLog = 0.0f;

  bool dropped = false;

  mav::RadioControlOverride lastAutopilotOverride{.roll = mav::kPwmNeutral, .throttle = mav::kPwmNeutral};
  mav::RadioControlOverride lastOperatorOverride{.roll = mav::kPwmNeutral, .throttle = mav::kPwmNeutral};

  bool MISSION_COMPLETE = false;

  while (!MISSION_COMPLETE) {
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
        const Coord aimPoint{.x = static_cast<float>(drop.latitude), .y = static_cast<float>(drop.longitude)};

        const DroneTelemetry telemetryAtDrop = physics.getTelemetry();

        VEHICLE_LOG("DROP t=" << telemetryAtDrop.timeSinceStart << " aim=(" << aimPoint.x << "," << aimPoint.y << ")");
      }
      else if (msg.msgid == MAVLINK_MSG_ID_COMMAND_LONG && mavlink_msg_command_long_get_command(&msg) == mav::kMissionCompleteCommandId) {
        VEHICLE_LOG("mission complete notification received");
        MISSION_COMPLETE = true;
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

  if (MISSION_COMPLETE) {
    // Виключно для зручності тестування, щоб не вбивати процесс вручну
    std::exit(0);
  }
}

// NOLINTEND(cppcoreguidelines-pro-bounds-pointer-arithmetic)
