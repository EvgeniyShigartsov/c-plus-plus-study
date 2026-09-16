// autopilot — модуль-автопілот
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "Logger.hpp"
#include "authority/AuthorityStateMachine.hpp"
#include "control/DroneController.hpp"
#include "link/UdpLink.hpp"
#include "mavlink/Codec.hpp"
#include "mavlink/Endpoint.hpp"
#include "MissionProcessor.hpp"
#include "providers/CachedTargetProvider.hpp"
#include "sim/FileConfigLoader.hpp"
#include "solvers/TableSolver.hpp"
#include "third_party/json.hpp"

#define AUTOPILOT_LOG(msg) LOG("[AUTOPILOT]: " << msg)
#define AUTOPILOT_DEBUG(msg) DEBUG("[AUTOPILOT]: " << msg)

using json = nlohmann::json;

// NOLINTBEGIN(cppcoreguidelines-pro-bounds-pointer-arithmetic)

const std::string defaultDataDir = "homeworks/20_autopilot_MAVLINK_course_project/data";

struct CliOptions {
  std::string configPath = defaultDataDir + "/config.json";
  std::string ammoPath = defaultDataDir + "/ammo.json";
  std::string ballisticTable = defaultDataDir + "/ballistic_table.txt";
  std::string gcsHost = "127.0.0.1";
  uint16_t gcsPort = 14550;
  std::string droneHost = "127.0.0.1";
  uint16_t dronePort = 14555;  // домашній порт drone_sim - туди шлемо RC_CHANNELS_OVERRIDE
  uint16_t ownPort = 14560;  // домашній порт автопілота - сюди отримуємо телеметрію від drone_sim
  float heartbeatTimeoutSec =
    3.0f;  // Скільки секунд чекати HEARTBEAT від оператора перш ніж перейти у failsafe, ділиться на timeScale щоб час йшов консистентно
  float dropRefusalTimeoutSec = 5.0f;  // Якщо HEARTBEAT оператора мовчить довше цього - значить нема актуальних даних по цілям, розрахунок
                                       // точки скиду з кожним тіком стає менш точним, але якщо дрон досяг точки скиду раніше за цей таймаут
                                       // - спробувати уразити ціль. Так само ділиться на timeScale для консистентності часу.
  float timeScale = -1.0f;             // -1 = не задано явно, береться з config.json
  std::string simOutput = "simulation.json";  // реальний (не заглушковий) лог кроків наведення, легасі-формат ДЗ-09
};

CliOptions parseArgs(const std::vector<std::string>& args)
{
  CliOptions opts;

  for (size_t i = 0; i + 1 < args.size(); i += 2) {
    const std::string& key = args[i];
    const std::string& value = args[i + 1];

    if (key == "--config-path") {
      opts.configPath = value;
    }
    else if (key == "--ammo-path") {
      opts.ammoPath = value;
    }
    else if (key == "--ballistic-table") {
      opts.ballisticTable = value;
    }
    else if (key == "--gcs-host") {
      opts.gcsHost = value;
    }
    else if (key == "--gcs-port") {
      opts.gcsPort = static_cast<uint16_t>(std::stoi(value));
    }
    else if (key == "--drone-host") {
      opts.droneHost = value;
    }
    else if (key == "--drone-port") {
      opts.dronePort = static_cast<uint16_t>(std::stoi(value));
    }
    else if (key == "--own-port") {
      opts.ownPort = static_cast<uint16_t>(std::stoi(value));
    }
    else if (key == "--heartbeat-timeout") {
      opts.heartbeatTimeoutSec = std::stof(value);
    }
    else if (key == "--drop-refusal-timeout") {
      opts.dropRefusalTimeoutSec = std::stof(value);
    }
    else if (key == "--time-scale") {
      opts.timeScale = std::stof(value);
    }
    else if (key == "--sim-output") {
      opts.simOutput = value;
    }
    else {
      std::cerr << "Unknown argument at autopilot.cpp: " << key << '\n';
    }
  }

  return opts;
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

int main(int argc, char* argv[])
{
  std::vector<std::string> args;
  for (int i = 1; i < argc; i++) {
    args.emplace_back(argv[i]);
  }
  const CliOptions opts = parseArgs(args);

  AUTOPILOT_LOG("autopilot:\n"
                << "  config-path     = " << opts.configPath << '\n'
                << "  ammo-path       = " << opts.ammoPath << '\n'
                << "  ballistic-table = " << opts.ballisticTable << '\n'
                << "  gcs-host        = " << opts.gcsHost << '\n'
                << "  gcs-port        = " << opts.gcsPort << '\n'
                << "  drone-host      = " << opts.droneHost << '\n'
                << "  drone-port      = " << opts.dronePort << '\n'
                << "  own-port        = " << opts.ownPort << '\n'
                << "  heartbeat-timeout = " << opts.heartbeatTimeoutSec << '\n'
                << "  drop-refusal-timeout = " << opts.dropRefusalTimeoutSec << '\n'
                << "  sim-output      = " << opts.simOutput);

  FileConfigLoader loader;
  if (!loader.load(opts.configPath, opts.ammoPath)) {
    AUTOPILOT_LOG("Failed to load config or ammo");
    return 1;
  }
  const DroneConfig droneConfig = loader.getConfig();
  const BombParams ammo = loader.getAmmoParams();

  const bool timeScaleFromCli = opts.timeScale > 0.0f;
  const float timeScale = timeScaleFromCli ? opts.timeScale : loader.getTimeScale();
  AUTOPILOT_LOG("  time-scale = " << timeScale << (timeScaleFromCli ? " (CLI)" : " (config.json)"));

  const DroneController controller(droneConfig);

  const UdpLink homeUdp(opts.droneHost, opts.dronePort, opts.ownPort);
  if (!homeUdp.isOpen()) {
    AUTOPILOT_LOG("Failed to open UDP link to " << opts.droneHost << ":" << opts.dronePort << " on own port " << opts.ownPort);
    return 1;
  }

  auto ballisticSolver = std::make_unique<TableSolver>(opts.ballisticTable, ammo, droneConfig);

  if (!ballisticSolver->isLoadSuccess()) {
    AUTOPILOT_LOG("Failed to load ballistic table " << opts.ballisticTable);
    return 1;
  }

  const mav::MavlinkEndpoint homeEndpoint(homeUdp, MAVLINK_COMM_1);

  const UdpLink gcsUdp(opts.gcsHost, opts.gcsPort);
  if (!gcsUdp.isOpen()) {
    AUTOPILOT_LOG("Failed to open UDP link to " << opts.gcsHost << ":" << opts.gcsPort);
    return 1;
  }
  const mav::MavlinkEndpoint gcsEndpoint(gcsUdp, MAVLINK_COMM_2);

  AuthorityStateMachine authority;
  bool enabled = false;
  bool operatorInDeadband = true;

  const auto heartbeatTimeout = std::chrono::duration<float>(opts.heartbeatTimeoutSec / timeScale);
  const auto dropRefusalTimeout = std::chrono::duration<float>(opts.dropRefusalTimeoutSec / timeScale);
  std::chrono::steady_clock::time_point lastOperatorHeartbeat;
  bool dropRefusedLogged = false;

  mav::LocalPositionNed lastPosition{};
  mav::Attitude lastAttitude{};
  bool havePosition = false;
  bool haveAttitude = false;
  DroneTelemetry lastTelemetry{};

  std::shared_ptr<CachedTargetProvider> targets;
  std::unique_ptr<MissionProcessor> mission;
  SimStep lastStep{};
  std::vector<SimStep> stepsLog;

  constexpr std::chrono::seconds heartbeatPeriod = std::chrono::seconds(1);
  std::chrono::steady_clock::time_point lastOwnHeartbeat;

  bool MISSION_COMPLETE = false;

  while (!MISSION_COMPLETE) {
    for (const mavlink_message_t& msg : homeEndpoint.poll()) {
      bool telemetryUpdated = false;

      if (msg.msgid == MAVLINK_MSG_ID_LOCAL_POSITION_NED) {
        lastPosition = mav::parse_local_position_ned(msg);
        havePosition = true;
        telemetryUpdated = true;
      }
      else if (msg.msgid == MAVLINK_MSG_ID_ATTITUDE) {
        lastAttitude = mav::parse_attitude(msg);
        haveAttitude = true;
        telemetryUpdated = true;
      }
      else if (msg.msgid == MAVLINK_MSG_ID_COMMAND_INT && mavlink_msg_command_int_get_command(&msg) == mav::kTargetDesignationCommandId) {
        const mav::TargetDesignation designation = mav::parse_target_designation(msg);

        if (!targets) {
          targets = std::make_shared<CachedTargetProvider>(designation.target_count);
          AUTOPILOT_LOG("targets: create cache for " << static_cast<int>(designation.target_count) << " targets");
        }

        const Coord pos{.x = static_cast<float>(designation.latitude), .y = static_cast<float>(designation.longitude)};
        targets->update(designation.target_id, pos, lastTelemetry.timeSinceStart);

        if (!mission) {
          mission = std::make_unique<MissionProcessor>(targets, std::move(ballisticSolver));
          const bool isMissionInitSucces = mission->init(droneConfig);

          AUTOPILOT_LOG((isMissionInitSucces ? "mission: guidance core ready" : "mission: failed to init mission processor"));
        }
      }
      else if (msg.msgid == MAVLINK_MSG_ID_HEARTBEAT && mav::Identity{.sysid = msg.sysid, .compid = msg.compid} == mav::kGcs) {
        const bool wasHeartbeatOk = std::chrono::steady_clock::now() - lastOperatorHeartbeat < heartbeatTimeout;
        lastOperatorHeartbeat = std::chrono::steady_clock::now();

        if (!wasHeartbeatOk) {
          AUTOPILOT_LOG("operator heartbeat: restored");
        }
      }
      else if (msg.msgid == MAVLINK_MSG_ID_RC_CHANNELS) {
        const mav::RadioControlChannels channels = mav::parse_radio_control_channels(msg);
        const bool wasInDeadband = operatorInDeadband;

        operatorInDeadband = mav::are_channels_in_deadband(channels);

        if (operatorInDeadband != wasInDeadband) {
          AUTOPILOT_LOG("operator " << (operatorInDeadband ? "released sticks" : "touched sticks") << " roll=" << channels.roll
                                    << " throttle=" << channels.throttle);
        }
      }
      else if (msg.msgid == MAVLINK_MSG_ID_COMMAND_LONG && mavlink_msg_command_long_get_command(&msg) == mav::kEnableCommandId) {
        const mav::EnableCommand command = mav::parse_enable_command(msg);
        enabled = command.enabled;
        AUTOPILOT_LOG("enable: " << (enabled ? "true" : "false"));

        gcsEndpoint.send(
          mav::pack_command_acknowledgement(mav::kAutopilot, mav::kGcs, {.command = mav::kEnableCommandId, .result = MAV_RESULT_ACCEPTED}));
      }

      if (telemetryUpdated && havePosition && haveAttitude) {
        const VehicleState state = mav::to_vehicle_state(lastPosition, lastAttitude);
        lastTelemetry = mav::to_drone_telemetry(state);

        const bool hasNextStep = mission && mission->hasNext();

        if (hasNextStep) {
          lastStep = mission->step(lastTelemetry);
          stepsLog.push_back(lastStep);
        }

        const bool operatorHeartbeatOk = std::chrono::steady_clock::now() - lastOperatorHeartbeat < heartbeatTimeout;

        const bool hasAuthorityChanged = authority.update({
          .enabled = enabled,
          .hasMission = mission != nullptr,
          .operatorInDeadband = operatorInDeadband,
          .operatorHeartbeatOk = operatorHeartbeatOk,
          .reachedFirePoint = mission && !hasNextStep,
        });
        if (hasAuthorityChanged) {
          AUTOPILOT_LOG("state changed to: -> " << authority.to_string());

          const bool isFailsafe = authority.state() == AuthorityState::Failsafe;
          gcsEndpoint.send(mav::pack_status_text(mav::kAutopilot,
                                                 {.severity = static_cast<uint8_t>(isFailsafe ? MAV_SEVERITY_WARNING : MAV_SEVERITY_INFO),
                                                  .text = "state -> " + authority.to_string()}));
        }

        if (authority.state() == AuthorityState::Complete && !MISSION_COMPLETE) {
          const bool heartbeatTooStaleToDrop = std::chrono::steady_clock::now() - lastOperatorHeartbeat > dropRefusalTimeout;

          if (heartbeatTooStaleToDrop) {
            if (!dropRefusedLogged) {
              AUTOPILOT_LOG("mission: reached fire point, but operator heartbeat expired > " << opts.dropRefusalTimeoutSec
                                                                                             << "s -- refusing to drop, waiting");
              dropRefusedLogged = true;
            }
          }
          else {
            homeEndpoint.send(mav::pack_drop_notification(mav::kAutopilot,
                                                          mav::kVehicle,
                                                          {.latitude = lastStep.predictedTarget.x,
                                                           .longitude = lastStep.predictedTarget.y,
                                                           .altitude = 0.0f,
                                                           .target_id = static_cast<uint8_t>(lastStep.targetIdx),
                                                           .bomb_flight_time_sec = mission->getBombFlightTime()}));

            AUTOPILOT_LOG("mission: complete, aiming at (" << lastStep.predictedTarget.x << "," << lastStep.predictedTarget.y
                                                           << ") target=" << lastStep.targetIdx);

            writeSimulationJson(stepsLog, opts.simOutput);
            AUTOPILOT_LOG("simulation.json written: " << stepsLog.size() << " steps -> " << opts.simOutput);

            const mavlink_message_t missionCompleteMsg =
              mav::pack_mission_complete_notification(mav::kAutopilot, mav::kVehicle, {.completed = true});
            homeEndpoint.send(missionCompleteMsg);
            gcsEndpoint.send(missionCompleteMsg);

            MISSION_COMPLETE = true;
            AUTOPILOT_LOG("mission complete");
          }
        }

        if (hasNextStep && authority.hasControl()) {
          const ControlSignal control = controller.compute(mission->getLastCommand(), lastTelemetry);
          homeEndpoint.send(mav::pack_radio_control_channels_override(mav::kAutopilot, mav::kVehicle, control));

          AUTOPILOT_DEBUG("guidance t=" << lastTelemetry.timeSinceStart << " pos=(" << lastTelemetry.pos.x << "," << lastTelemetry.pos.y
                                        << ") state=" << lastStep.state << " target=" << lastStep.targetIdx << " dropPoint=("
                                        << lastStep.dropPoint.x << "," << lastStep.dropPoint.y << ") accel=" << control.accel
                                        << " turnRate=" << control.turnRate);
        }
        else {
          AUTOPILOT_DEBUG("telemetry t=" << lastTelemetry.timeSinceStart << " pos=(" << lastTelemetry.pos.x << "," << lastTelemetry.pos.y
                                         << ") speed=" << lastTelemetry.speed << " dir=" << lastTelemetry.dir);
        }
      }
    }

    const std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now();

    if (now - lastOwnHeartbeat >= heartbeatPeriod) {
      gcsEndpoint.send(mav::pack_heartbeat(
        mav::kAutopilot,
        {.type = MAV_TYPE_ONBOARD_CONTROLLER, .custom_mode = static_cast<uint32_t>(authority.state()), .system_status = MAV_STATE_ACTIVE}));
      lastOwnHeartbeat = now;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }

  if (MISSION_COMPLETE) {
    // Виключно для зручності тестування, щоб не вбивати процесс вручну
    std::exit(0);
  }
}

// NOLINTEND(cppcoreguidelines-pro-bounds-pointer-arithmetic)
