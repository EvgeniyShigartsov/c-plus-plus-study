// autopilot — модуль-автопілот
#include <chrono>
#include <cstdint>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "Logger.hpp"
#include "control/DroneController.hpp"
#include "link/UdpLink.hpp"
#include "mavlink/Codec.hpp"
#include "mavlink/Endpoint.hpp"
#include "MissionProcessor.hpp"
#include "providers/CachedTargetProvider.hpp"
#include "sim/FileConfigLoader.hpp"
#include "solvers/TableSolver.hpp"

// NOLINTBEGIN(cppcoreguidelines-pro-bounds-pointer-arithmetic)

const std::string defaultDataDir = "homeworks/20_autopilot_MAVLINK_course_project/data";

struct CliOptions {
  std::string configPath = defaultDataDir + "/config.json";
  std::string ammoPath = defaultDataDir + "/ammo.json";
  std::string ballisticTable = defaultDataDir + "/ballistic_table.txt";
  std::string gcsHost = "127.0.0.1";
  uint16_t gcsPort = 14550;
  std::string vehicleHost = "127.0.0.1";
  uint16_t vehiclePort = 14555;  // домашній порт vehicle_sim - туди шлемо RC_CHANNELS_OVERRIDE
  uint16_t ownPort = 14560;  // домашній порт автопілота - сюди отримуємо телеметрію від vehicle_sim
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
    else if (key == "--vehicle-host") {
      opts.vehicleHost = value;
    }
    else if (key == "--vehicle-port") {
      opts.vehiclePort = static_cast<uint16_t>(std::stoi(value));
    }
    else if (key == "--own-port") {
      opts.ownPort = static_cast<uint16_t>(std::stoi(value));
    }
    else {
      std::cerr << "Unknown argument at autopilot.cpp: " << key << '\n';
    }
  }

  return opts;
}

int main(int argc, char* argv[])
{
  std::vector<std::string> args;
  for (int i = 1; i < argc; i++) {
    args.emplace_back(argv[i]);
  }
  const CliOptions opts = parseArgs(args);

  LOG("autopilot:\n"
      << "  config-path     = " << opts.configPath << '\n'
      << "  ammo-path       = " << opts.ammoPath << '\n'
      << "  ballistic-table = " << opts.ballisticTable << '\n'
      << "  gcs-host        = " << opts.gcsHost << '\n'
      << "  gcs-port        = " << opts.gcsPort << '\n'
      << "  vehicle-host    = " << opts.vehicleHost << '\n'
      << "  vehicle-port    = " << opts.vehiclePort << '\n'
      << "  own-port        = " << opts.ownPort);

  FileConfigLoader loader;
  if (!loader.load(opts.configPath, opts.ammoPath)) {
    LOG("Failed to load config or ammo");
    return 1;
  }
  const DroneConfig droneConfig = loader.getConfig();
  const BombParams ammo = loader.getAmmoParams();

  const DroneController controller(droneConfig);

  const UdpLink udp(opts.vehicleHost, opts.vehiclePort, opts.ownPort);
  if (!udp.isOpen()) {
    LOG("Failed to open UDP link to " << opts.vehicleHost << ":" << opts.vehiclePort << " on own port " << opts.ownPort);
    return 1;
  }
  const mav::MavlinkEndpoint endpoint(udp, MAVLINK_COMM_1);

  mav::LocalPositionNed lastPosition{};
  mav::Attitude lastAttitude{};
  bool havePosition = false;
  bool haveAttitude = false;
  DroneTelemetry lastTelemetry{};

  std::shared_ptr<CachedTargetProvider> targets;
  std::unique_ptr<MissionProcessor> mission;
  SimStep lastStep{};
  bool reportedReady = false;

  while (true) {
    for (const mavlink_message_t& msg : endpoint.poll()) {
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
          LOG("targets: create cache for " << static_cast<int>(designation.target_count) << " targets");
        }

        const Coord pos{.x = static_cast<float>(designation.latitude), .y = static_cast<float>(designation.longitude)};
        targets->update(designation.target_id, pos, lastTelemetry.timeSinceStart);

        if (!mission) {
          auto solver = std::make_unique<TableSolver>(opts.ballisticTable, ammo, droneConfig);
          if (!solver->isLoadSuccess()) {
            LOG("Failed to load ballistic table " << opts.ballisticTable);
          }
          mission = std::make_unique<MissionProcessor>(targets, std::move(solver));
          if (!mission->init(droneConfig)) {
            LOG("Failed to init mission processor");
          }
          LOG("mission: guidance core ready");
        }
      }

      if (telemetryUpdated && havePosition && haveAttitude) {
        const VehicleState state = mav::to_vehicle_state(lastPosition, lastAttitude);
        lastTelemetry = mav::to_drone_telemetry(state);

        if (mission && mission->hasNext()) {
          lastStep = mission->step(lastTelemetry);
          const ControlSignal control = controller.compute(mission->getLastCommand(), lastTelemetry);
          endpoint.send(mav::pack_radio_control_channels_override(mav::kAutopilot, mav::kVehicle, control));

          LOG("guidance t=" << lastTelemetry.timeSinceStart << " pos=(" << lastTelemetry.pos.x << "," << lastTelemetry.pos.y
                            << ") state=" << lastStep.state << " target=" << lastStep.targetIdx << " dropPoint=(" << lastStep.dropPoint.x
                            << "," << lastStep.dropPoint.y << ") accel=" << control.accel << " turnRate=" << control.turnRate);
        }
        else if (mission && !reportedReady) {
          // Скид відбувається тут, поки що - лише сигнал у лог,
          LOG("mission: reached fire point, ready to drop -- last dropPoint=(" << lastStep.dropPoint.x << "," << lastStep.dropPoint.y
                                                                               << ")");
          reportedReady = true;
        }
        else {
          LOG("telemetry t=" << lastTelemetry.timeSinceStart << " pos=(" << lastTelemetry.pos.x << "," << lastTelemetry.pos.y
                             << ") speed=" << lastTelemetry.speed << " dir=" << lastTelemetry.dir);
        }
      }
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
}

// NOLINTEND(cppcoreguidelines-pro-bounds-pointer-arithmetic)
