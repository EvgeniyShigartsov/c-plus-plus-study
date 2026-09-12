// autopilot — модуль-автопілот
#include <chrono>
#include <cstdint>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "Logger.hpp"
#include "link/UdpLink.hpp"
#include "mavlink/Codec.hpp"
#include "mavlink/Endpoint.hpp"
#include "providers/CachedTargetProvider.hpp"

// NOLINTBEGIN(cppcoreguidelines-pro-bounds-pointer-arithmetic)

struct CliOptions {
  std::string gcsHost = "127.0.0.1";
  uint16_t gcsPort = 14550;
  std::string vehicleHost = "127.0.0.1";
  uint16_t vehiclePort = 14555;  // домашній порт vehicle_sim - туди шлемо RC_CHANNELS_OVERRIDE
  uint16_t ownPort = 14560;  //  домашній порт автопілота - сюди отримуємо телеметрію від vehicle_sim
  std::string ballisticTable = "data/ballistic_table.txt";
};

CliOptions parseArgs(const std::vector<std::string>& args)
{
  CliOptions opts;

  for (size_t i = 0; i + 1 < args.size(); i += 2) {
    const std::string& key = args[i];
    const std::string& value = args[i + 1];

    if (key == "--gcs-host") {
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
    else if (key == "--ballistic-table") {
      opts.ballisticTable = value;
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
      << "  gcs-host        = " << opts.gcsHost << '\n'
      << "  gcs-port        = " << opts.gcsPort << '\n'
      << "  vehicle-host    = " << opts.vehicleHost << '\n'
      << "  vehicle-port    = " << opts.vehiclePort << '\n'
      << "  own-port        = " << opts.ownPort << '\n'
      << "  ballistic-table = " << opts.ballisticTable);

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

  std::unique_ptr<CachedTargetProvider> targets;

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
          targets = std::make_unique<CachedTargetProvider>(designation.target_count);
          LOG("targets: create cache for " << designation.target_count << " targets");
        }

        const Coord pos{.x = static_cast<float>(designation.latitude), .y = static_cast<float>(designation.longitude)};
        targets->update(designation.target_id, pos, lastTelemetry.timeSinceStart);

        const Target updated = targets->getTarget(designation.target_id);
        LOG("target " << designation.target_id << " -> pos=(" << updated.pos.x << "," << updated.pos.y << ") vel=(" << updated.velocity.x
                      << "," << updated.velocity.y << ")");
      }

      if (telemetryUpdated && havePosition && haveAttitude) {
        const VehicleState state = mav::to_vehicle_state(lastPosition, lastAttitude);
        lastTelemetry = mav::to_drone_telemetry(state);
        LOG("telemetry t=" << lastTelemetry.timeSinceStart << " pos=(" << lastTelemetry.pos.x << "," << lastTelemetry.pos.y
                           << ") speed=" << lastTelemetry.speed << " dir=" << lastTelemetry.dir);
      }
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
}

// NOLINTEND(cppcoreguidelines-pro-bounds-pointer-arithmetic)
