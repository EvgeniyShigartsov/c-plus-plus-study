// autopilot — модуль-автопілот
#include <chrono>
#include <cstdint>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

#include "Logger.hpp"
#include "link/UdpLink.hpp"
#include "mavlink/Codec.hpp"
#include "mavlink/Endpoint.hpp"

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

      if (telemetryUpdated && havePosition && haveAttitude) {
        const VehicleState state = mav::to_vehicle_state(lastPosition, lastAttitude);
        const DroneTelemetry telemetry = mav::to_drone_telemetry(state);
        LOG("telemetry t=" << telemetry.timeSinceStart << " pos=(" << telemetry.pos.x << "," << telemetry.pos.y
                           << ") speed=" << telemetry.speed << " dir=" << telemetry.dir);
      }
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
}

// NOLINTEND(cppcoreguidelines-pro-bounds-pointer-arithmetic)
