// operator_sim — тестовий скриптований оператор
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
  std::string apHost = "127.0.0.1";
  uint16_t apPort = 14560;  // домашній порт autopilot, сюди шлемо HEARTBEAT або команди
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
    else {
      std::cerr << "Unknown argument at operator_sim.cpp: " << key << '\n';
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

  LOG("operator_sim:\n"
      << "  ap-host = " << opts.apHost << '\n'
      << "  ap-port = " << opts.apPort);

  const UdpLink udp(opts.apHost, opts.apPort);
  if (!udp.isOpen()) {
    LOG("Failed to open UDP link to " << opts.apHost << ":" << opts.apPort);
    return 1;
  }
  const mav::MavlinkEndpoint endpoint(udp, MAVLINK_COMM_1);

  const std::chrono::milliseconds kHeartbeatPeriod = std::chrono::milliseconds(500);  // 2 Гц
  std::chrono::steady_clock::time_point lastHeartbeat;

  while (true) {
    const std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now();
    if (now - lastHeartbeat >= kHeartbeatPeriod) {
      endpoint.send(mav::pack_heartbeat(mav::kGcs, {.type = MAV_TYPE_GCS}));
      lastHeartbeat = now;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
}

// NOLINTEND(cppcoreguidelines-pro-bounds-pointer-arithmetic)
