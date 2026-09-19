// serial_bridge - міст між дроном на послідовному порту (ESP32) і рештою системи по UDP
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

#include "Logger.hpp"
#include "bridge/SerialBridge.hpp"
#include "link/SerialLink.hpp"
#include "link/UdpLink.hpp"
#include "mavlink/Endpoint.hpp"

#define BRIDGE_LOG(msg) LOG("[BRIDGE]: " << msg)

// NOLINTBEGIN(cppcoreguidelines-pro-bounds-pointer-arithmetic)

struct CliOptions {
  std::string serialDevice = "/dev/serial0";  // порт до дрона
  int baudRate = 115200;                      // швидкість передачі
  uint16_t dronePort = 14555;  // порт дрона - сюди автопілот і оператор шлють повідомлення
  std::string apHost = "127.0.0.1";
  uint16_t apPort = 14560;  // домашній порт автопілота, сюди пересилається телеметрія дрона
  std::string gcsHost = "127.0.0.1";
  uint16_t gcsPort = 14550;
};

CliOptions parseArgs(const std::vector<std::string>& args)
{
  CliOptions opts;

  for (size_t i = 0; i + 1 < args.size(); i += 2) {
    const std::string& key = args[i];
    const std::string& value = args[i + 1];

    if (key == "--serial-device") {
      opts.serialDevice = value;
    }
    else if (key == "--baud") {
      opts.baudRate = std::stoi(value);
    }
    else if (key == "--drone-port") {
      opts.dronePort = static_cast<uint16_t>(std::stoi(value));
    }
    else if (key == "--ap-host") {
      opts.apHost = value;
    }
    else if (key == "--ap-port") {
      opts.apPort = static_cast<uint16_t>(std::stoi(value));
    }
    else if (key == "--gcs-host") {
      opts.gcsHost = value;
    }
    else if (key == "--gcs-port") {
      opts.gcsPort = static_cast<uint16_t>(std::stoi(value));
    }
    else {
      std::cerr << "Unknown argument at serial_bridge.cpp: " << key << '\n';
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

  const SerialLink serialLink(opts.serialDevice, opts.baudRate);
  if (!serialLink.isOpen()) {
    BRIDGE_LOG("Failed to open serial device " << opts.serialDevice);
    return 1;
  }

  // Порт для прийому повідомлень від решти системи, оператора/автопілота
  const UdpLink appsUdp("127.0.0.1", opts.dronePort, opts.dronePort);

  // Порт для відправки даних автопілоту
  const UdpLink autopilotUdp(opts.apHost, opts.apPort);

  // Порт для відправки даних на GCS
  const UdpLink gcsUdp(opts.gcsHost, opts.gcsPort);
  if (!appsUdp.isOpen() || !autopilotUdp.isOpen() || !gcsUdp.isOpen()) {
    BRIDGE_LOG("Failed to open UDP links");
    return 1;
  }

  const mav::MavlinkEndpoint drone(serialLink, MAVLINK_COMM_0);
  const mav::MavlinkEndpoint fromApps(appsUdp, MAVLINK_COMM_1);
  const mav::MavlinkEndpoint toAutopilot(autopilotUdp, MAVLINK_COMM_2);
  const mav::MavlinkEndpoint toGcs(gcsUdp, MAVLINK_COMM_3);
  SerialBridge bridge(drone, fromApps, toAutopilot, toGcs);

  BRIDGE_LOG("serial " << opts.serialDevice << " @" << opts.baudRate << " <-> UDP: drone port " << opts.dronePort << ", autopilot "
                       << opts.apHost << ":" << opts.apPort << ", gcs " << opts.gcsHost << ":" << opts.gcsPort);

  while (!bridge.isMissionComplete()) {
    bridge.pump();
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  // Для зручності тестування, щоб не вбивати процесс вручну
  BRIDGE_LOG("mission complete, exiting");
  std::exit(0);
}

// NOLINTEND(cppcoreguidelines-pro-bounds-pointer-arithmetic)
