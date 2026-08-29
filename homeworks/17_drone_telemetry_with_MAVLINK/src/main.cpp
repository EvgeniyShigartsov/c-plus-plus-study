#include <unistd.h>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "Logger.hpp"
#include "MissionProcessor.hpp"
#include "control/DroneController.hpp"
#include "gpio/GpioSignals.hpp"
#include "link/MavlinkDrop.hpp"
#include "link/MavlinkReporter.hpp"
#include "link/MissionConfig.hpp"
#include "link/PacketMappers.hpp"
#include "link/UartLink.hpp"
#include "link/UdpLink.hpp"
#include "providers/UartTargetProvider.hpp"
#include "solvers/TableSolver.hpp"
#include "third_party/drone_link.h"
#include "types.hpp"

const std::string BALLISTIC_TABLE_FILE_PATH = "data/ballistic_table.txt";

// NOLINTBEGIN(cppcoreguidelines-pro-bounds-array-to-pointer-decay,cppcoreguidelines-avoid-c-arrays)
// NOLINTBEGIN(cppcoreguidelines-pro-bounds-pointer-arithmetic)

struct CliOptions {
  std::string uartDev = "/tmp/ttyA";
  std::string gpioChip = "gpiochip1";
  std::string ballisticTable = BALLISTIC_TABLE_FILE_PATH;
  std::string mavHost = "127.0.0.1";
  uint16_t mavPort = 14550;
  int startLine = 24;
  int dropLine = 23;
};

CliOptions parseArgs(const std::vector<std::string>& args)
{
  CliOptions opts;

  for (size_t i = 0; i + 1 < args.size(); i += 2) {
    const std::string& key = args[i];
    const std::string& value = args[i + 1];

    if (key == "--uart") {
      opts.uartDev = value;
    }
    else if (key == "--gpiochip") {
      opts.gpioChip = value;
    }
    else if (key == "--start-line") {
      opts.startLine = std::stoi(value);
    }
    else if (key == "--drop-line") {
      opts.dropLine = std::stoi(value);
    }
    else if (key == "--ballistic-table") {
      opts.ballisticTable = value;
    }
    else if (key == "--mav-dest") {
      const size_t colon = value.rfind(':');
      if (colon != std::string::npos) {
        opts.mavHost = value.substr(0, colon);
        opts.mavPort = static_cast<uint16_t>(std::stoi(value.substr(colon + 1)));
      }
    }
    else {
      std::cerr << "Unknown argument: " << key << "\n";
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

  UartLink link(opts.uartDev);

  if (!link.isOpen()) {
    return 1;
  }

  GpioSignals gpio(opts.gpioChip, opts.startLine, opts.dropLine);

  if (!gpio.isOpen()) {
    return 1;
  }

  gpio.assertStart();  // START = 1 -> чекер запускає симуляцію

  LOG("Simulation started on " << opts.gpioChip << ", listening " << opts.uartDev << "\n");

  dlink::Parser parser;

  const MissionConfigPackets mission = collectMissionConfig(link, parser);

  const DroneConfig droneConfig = buildDroneConfig(mission.ammo, mission.droneCfg, mission.firstTelemetry);
  const BombParams bombParams = toBombParams(mission.ammo);
  const DroneTelemetry firstTelemetry = toDroneTelemetry(mission.firstTelemetry);

  auto targetProvider = std::make_shared<UartTargetProvider>(static_cast<int>(mission.ammo.nTargets));

  for (size_t i = 0; i < mission.initialTargets.size(); i++) {
    targetProvider->update(static_cast<int>(i), mission.initialTargets[i], firstTelemetry.timeSinceStart);
  }

  auto solver = std::make_unique<TableSolver>(opts.ballisticTable, bombParams, droneConfig);

  if (!solver->isLoadSuccess()) {
    std::cerr << "ballistic table load failed: " << opts.ballisticTable << "\n";
    return 1;
  }

  MissionProcessor missionProcessor(targetProvider, std::move(solver));

  if (!missionProcessor.init(droneConfig)) {
    return 1;
  }

  const DroneController controller(droneConfig);

  UdpLink udp(opts.mavHost, opts.mavPort);
  if (!udp.isOpen()) {
    return 1;
  }
  MavlinkReporter reporter(udp);
  MavlinkDrop drop(udp);
  reporter.sendHeartbeatIfDue();

  LOG("MAVLink telemetry -> " << opts.mavHost << ":" << opts.mavPort << "\n");

  uint8_t buf[256];
  uint8_t type = 0;
  uint8_t len = 0;
  uint8_t payload[260];
  uint8_t udpBuf[512];

  float lastTimeSeconds = firstTelemetry.timeSinceStart;
  bool dropStarted = false;

  while (true) {
    const int udpBytes = udp.receive(udpBuf, sizeof(udpBuf));
    if (udpBytes > 0) {
      drop.feed(udpBuf, udpBytes);
    }
    reporter.sendHeartbeatIfDue();

    if (dropStarted) {
      drop.resendIfFailed();
      if (drop.finished()) {
        LOG((drop.acked() ? "MAVLink: drop confirmed by ACK" : "MAVLink: drop NOT confirmed, no ACK") << "\n");
        break;
      }
    }

    const int bytes = link.readBytes(buf, sizeof(buf));
    if (bytes <= 0) {
      usleep(1000);
      continue;
    }

    for (int i = 0; i < bytes; i++) {
      if (!parser.feed(buf[i], type, payload, len)) {
        continue;
      }

      if (type == dlink::PKT_TARGET) {
        dlink::TargetPos tp{};
        std::memcpy(&tp, payload, sizeof tp);
        targetProvider->update(tp.id, {tp.x, tp.y}, lastTimeSeconds);
      }
      else if (type == dlink::PKT_TELEMETRY) {
        dlink::Telemetry t{};
        std::memcpy(&t, payload, sizeof t);

        reporter.sendTelemetry(t);

        if (dropStarted) {
          continue;  // скид відбувся, місію крутити не треба, тільки слати телеметрію
        }

        const DroneTelemetry telemetry = toDroneTelemetry(t);
        lastTimeSeconds = telemetry.timeSinceStart;

        const SimStep stepResult = missionProcessor.step(telemetry);
        const ControlSignal control = controller.compute(missionProcessor.getLastCommand(), telemetry);
        link.sendControl(control.accel, control.turnRate);

        if (!missionProcessor.hasNext()) {
          double dropLat = 0.0;
          double dropLon = 0.0;
          MavlinkReporter::toGps(t.x, t.y, dropLat, dropLon);

          LOG("DROP! t=" << telemetry.timeSinceStart << "s pos=(" << t.x << "," << t.y << ") target=" << stepResult.targetIdx << "\n");
          gpio.pulseDrop();
          drop.begin(dropLat, dropLon, t.z);
          dropStarted = true;
        }
      }
    }
  }

  return 0;
}
// NOLINTEND(cppcoreguidelines-pro-bounds-pointer-arithmetic)
// NOLINTEND(cppcoreguidelines-pro-bounds-array-to-pointer-decay,cppcoreguidelines-avoid-c-arrays)
