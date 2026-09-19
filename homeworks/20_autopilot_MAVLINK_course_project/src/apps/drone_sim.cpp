#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

#include "Logger.hpp"
#include "drone/DroneNode.hpp"
#include "interfaces/IDroneOutput.hpp"
#include "link/UdpLink.hpp"
#include "mavlink/Endpoint.hpp"
#include "sim/FileConfigLoader.hpp"

#define DRONE_LOG(msg) LOG("[DRONE]: " << msg)
#define DRONE_DEBUG(msg) DEBUG("[DRONE]: " << msg)

// NOLINTBEGIN(cppcoreguidelines-pro-bounds-pointer-arithmetic, cppcoreguidelines-special-member-functions)
// NOLINTBEGIN(cppcoreguidelines-avoid-const-or-ref-data-members)

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
      std::cerr << "Unknown argument at drone_sim.cpp: " << key << '\n';
    }
  }

  return opts;
}

class UdpDroneOutput : public IDroneOutput {
public:
  UdpDroneOutput(const mav::MavlinkEndpoint& autopilotEndpoint, const mav::MavlinkEndpoint& gcsEndpoint)
    : autopilotEndpoint(autopilotEndpoint)
    , gcsEndpoint(gcsEndpoint)
  {
  }

  void send(const mavlink_message_t& msg) override
  {
    autopilotEndpoint.send(msg);
    gcsEndpoint.send(msg);
  }

  void onDrop(const Coord& aimPoint, const float timeSinceStart) override
  {
    DRONE_LOG("DROP t=" << timeSinceStart << " aim=(" << aimPoint.x << "," << aimPoint.y << ")");
  }

  void onMissionComplete() override { DRONE_LOG("mission complete notification received"); }

private:
  const mav::MavlinkEndpoint& autopilotEndpoint;
  const mav::MavlinkEndpoint& gcsEndpoint;
};

int main(int argc, char* argv[])
{
  std::vector<std::string> args;
  for (int i = 1; i < argc; i++) {
    args.emplace_back(argv[i]);
  }
  const CliOptions opts = parseArgs(args);

  DRONE_DEBUG("drone_sim:\n"
              << "  ap-host    = " << opts.apHost << '\n'
              << "  ap-port    = " << opts.apPort << '\n'
              << "  own-port   = " << opts.ownPort << '\n'
              << "  gcs-host   = " << opts.gcsHost << '\n'
              << "  gcs-port   = " << opts.gcsPort);

  FileConfigLoader loader;
  if (!loader.load(opts.configPath, opts.ammoPath)) {
    DRONE_LOG("Failed to load config or ammo");
    return 1;
  }

  const DroneConfig droneConfig = loader.getConfig();
  const float physicsTimeStep = loader.getPhysicsTimeStep();
  const bool timeScaleFromCli = opts.timeScale > 0.0f;
  const float timeScale = timeScaleFromCli ? opts.timeScale : loader.getTimeScale();
  DRONE_LOG("  time-scale = " << timeScale << (timeScaleFromCli ? " (CLI)" : " (config.json)"));

  const UdpLink udp(opts.apHost, opts.apPort, opts.ownPort);
  if (!udp.isOpen()) {
    DRONE_LOG("Failed to open UDP link to " << opts.apHost << ":" << opts.apPort << " on own port " << opts.ownPort);
    return 1;
  }
  const mav::MavlinkEndpoint endpoint(udp, MAVLINK_COMM_1);

  // Дублювання телеметрії на GCS
  const UdpLink gcsUdp(opts.gcsHost, opts.gcsPort);
  if (!gcsUdp.isOpen()) {
    DRONE_LOG("Failed to open UDP link to " << opts.gcsHost << ":" << opts.gcsPort);
    return 1;
  }
  const mav::MavlinkEndpoint gcsEndpoint(gcsUdp, MAVLINK_COMM_2);

  UdpDroneOutput output(endpoint, gcsEndpoint);
  DroneNode node(droneConfig, physicsTimeStep, timeScale, output);

  std::chrono::time_point last = std::chrono::steady_clock::now();

  while (!node.isMissionComplete()) {
    for (const mavlink_message_t& msg : endpoint.poll()) {
      node.onMessage(msg);
    }

    const std::chrono::time_point now = std::chrono::steady_clock::now();
    node.update(std::chrono::duration<float>(now - last).count());
    last = now;

    std::this_thread::sleep_for(std::chrono::duration<float>(physicsTimeStep / timeScale));
  }

  // Виключно для зручності тестування, щоб не вбивати процесс вручну
  std::exit(0);
}

// NOLINTEND(cppcoreguidelines-avoid-const-or-ref-data-members)
// NOLINTEND(cppcoreguidelines-pro-bounds-pointer-arithmetic,cppcoreguidelines-special-member-functions)
