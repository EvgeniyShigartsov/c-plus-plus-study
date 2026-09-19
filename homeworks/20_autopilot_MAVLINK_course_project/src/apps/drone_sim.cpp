#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "Logger.hpp"
#include "drone/DroneNode.hpp"
#include "interfaces/IDroneOutput.hpp"
#include "link/ILink.hpp"
#include "link/SerialLink.hpp"
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
  // Якщо задано - замість UDP дані шлються по UART (імітація ESP32), а приймає їх окремий
  // процесс SerialBridge (на залізі це Raspberry PI), і вже він розсилає дані решті системи по UDP.
  // Режим для імітації ESP32 при перевірці SerialBridge без реального заліза
  std::string serialDevice;
  int baudRate = 115200;
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
    else if (key == "--serial-device") {
      opts.serialDevice = value;
    }
    else if (key == "--baud") {
      opts.baudRate = std::stoi(value);
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

// DroneNode віддає повідомлення у кожен із заданих ендпоінтів.
// UDP-режим: автопілот і GCS (QGC/чекер). Serial-режим: лише UART порт, далі розсилає міст
class DroneOutput : public IDroneOutput {
public:
  explicit DroneOutput(std::vector<const mav::MavlinkEndpoint*> endpoints)
    : endpoints(std::move(endpoints))
  {
  }

  void send(const mavlink_message_t& msg) override
  {
    for (const mav::MavlinkEndpoint* endpoint : endpoints) {
      endpoint->send(msg);
    }
  }

  void onDrop(const Coord& aimPoint, const float timeSinceStart) override
  {
    DRONE_LOG("DROP t=" << timeSinceStart << " aim=(" << aimPoint.x << "," << aimPoint.y << ")");
  }

  void onMissionComplete() override { DRONE_LOG("mission complete notification received"); }

private:
  std::vector<const mav::MavlinkEndpoint*> endpoints;
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
              << "  gcs-port   = " << opts.gcsPort << '\n'
              << "  serial     = " << (opts.serialDevice.empty() ? "(off, UDP mode)" : opts.serialDevice) << " @" << opts.baudRate);

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

  const bool isSerialMode = !opts.serialDevice.empty();

  // Основний канал: UDP до автопілота або UART порт до моста
  std::unique_ptr<ILink> link;
  if (isSerialMode) {
    link = std::make_unique<SerialLink>(opts.serialDevice, opts.baudRate);
    if (!link->isOpen()) {
      DRONE_LOG("Failed to open serial device " << opts.serialDevice << " @" << opts.baudRate);
      return 1;
    }
    DRONE_LOG("Serial mode: " << opts.serialDevice << " @" << opts.baudRate);
  }
  else {
    link = std::make_unique<UdpLink>(opts.apHost, opts.apPort, opts.ownPort);
    if (!link->isOpen()) {
      DRONE_LOG("Failed to open UDP link to " << opts.apHost << ":" << opts.apPort << " on own port " << opts.ownPort);
      return 1;
    }
  }
  const mav::MavlinkEndpoint mainEndpoint(*link, MAVLINK_COMM_1);

  std::vector<const mav::MavlinkEndpoint*> outputs{&mainEndpoint};

  // Дублювання телеметрії на GCS: лише в UDP-режимі, у serial-режимі це робить SerialBridge
  std::unique_ptr<UdpLink> gcsUdp;
  std::unique_ptr<mav::MavlinkEndpoint> gcsEndpoint;
  if (!isSerialMode) {
    gcsUdp = std::make_unique<UdpLink>(opts.gcsHost, opts.gcsPort);

    if (!gcsUdp->isOpen()) {
      DRONE_LOG("Failed to open UDP link to " << opts.gcsHost << ":" << opts.gcsPort);
      return 1;
    }
    gcsEndpoint = std::make_unique<mav::MavlinkEndpoint>(*gcsUdp, MAVLINK_COMM_2);
    outputs.push_back(gcsEndpoint.get());
  }

  DroneOutput output(std::move(outputs));
  DroneNode node(droneConfig, physicsTimeStep, timeScale, output);

  std::chrono::time_point last = std::chrono::steady_clock::now();

  while (!node.isMissionComplete()) {
    for (const mavlink_message_t& msg : mainEndpoint.poll()) {
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
