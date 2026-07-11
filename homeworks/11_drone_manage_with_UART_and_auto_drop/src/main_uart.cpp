#include <unistd.h>
#include <cstring>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "MathUtils.hpp"
#include "MissionProcessor.hpp"
#include "control/DroneController.hpp"
#include "gpio/GpioSignals.hpp"
#include "link/UartLink.hpp"
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
    else {
      std::cerr << "Unknown argument: " << key << "\n";
    }
  }

  return opts;
}

DroneTelemetry toDroneTelemetry(const dlink::Telemetry& t)
{
  return {
    .pos = {t.x, t.y},
    .speed = t.speed,
    .dir = t.dir,
    .timeSinceStart = static_cast<float>(t.t_ms) / 1000.0f,
  };
}

DroneConfig buildDroneConfig(const dlink::AmmoCfg& ammo, const dlink::DroneCfg& cfg, const dlink::Telemetry& tele)
{
  return {
    .ammoName = ammo.name,
    .startPos = {tele.x, tele.y},
    .altitude = tele.z,
    .initialDir = tele.dir,
    .v0 = cfg.attackSpeed,
    .accelerationPath = cfg.accelerationPath,
    .arrayTimeStep = cfg.timeStep,
    .simTimeStep = cfg.timeStep,
    .hitRadius = ammo.hitRadius,
    .angularSpeed = cfg.angularSpeed,
    .turnThreshold = cfg.turnThreshold,
    .physicsTimeStep = cfg.timeStep,  // локальної фізики немає, поле не використовується
    .timeScale = cfg.timeScale,
  };
}

int main(int argc, char* argv[])
{
  const std::vector<std::string> args(argv + 1, argv + argc);
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

  std::cout << "hm11 uart autopilot: START=1 on " << opts.gpioChip << ", listening " << opts.uartDev << "\n";

  dlink::Parser parser;

  uint8_t buf[256];
  uint8_t type = 0;
  uint8_t len = 0;
  uint8_t payload[260];

  dlink::AmmoCfg ammo{};
  dlink::DroneCfg cfg{};
  dlink::Telemetry firstTele{};
  bool haveAmmo = false;
  bool haveCfg = false;
  bool haveTele = false;

  std::vector<Coord> initialTargets;
  std::vector<bool> targetSeen;
  int targetsSeen = 0;

  const bool waitingConfig = true;
  while (waitingConfig) {
    const int n = link.readBytes(buf, sizeof(buf));

    if (n <= 0) {
      usleep(1000);
      continue;
    }

    for (int i = 0; i < n; i++) {
      if (!parser.feed(buf[i], type, payload, len)) {
        continue;
      }

      if (type == dlink::PKT_AMMO) {
        std::memcpy(&ammo, payload, sizeof ammo);
        haveAmmo = true;
        initialTargets.resize(ammo.nTargets);
        targetSeen.resize(ammo.nTargets, false);
      }
      else if (type == dlink::PKT_CONFIG) {
        std::memcpy(&cfg, payload, sizeof cfg);
        haveCfg = true;
      }
      else if (type == dlink::PKT_TELEMETRY) {
        std::memcpy(&firstTele, payload, sizeof firstTele);
        haveTele = true;
      }
      else if (type == dlink::PKT_TARGET && haveAmmo) {
        dlink::TargetPos tp{};
        std::memcpy(&tp, payload, sizeof tp);

        if (tp.id < initialTargets.size() && !targetSeen[tp.id]) {
          initialTargets[tp.id] = {tp.x, tp.y};
          targetSeen[tp.id] = true;
          targetsSeen++;
        }
      }
    }

    if (haveAmmo && haveCfg && haveTele && targetsSeen == static_cast<int>(initialTargets.size())) {
      break;
    }
  }

  std::cout << "mission: ammo=" << ammo.name << " hitRadius=" << ammo.hitRadius << " targets=" << static_cast<int>(ammo.nTargets)
            << " | drone: v0=" << cfg.attackSpeed << " maxW=" << cfg.angularSpeed << " alt=" << firstTele.z << "\n";

  const DroneConfig droneConfig = buildDroneConfig(ammo, cfg, firstTele);
  const BombParams bombParams = {.name = ammo.name, .mass = ammo.mass, .drag = ammo.drag, .lift = ammo.lift};

  auto targetProvider = std::make_shared<UartTargetProvider>(static_cast<int>(ammo.nTargets));

  const float firstTimeSec = static_cast<float>(firstTele.t_ms) / 1000.0f;
  for (size_t i = 0; i < initialTargets.size(); i++) {
    targetProvider->update(static_cast<int>(i), initialTargets[i], firstTimeSec);
  }

  auto solver = std::make_unique<TableSolver>(BALLISTIC_TABLE_FILE_PATH, bombParams, droneConfig);

  if (!solver->isLoadSuccess()) {
    std::cerr << "ballistic table load failed: " << BALLISTIC_TABLE_FILE_PATH << " (запускати з теки домашки)\n";
    return 1;
  }

  MissionProcessor mission(targetProvider, nullptr, std::move(solver));

  if (!mission.init(droneConfig)) {
    return 1;
  }

  const DroneController controller(droneConfig);

  float lastTimeSec = firstTimeSec;
  int teleCount = 0;
  bool flying = true;

  while (flying) {
    const int n = link.readBytes(buf, sizeof(buf));
    if (n <= 0) {
      usleep(1000);
      continue;
    }

    for (int i = 0; i < n; i++) {
      if (!parser.feed(buf[i], type, payload, len)) {
        continue;
      }

      if (type == dlink::PKT_TARGET) {
        dlink::TargetPos tp{};
        std::memcpy(&tp, payload, sizeof tp);
        targetProvider->update(tp.id, {tp.x, tp.y}, lastTimeSec);
      }
      else if (type == dlink::PKT_TELEMETRY) {
        dlink::Telemetry t{};
        std::memcpy(&t, payload, sizeof t);

        const DroneTelemetry tele = toDroneTelemetry(t);
        lastTimeSec = tele.timeSinceStart;

        const SimStep stepResult = mission.step(tele);
        const ControlSignal control = controller.compute(mission.getLastCommand(), tele);
        link.sendControl(control.accel, control.turnRate);

        if (++teleCount % 10 == 0) {
          std::cout << "t=" << tele.timeSinceStart << "s pos=(" << tele.pos.x << "," << tele.pos.y << ") v=" << tele.speed
                    << " state=" << stepResult.state << " target=" << stepResult.targetIdx
                    << " distToDrop=" << length(tele.pos - stepResult.dropPoint) << "\n";
        }

        if (!mission.hasNext()) {
          std::cout << "DROP! t=" << tele.timeSinceStart << "s pos=(" << tele.pos.x << "," << tele.pos.y
                    << ") target=" << stepResult.targetIdx << "\n";
          gpio.pulseDrop();
          flying = false;
          break;
        }
      }
    }
  }

  std::cout << "mission finished - verdict (HIT/MISS) у терміналі чекера\n";

  return 0;
}
// NOLINTEND(cppcoreguidelines-pro-bounds-pointer-arithmetic)
// NOLINTEND(cppcoreguidelines-pro-bounds-array-to-pointer-decay,cppcoreguidelines-avoid-c-arrays)
