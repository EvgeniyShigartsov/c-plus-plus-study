#include <unistd.h>
#include <cstring>
#include <iostream>
#include <string>
#include <vector>
#include "link/UartLink.hpp"
#include "third_party/drone_link.h"

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

// NOLINTBEGIN(cppcoreguidelines-pro-bounds-array-to-pointer-decay,cppcoreguidelines-avoid-c-arrays)
// NOLINTBEGIN(cppcoreguidelines-pro-bounds-pointer-arithmetic)
void printFrame(uint8_t type, const uint8_t* payload, uint8_t len)
{
  using namespace dlink;

  switch (type) {
    case PKT_TELEMETRY: {
      Telemetry t{};
      std::memcpy(&t, payload, sizeof t);
      std::cout << "[TELEMETRY] t=" << t.t_ms << "ms pos=(" << t.x << "," << t.y << ") z=" << t.z << " v=(" << t.vx << "," << t.vy
                << ") speed=" << t.speed << " dir=" << t.dir << " state=" << static_cast<int>(t.state) << "\n";
      break;
    }
    case PKT_TARGET: {
      TargetPos tp{};
      std::memcpy(&tp, payload, sizeof tp);
      std::cout << "[TARGET] id=" << static_cast<int>(tp.id) << " pos=(" << tp.x << "," << tp.y << ")\n";
      break;
    }
    case PKT_AMMO: {
      AmmoCfg ammo{};
      std::memcpy(&ammo, payload, sizeof ammo);
      std::cout << "[AMMO] name=" << ammo.name << " mass=" << ammo.mass << " drag=" << ammo.drag << " lift=" << ammo.lift
                << " hitRadius=" << ammo.hitRadius << " nTargets=" << static_cast<int>(ammo.nTargets) << "\n";
      break;
    }
    case PKT_CONFIG: {
      DroneCfg cfg{};
      std::memcpy(&cfg, payload, sizeof cfg);
      std::cout << "[CONFIG] attackSpeed=" << cfg.attackSpeed << " accelPath=" << cfg.accelerationPath
                << " angularSpeed=" << cfg.angularSpeed << " turnThreshold=" << cfg.turnThreshold << " timeStep=" << cfg.timeStep
                << " timeScale=" << cfg.timeScale << "\n";
      break;
    }
    default:
      std::cout << "[UNKNOWN] type=0x" << std::hex << static_cast<int>(type) << std::dec << " len=" << static_cast<int>(len) << "\n";
      break;
  }
}

int main(int argc, char* argv[])
{
  const std::vector<std::string> args(argv + 1, argv + argc);
  const CliOptions opts = parseArgs(args);

  UartLink link(opts.uartDev);

  if (!link.isOpen()) {
    return 1;
  }

  std::cout << "hm11 uart autopilot: listening " << opts.uartDev << "\n";

  dlink::Parser parser;

  uint8_t buf[256];
  uint8_t type = 0;
  uint8_t len = 0;
  uint8_t payload[260];

  while (true) {
    const int n = link.readBytes(buf, sizeof(buf));
    if (n <= 0) {
      usleep(1000);
      continue;
    }

    for (int i = 0; i < n; i++) {
      if (parser.feed(buf[i], type, payload, len)) {  // зібрався цілий кадр
        printFrame(type, payload, len);
      }
    }
  }

  return 0;
}

// NOLINTEND(cppcoreguidelines-pro-bounds-pointer-arithmetic)
// NOLINTEND(cppcoreguidelines-pro-bounds-array-to-pointer-decay,cppcoreguidelines-avoid-c-arrays)