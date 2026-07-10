#include <iostream>
#include <string>
#include <vector>
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

int main(int argc, char* argv[])
{
  // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-pointer-arithmetic)
  const std::vector<std::string> args(argv + 1, argv + argc);
  const CliOptions opts = parseArgs(args);

  std::cout << "hm11 uart autopilot (skeleton)\n"
            << "  uart:       " << opts.uartDev << "\n"
            << "  gpiochip:   " << opts.gpioChip << "\n"
            << "  start-line: " << opts.startLine << "\n"
            << "  drop-line:  " << opts.dropLine << "\n";

  return 0;
}
