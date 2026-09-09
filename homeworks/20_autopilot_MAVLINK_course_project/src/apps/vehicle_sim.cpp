#include <iostream>
#include <string>
#include <vector>

#include "Logger.hpp"

// NOLINTBEGIN(cppcoreguidelines-pro-bounds-pointer-arithmetic)

struct CliOptions {
  std::string scenario = "data";
  std::string apEndpoint = "udp://127.0.0.1:14555";
  float timeScale = 1.0f;
};

CliOptions parseArgs(const std::vector<std::string>& args)
{
  CliOptions opts;

  for (size_t i = 0; i + 1 < args.size(); i += 2) {
    const std::string& key = args[i];
    const std::string& value = args[i + 1];

    if (key == "--scenario") {
      opts.scenario = value;
    }
    else if (key == "--ap-endpoint") {
      opts.apEndpoint = value;
    }
    else if (key == "--time-scale") {
      opts.timeScale = std::stof(value);
    }
    else {
      std::cerr << "Unknown argument at vehicle_sim.cpp: " << key << '\n';
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

  LOG("vehicle_sim:\n"
      << "  scenario    = " << opts.scenario << '\n'
      << "  ap-endpoint = " << opts.apEndpoint << '\n'
      << "  time-scale  = " << opts.timeScale);
  return 0;
}

// NOLINTEND(cppcoreguidelines-pro-bounds-pointer-arithmetic)
