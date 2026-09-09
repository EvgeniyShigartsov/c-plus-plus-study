// autopilot — модуль-автопілот
#include <iostream>
#include <string>
#include <vector>

#include "Logger.hpp"

// NOLINTBEGIN(cppcoreguidelines-pro-bounds-pointer-arithmetic)

struct CliOptions {
  std::string gcsEndpoint = "udp://127.0.0.1:14550";
  std::string vehicleEndpoint = "udp://127.0.0.1:14555";
  std::string ballisticTable = "data/ballistic_table.txt";
};

CliOptions parseArgs(const std::vector<std::string>& args)
{
  CliOptions opts;

  for (size_t i = 0; i + 1 < args.size(); i += 2) {
    const std::string& key = args[i];
    const std::string& value = args[i + 1];

    if (key == "--gcs-endpoint") {
      opts.gcsEndpoint = value;
    }
    else if (key == "--vehicle-endpoint") {
      opts.vehicleEndpoint = value;
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
      << "  gcs-endpoint     = " << opts.gcsEndpoint << '\n'
      << "  vehicle-endpoint = " << opts.vehicleEndpoint << '\n'
      << "  ballistic-table  = " << opts.ballisticTable);
  return 0;
}

// NOLINTEND(cppcoreguidelines-pro-bounds-pointer-arithmetic)
