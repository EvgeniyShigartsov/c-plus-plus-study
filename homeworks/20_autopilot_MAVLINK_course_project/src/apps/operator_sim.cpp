#include <iostream>
#include <string>
#include <vector>

#include "Logger.hpp"

// NOLINTBEGIN(cppcoreguidelines-pro-bounds-pointer-arithmetic)

struct CliOptions {
  std::string script;
  std::string targets = "data/targets.json";
  std::string apEndpoint = "udp://127.0.0.1:14550";
};

CliOptions parseArgs(const std::vector<std::string>& args)
{
  CliOptions opts;

  for (size_t i = 0; i + 1 < args.size(); i += 2) {
    const std::string& key = args[i];
    const std::string& value = args[i + 1];

    if (key == "--script") {
      opts.script = value;
    }
    else if (key == "--targets") {
      opts.targets = value;
    }
    else if (key == "--ap-endpoint") {
      opts.apEndpoint = value;
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
      << "  script      = " << (opts.script.empty() ? "(no script)" : opts.script) << '\n'
      << "  targets     = " << opts.targets << '\n'
      << "  ap-endpoint = " << opts.apEndpoint);
  return 0;
}

// NOLINTEND(cppcoreguidelines-pro-bounds-pointer-arithmetic)
