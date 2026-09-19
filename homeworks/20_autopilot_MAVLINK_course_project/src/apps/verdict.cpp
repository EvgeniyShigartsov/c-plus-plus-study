#include <iostream>
#include <string>
#include <vector>

#include "Logger.hpp"

// NOLINTBEGIN(cppcoreguidelines-pro-bounds-pointer-arithmetic)

struct CliOptions {
  std::string endpoint = "udp://127.0.0.1:14550";
  std::string scenario = "data";
};

CliOptions parseArgs(const std::vector<std::string>& args)
{
  CliOptions opts;

  for (size_t i = 0; i + 1 < args.size(); i += 2) {
    const std::string& key = args[i];
    const std::string& value = args[i + 1];

    if (key == "--endpoint") {
      opts.endpoint = value;
    }
    else if (key == "--scenario") {
      opts.scenario = value;
    }
    else {
      std::cerr << "Unknown argument at verdict.cpp: " << key << '\n';
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

  LOG("verdict:\n"
      << "  endpoint = " << opts.endpoint << '\n'
      << "  scenario = " << opts.scenario);
  return 0;
}

// NOLINTEND(cppcoreguidelines-pro-bounds-pointer-arithmetic)
