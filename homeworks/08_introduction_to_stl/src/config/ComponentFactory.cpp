#include "config/ComponentFactory.hpp"
#include <string>
#include "config/FileConfigLoader.hpp"
#include "types.hpp"
#include "providers/JsonTargetProvider.hpp"
#include "solvers/AnalyticalSolver.hpp"
#include "Logger.hpp"

// NOLINTBEGIN(cppcoreguidelines-owning-memory)

IConfigLoader* createLoader(LoaderType type)
{
  switch (type) {
    case LoaderType::FILE:
      return new FileConfigLoader;
    default:
      return nullptr;
  }
}

IBallisticSolver* createSolver(SolverType type, const BombParams& bp, const DroneConfig& dc)
{
  switch (type) {
    case SolverType::ANALYTICAL: {
      auto* solver = new AnalyticalSolver{bp, dc};

      if (solver->isLoadSuccess()) {
        LOG("AnalyticalSolver load error.");
        return solver;
      }
      return nullptr;
    }
    default:
      return nullptr;
  }
}

ITargetProvider* createProvider(ProviderType type, const std::string& pathToConfig, const DroneConfig& droneConfig)
{
  switch (type) {
    case ProviderType::JSON:
      return new JsonTargetProvider{pathToConfig, droneConfig};
    default:
      return nullptr;
  }
}

// NOLINTEND(cppcoreguidelines-owning-memory)