#include "config/ComponentFactory.hpp"
#include <memory>
#include <string>
#include "config/FileConfigLoader.hpp"
#include "interfaces/IBallisticSolver.hpp"
#include "interfaces/ITargetProvider.hpp"
#include "solvers/TableSolver.hpp"
#include "types.hpp"
#include "providers/JsonTargetProvider.hpp"
#include "solvers/AnalyticalSolver.hpp"

std::unique_ptr<IConfigLoader> createLoader(LoaderType type)
{
  switch (type) {
    case LoaderType::FILE:
      return std::make_unique<FileConfigLoader>();
    default:
      return nullptr;
  }
}

std::unique_ptr<IBallisticSolver> createSolver(SolverType type,
                                               const std::string& tableFilePath,
                                               const BombParams& bomb,
                                               const DroneConfig& droneConfig)
{
  switch (type) {
    case SolverType::ANALYTICAL:
      return std::make_unique<AnalyticalSolver>();
    case SolverType::TABLE:
      return std::make_unique<TableSolver>(tableFilePath, bomb, droneConfig);
    default:
      return nullptr;
  }
}

std::shared_ptr<ITargetProvider> createProvider(ProviderType type, const std::string& pathToConfig, const DroneConfig& droneConfig)
{
  switch (type) {
    case ProviderType::JSON:
      return std::make_shared<JsonTargetProvider>(pathToConfig, droneConfig);
    default:
      return nullptr;
  }
}
