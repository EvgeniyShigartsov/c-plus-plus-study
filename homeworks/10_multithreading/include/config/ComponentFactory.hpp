#pragma once
#include <memory>
#include "types.hpp"

class IConfigLoader;
class IBallisticSolver;
class ITargetProvider;

enum class SolverType { ANALYTICAL, TABLE };
enum class ProviderType { JSON };
enum class LoaderType { FILE };

std::unique_ptr<IConfigLoader> createLoader(LoaderType type);
std::unique_ptr<IBallisticSolver> createSolver(SolverType type,
                                               const std::string& tableFilePath,
                                               const BombParams& bomb,
                                               const DroneConfig& droneConfig);
std::shared_ptr<ITargetProvider> createProvider(ProviderType type, const std::string& pathToConfig, const DroneConfig& droneConfig);