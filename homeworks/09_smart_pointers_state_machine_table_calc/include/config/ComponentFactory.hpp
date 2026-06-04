#pragma once
#include <memory>
#include "interfaces/IConfigLoader.hpp"
#include "interfaces/IBallisticSolver.hpp"
#include "interfaces/ITargetProvider.hpp"

enum class SolverType { ANALYTICAL };
enum class ProviderType { JSON };
enum class LoaderType { FILE };

std::unique_ptr<IConfigLoader> createLoader(LoaderType type);
std::unique_ptr<IBallisticSolver> createSolver(SolverType type);
std::shared_ptr<ITargetProvider> createProvider(ProviderType type, const std::string& pathToConfig, const DroneConfig& droneConfig);