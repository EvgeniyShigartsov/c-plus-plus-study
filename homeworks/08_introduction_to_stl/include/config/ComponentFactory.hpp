#pragma once
#include <string>
#include "types.hpp"

class IConfigLoader;
class IBallisticSolver;
class ITargetProvider;

enum class SolverType { ANALYTICAL };
enum class ProviderType { JSON };
enum class LoaderType { FILE };

IConfigLoader* createLoader(LoaderType type);
IBallisticSolver* createSolver(SolverType type, const BombParams& bp, const DroneConfig& dc);
ITargetProvider* createProvider(ProviderType type, const std::string& pathToConfig, const DroneConfig& droneConfig);