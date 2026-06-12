#include "solvers/TableSolver.hpp"
#include "MathUtils.hpp"
#include "types.hpp"

TableSolver::TableSolver(const std::string& tablePath, BombParams bomb, DroneConfig droneConfig)
  : ammo(std::move(bomb))
  , droneConfig(std::move(droneConfig))
{
  table.load(tablePath);
}

Coord TableSolver::solve(const Coord targetCoord, const Coord droneCoord, const float /* hDist */)
{
  const auto result = table.lookup(droneConfig.altitude, droneConfig.v0, ammo.mass, ammo.drag, ammo.lift);

  const Coord delta = targetCoord - droneCoord;

  return targetCoord - normalizeCoord(delta) * result.hDist;
}

TableSolver::~TableSolver() = default;