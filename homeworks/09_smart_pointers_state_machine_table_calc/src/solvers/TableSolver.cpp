#include "solvers/TableSolver.hpp"
#include "MathUtils.hpp"
#include "types.hpp"

TableSolver::TableSolver(const std::string& tablePath, BombParams bomb, DroneConfig droneConfig)
  : ammo(std::move(bomb))
  , droneConfig(std::move(droneConfig))
  , isLoadedSuccesful(table.load(tablePath))
{
  if (isLoadedSuccesful) {
    const auto result = table.lookup(droneConfig.altitude, droneConfig.v0, ammo.mass, ammo.drag, ammo.lift);
    bombFlightTime = result.t;
    h = result.hDist;
  }
}

Coord TableSolver::solve(const Coord targetCoord, const Coord droneCoord, const float /* hDist */)
{
  const Coord delta = targetCoord - droneCoord;

  return targetCoord - normalizeCoord(delta) * h;
}

float TableSolver::getBombFlightTime()
{
  return bombFlightTime;
}

float TableSolver::get_h()
{
  return h;
}

bool TableSolver::isLoadSuccess()
{
  return isLoadedSuccesful;
}

bool TableSolver::isLoadedSucces() const
{
  return isLoadedSuccesful;
}

TableSolver::~TableSolver() = default;