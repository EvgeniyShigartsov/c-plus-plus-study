#include "solvers/TableSolver.hpp"
#include "MathUtils.hpp"
#include "types.hpp"

TableSolver::TableSolver(const char* tablePath, BombParams& bomb, const float v0, const float alt)
  : ammo(bomb)
  , v0(v0)
  , altitude(alt)
{
  table.load(tablePath);
}

Coord TableSolver::solve(const Coord targetCoord, const Coord droneCoord, const float /* hDist */)
{
  const auto result = table.lookup(altitude, v0, ammo.mass, ammo.drag, ammo.lift);

  const Coord delta = targetCoord - droneCoord;

  return targetCoord - normalizeCoord(delta) * result.hDist;
}

TableSolver::~TableSolver() = default;