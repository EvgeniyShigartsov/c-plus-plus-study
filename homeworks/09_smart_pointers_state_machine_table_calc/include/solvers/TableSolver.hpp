#pragma once
#include "types.hpp"
#include "interfaces/IBallisticSolver.hpp"
#include "BallisticTable.hpp"

// NOLINTNEXTLINE(cppcoreguidelines-special-member-functions)
class TableSolver : public IBallisticSolver {
  BallisticTable table;
  BombParams ammo;
  float v0;
  float altitude;

public:
  TableSolver(const char* tablePath, BombParams& bomb, const float v0, const float alt);

  Coord solve(const Coord targetCoord, const Coord droneCoord, const float hDist) override;
  virtual ~TableSolver();
};