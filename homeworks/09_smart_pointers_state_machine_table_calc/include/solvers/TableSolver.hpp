#pragma once
#include "types.hpp"
#include "interfaces/IBallisticSolver.hpp"
#include "BallisticTable.hpp"

// NOLINTNEXTLINE(cppcoreguidelines-special-member-functions)
class TableSolver : public IBallisticSolver {
  BallisticTable table;
  BombParams ammo;
  DroneConfig droneConfig;
  bool isLoadedSuccesful = false;

public:
  TableSolver(const std::string& tablePath, BombParams bomb, DroneConfig droneConfig);

  Coord solve(const Coord targetCoord, const Coord droneCoord, const float hDist) override;
  float getBombFlightTime() override { return 0.0f; }
  float get_h() override { return 0.0f; }
  bool isLoadSuccess() override { return isLoadedSuccesful; }
  [[nodiscard]] bool isLoadedSucces() const;
  virtual ~TableSolver();
};