#pragma once
#include "types.hpp"
#include "interfaces/IBallisticSolver.hpp"
#include "BallisticTable.hpp"

// NOLINTNEXTLINE(cppcoreguidelines-special-member-functions)
class TableSolver : public IBallisticSolver {
private:
  BallisticTable table;
  BombParams ammo;
  DroneConfig droneConfig;
  bool isLoadedSuccesful = false;
  float bombFlightTime = 0.0f;
  float h = 0.0f;

public:
  TableSolver(const std::string& tablePath, BombParams bomb, DroneConfig droneConfig);

  Coord solve(const Coord targetCoord, const Coord droneCoord, const float hDist) override;
  float getBombFlightTime() override;
  float get_h() override;
  bool isLoadSuccess() override;
  [[nodiscard]] bool isLoadedSucces() const;
  virtual ~TableSolver();
};