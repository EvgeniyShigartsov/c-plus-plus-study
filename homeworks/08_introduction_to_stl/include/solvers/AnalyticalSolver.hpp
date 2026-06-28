#pragma once
#include "types.hpp"
#include "interfaces/IBallisticSolver.hpp"

// NOLINTNEXTLINE(cppcoreguidelines-special-member-functions)
class AnalyticalSolver : public IBallisticSolver {
public:
  AnalyticalSolver(const BombParams& bp, const DroneConfig& dc);
  Coord solve(const Coord targetCoord, const Coord droneCoord, const float hDist) override;
  float getBombFlightTime() override;
  float get_h() override;
  bool isLoadSuccess() override;
  virtual ~AnalyticalSolver();

private:
  bool loadSuccesfully = false;
  float bombFlightTime = 0.0f;
  float h = 0.0f;

  bool calcBombFlightTime(const float d, const float g, const float m, const float l, const float v0, const float zd);
  float calc_h(const float d, const float g, const float l, const float m, const float v0) const;
};