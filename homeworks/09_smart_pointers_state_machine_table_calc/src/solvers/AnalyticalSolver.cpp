#include "types.hpp"
#include "solvers/AnalyticalSolver.hpp"
#include "MathUtils.hpp"
#include "Logger.hpp"

AnalyticalSolver::AnalyticalSolver(const BombParams& bp, const DroneConfig& dc)
{
  const float GRAVITY = 9.81f;

  const bool isBombOk = calcBombFlightTime(bp.drag, GRAVITY, bp.mass, bp.lift, dc.v0, dc.altitude);
  h = calc_h(bp.drag, GRAVITY, bp.lift, bp.mass, dc.v0);

  loadSuccesfully = isBombOk;
}

Coord AnalyticalSolver::solve(const Coord targetCoord, const Coord droneCoord, const float hDist)
{
  const Coord delta = targetCoord - droneCoord;
  return targetCoord - normalizeCoord(delta) * hDist;
}

float AnalyticalSolver::getBombFlightTime()
{
  return bombFlightTime;
}

float AnalyticalSolver::get_h()
{
  return h;
}

bool AnalyticalSolver::isLoadSuccess()
{
  return loadSuccesfully;
}

bool AnalyticalSolver::calcBombFlightTime(const float d, const float g, const float m, const float l, const float v0, const float zd)
{
  float a = (d * g * m) - ((powf(d, 2) * 2) * l * v0);
  float b = ((-3 * g) * (powf(m, 2))) + ((d * 3) * l * m * v0);
  float c = (6 * powf(m, 2)) * zd;

  float p = -powf(b, 2) / (3 * powf(a, 2));
  float q = (2 * powf(b, 3)) / (27 * powf(a, 3)) + c / a;

  float angCos = 3 * q / (2 * p) * sqrtf(-3 / p);

  if (angCos > 1.0f || angCos < -1.0f) {
    LOG("arccos is out -1...1, value is: " << angCos);
    return false;
  }

  float fi = acosf(angCos);

  bombFlightTime = 2 * sqrtf(-p / 3) * cosf((fi + static_cast<float>(M_PI) * 4) / 3) - b / (3 * a);

  return true;
}

// NOLINTNEXTLINE(readability-convert-member-functions-to-static)
float AnalyticalSolver::calc_h(const float d, const float g, const float l, const float m, const float v0) const
{
  const float t = bombFlightTime;

  const float l2 = powf(l, 2);
  const float l2p1 = l2 + 1;

  float term1 = (powf(t, 3) * (6 * d * g * l * m - 6 * powf(d, 2) * (l2 - 1) * v0)) / (36 * powf(m, 2));

  float term2 = (powf(t, 5) * (3 * powf(d, 3) * g * powf(l, 3) * m - 3 * powf(d, 4) * l2 * l2p1 * v0)) / (36 * l2p1 * powf(m, 4));

  float term3 = (powf(t, 4) * (3 * powf(d, 3) * (l2p1)*l2 * v0 + 6 * powf(d, 3) * l2p1 * powf(l, 4) * v0 -
                               6 * powf(d, 2) * g * (powf(l, 4) + l2p1) * l * m)) /
                (36 * powf(l2p1, 2) * powf(m, 3));

  float term4 = (d * powf(t, 2) * v0) / (m * 2);

  return term1 + term2 + term3 - term4 + (t * v0);
}

AnalyticalSolver::~AnalyticalSolver() = default;