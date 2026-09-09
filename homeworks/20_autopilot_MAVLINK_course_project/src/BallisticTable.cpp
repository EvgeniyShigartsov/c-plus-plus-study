#include <string>
#include <vector>
#include <fstream>
#include "BallisticTable.hpp"

using Result = BallisticTable::Result;

// Індекс у плоскому масиві: [iZ0][iV0][iM][iD][iL]
[[nodiscard]] size_t BallisticTable::index(int iz, int iv, int im, int id, int il) const
{
  return ((((size_t)iz * axisV0.size() + iv) * axisM.size() + im) * axisD.size() + id) * axisL.size() + il;
}

[[nodiscard]] const Result& BallisticTable::at(int iz, int iv, int im, int id, int il) const
{
  return data[index(iz, iv, im, id, il)];
}

[[nodiscard]] Result BallisticTable::lookup(float Z0, float V0, float m, float d, float l) const
{
  Interp iz = findInterp(Z0, axisZ0);
  Interp iv = findInterp(V0, axisV0);
  Interp im = findInterp(m, axisM);
  Interp id = findInterp(d, axisD);
  Interp il = findInterp(l, axisL);

  // 2^5 = 32 вершини гіперкуба
  // Згортаємо: 32 → 16 → 8 → 4 → 2 → 1

  //  l: 32 → 16
  // NOLINTBEGIN(readability-braces-around-statements,modernize-avoid-c-arrays,cppcoreguidelines-avoid-c-arrays)
  Result v[16];
  for (int a = 0; a < 2; a++)
    for (int b = 0; b < 2; b++)
      for (int c = 0; c < 2; c++)
        for (int e = 0; e < 2; e++) {
          const auto& lo = at(iz.lo + a, iv.lo + b, im.lo + c, id.lo + e, il.lo);
          const auto& hi = at(iz.lo + a, iv.lo + b, im.lo + c, id.lo + e, il.lo + 1);
          v[a * 8 + b * 4 + c * 2 + e] = lerp(lo, hi, il.frac);
        }

  // d: 16 → 8
  Result w[8];
  for (int a = 0; a < 2; a++)
    for (int b = 0; b < 2; b++)
      for (int c = 0; c < 2; c++)
        w[a * 4 + b * 2 + c] = lerp(v[a * 8 + b * 4 + c * 2], v[a * 8 + b * 4 + c * 2 + 1], id.frac);

  // m: 8 → 4
  Result u[4];
  for (int a = 0; a < 2; a++)
    for (int b = 0; b < 2; b++)
      u[a * 2 + b] = lerp(w[a * 4 + b * 2], w[a * 4 + b * 2 + 1], im.frac);

  // V0: 4 → 2
  Result s[2];
  for (int a = 0; a < 2; a++)
    // NOLINTNEXTLINE(bugprone-implicit-widening-of-multiplication-result)
    s[a] = lerp(u[a * 2], u[a * 2 + 1], iv.frac);

  // Z0: 2 → 1
  return lerp(s[0], s[1], iz.frac);

  // NOLINTEND(readability-braces-around-statements,modernize-avoid-c-arrays,cppcoreguidelines-avoid-c-arrays)
}

// Завантаження з текстового файлу
bool BallisticTable::load(const std::string& path)
{
  std::ifstream file(path);
  if (!file.is_open()) {
    return false;
  }

  // NOLINTNEXTLINE(readability-isolate-declaration,cppcoreguidelines-init-variables)
  int nZ, nV, nM, nD, nL;
  file >> nZ >> nV >> nM >> nD >> nL;

  axisZ0.resize(nZ);
  for (auto& value : axisZ0) {
    file >> value;
  }
  axisV0.resize(nV);
  for (auto& value : axisV0) {
    file >> value;
  }
  axisM.resize(nM);
  for (auto& value : axisM) {
    file >> value;
  }
  axisD.resize(nD);
  for (auto& value : axisD) {
    file >> value;
  }
  axisL.resize(nL);
  for (auto& value : axisL) {
    file >> value;
  }

  size_t total = (size_t)nZ * nV * nM * nD * nL;
  data.resize(total);

  // Порядок: Z0 → V0 → m → d → l (зовнішній → внутрішній)
  for (size_t i = 0; i < total; i++) {
    file >> data[i].t >> data[i].hDist;
  }

  return file.good();
}

// Лінійна інтерполяція для Result (обидва поля паралельно)
Result BallisticTable::lerp(const Result& a, const Result& b, float t)
{
  return {a.t + (b.t - a.t) * t, a.hDist + (b.hDist - a.hDist) * t};
}

BallisticTable::Interp BallisticTable::findInterp(float val, const std::vector<float>& axis)
{
  if (val <= axis.front()) {
    return {0, 0.0f};
  }
  if (val >= axis.back()) {
    return {(int)axis.size() - 2, 1.0f};
  }

  auto it = std::lower_bound(axis.begin(), axis.end(), val);
  int i = (int)(it - axis.begin()) - 1;
  if (i < 0) {
    i = 0;
  }

  float frac = (val - axis[i]) / (axis[i + 1] - axis[i]);
  return {i, frac};
}