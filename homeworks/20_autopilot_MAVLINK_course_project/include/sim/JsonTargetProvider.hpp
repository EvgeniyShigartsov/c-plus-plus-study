#pragma once
#include "types.hpp"
#include <vector>

// NOLINTNEXTLINE(cppcoreguidelines-special-member-functions)
class JsonTargetProvider {
private:
  bool isSuccesFullyLoaded = false;
  std::vector<std::vector<Coord>> targetsInTime;

  int TARGETS_COUNT = 0;
  int TARGET_MOVES_COUNT = 0;
  float arrayTimeStep = 0.0f;
  float simTimeStep = 0.0f;

public:
  JsonTargetProvider(const std::string& pathToConfig, float arrayTimeStep, float simTimeStep);

  Target getTarget(const float simCurrentTime, const int targetIndex) const;
  int getTargetCount() const;
  bool isLoadSucces() const;

  virtual ~JsonTargetProvider();
};
