#pragma once
#include <vector>
#include "interfaces/ITargetProvider.hpp"
#include "types.hpp"

class UartTargetProvider : public ITargetProvider {
public:
  UartTargetProvider(int targetCount);

  void update(int targetIndex, Coord pos, float timeSec);

  int getTargetCount() const override;
  Target getTarget(const int targetIndex) const override;

private:
  struct TargetSlot {
    Target target = {{0.0f, 0.0f}, {0.0f, 0.0f}};
    float lastUpdateTime = 0.0f;
    bool seen = false;
  };

  std::vector<TargetSlot> slots;
};
