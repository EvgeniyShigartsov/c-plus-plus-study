#pragma once
#include <vector>
#include "interfaces/ITargetProvider.hpp"
#include "types.hpp"

class CachedTargetProvider : public ITargetProvider {
public:
  CachedTargetProvider(int targetCount);

  void update(int targetIndex, Coord pos, float timeSec);

  int getTargetCount() const override;
  Target getTarget(const int targetIndex) const override;

private:
  std::vector<TargetSlot> slots;
};
