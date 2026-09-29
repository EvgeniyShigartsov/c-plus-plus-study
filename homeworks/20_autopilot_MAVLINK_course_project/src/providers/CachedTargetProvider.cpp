#include "providers/CachedTargetProvider.hpp"
#include "Logger.hpp"

CachedTargetProvider::CachedTargetProvider(int targetCount)
{
  slots.resize(targetCount);
}

void CachedTargetProvider::update(int targetIndex, Coord pos, float timeSec)
{
  if (targetIndex < 0 || targetIndex >= getTargetCount()) {
    LOG("CachedTargetProvider::update - invalid targetIndex: " << targetIndex);
    return;
  }

  TargetSlot& slot = slots[targetIndex];

  const float dt = timeSec - slot.lastUpdateTime;

  if (slot.seen && dt > 0.0f) {
    slot.target.velocity = (pos - slot.target.pos) / dt;
  }

  slot.target.pos = pos;
  slot.lastUpdateTime = timeSec;
  slot.seen = true;
}

int CachedTargetProvider::getTargetCount() const
{
  return static_cast<int>(slots.size());
}

Target CachedTargetProvider::getTarget(const int targetIndex) const
{
  return slots[targetIndex].target;
}
