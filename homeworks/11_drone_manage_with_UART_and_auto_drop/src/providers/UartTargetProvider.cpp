#include "providers/UartTargetProvider.hpp"
#include "Logger.hpp"

UartTargetProvider::UartTargetProvider(int targetCount)

{
  slots.resize(targetCount);
}

void UartTargetProvider::update(int targetIndex, Coord pos, float timeSec)
{
  if (targetIndex < 0 || targetIndex >= getTargetCount()) {
    LOG("UartTargetProvider::update - invalid targetIndex: " << targetIndex);
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

int UartTargetProvider::getTargetCount() const
{
  return static_cast<int>(slots.size());
}

Target UartTargetProvider::getTarget(const int targetIndex) const
{
  return slots[targetIndex].target;
}

bool UartTargetProvider::isLoadSucces() const
{
  return true;
}

// Тимчасові заглушки: TODO: оновити ITargetProvider щоб ци методы не були більше потрібні.
bool UartTargetProvider::isThreadReady() const
{
  return true;
}

void UartTargetProvider::start() {}

void UartTargetProvider::stop() {}

void UartTargetProvider::run() {}
