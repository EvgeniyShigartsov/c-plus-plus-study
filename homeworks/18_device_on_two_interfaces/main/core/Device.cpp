#include "Device.hpp"

#include <cstdio>

#include "Command.hpp"
#include "StatusLine.hpp"

namespace {

constexpr uint32_t kMinPeriodMs = 100;
constexpr uint32_t kMaxPeriodMs = 1000;

}  // namespace

Device::Device(const uint32_t periodMs)
  : periodMsValue(periodMs)
{
}

bool Device::onTick(char* buf, const size_t capacity, const uint32_t tMs, const Reading& reading) const
{
  return StatusLine::format(buf, capacity, tMs, reading);
}

bool Device::onCommand(const std::string_view line, char* buf, const size_t capacity)
{
  const Command command = parseCommand(line);

  if (command.kind == Command::Kind::SetPeriod && command.periodMs >= kMinPeriodMs && command.periodMs <= kMaxPeriodMs) {
    periodMsValue = command.periodMs;
    const int written = std::snprintf(buf, capacity, "ok: period=%u ms", static_cast<unsigned>(command.periodMs));
    return written > 0 && static_cast<size_t>(written) < capacity;
  }

  if (command.kind == Command::Kind::SetPeriod) {
    const int written = std::snprintf(
      buf, capacity, "err: period out of range (%u-%u ms)", static_cast<unsigned>(kMinPeriodMs), static_cast<unsigned>(kMaxPeriodMs));
    return written > 0 && static_cast<size_t>(written) < capacity;
  }

  const int written = std::snprintf(buf, capacity, "err: unknown command");
  return written > 0 && static_cast<size_t>(written) < capacity;
}
