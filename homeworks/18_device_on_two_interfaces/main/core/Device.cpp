#include "Device.hpp"

#include <cstdio>

#include "Command.hpp"
#include "StatusLine.hpp"

bool Device::onTick(char* buf, const size_t capacity, const uint32_t tMs, const Reading& reading) const
{
  return StatusLine::format(buf, capacity, tMs, reading);
}

bool Device::onCommand(const std::string_view line, char* buf, const size_t capacity)
{
  const Command command = parseCommand(line);

  if (command.kind == Command::Kind::SetPeriod) {
    periodMsValue = command.periodMs;
    const int written = std::snprintf(buf, capacity, "ok: period=%u ms", static_cast<unsigned>(command.periodMs));
    return written > 0 && static_cast<size_t>(written) < capacity;
  }

  const int written = std::snprintf(buf, capacity, "err: unknown command");
  return written > 0 && static_cast<size_t>(written) < capacity;
}
