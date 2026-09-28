#include "Command.hpp"

#include <charconv>

namespace {

constexpr std::string_view kSetPeriodPrefix = "p ";

}  // namespace

Command parseCommand(const std::string_view line)
{
  if (line.size() > kSetPeriodPrefix.size() && line.substr(0, kSetPeriodPrefix.size()) == kSetPeriodPrefix) {
    const std::string_view arg = line.substr(kSetPeriodPrefix.size());
    uint32_t periodMs = 0;

    const std::from_chars_result result = std::from_chars(arg.data(), arg.data() + arg.size(), periodMs);

    if (result.ec == std::errc{} && result.ptr == arg.end()) {
      return {.kind = Command::Kind::SetPeriod, .periodMs = periodMs};
    }
  }

  return {.kind = Command::Kind::Unknown};
}
