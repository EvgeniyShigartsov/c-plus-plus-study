#pragma once
#include <string>

struct gpiod_chip;
struct gpiod_line;

// NOLINTNEXTLINE(cppcoreguidelines-special-member-functions)
class GpioSignals {
public:
  GpioSignals(const std::string& chipName, int startLine, int dropLine);
  ~GpioSignals();

  bool isOpen() const;

  void assertStart();

  void pulseDrop();

private:
  gpiod_chip* chip = nullptr;
  gpiod_line* startLine = nullptr;
  gpiod_line* dropLine = nullptr;
  bool dropDone = false;
};
