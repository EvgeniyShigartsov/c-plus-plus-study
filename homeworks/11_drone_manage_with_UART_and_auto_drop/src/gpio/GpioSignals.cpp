#include "gpio/GpioSignals.hpp"

#include <gpiod.h>
#include <unistd.h>

#include <cstdio>

GpioSignals::GpioSignals(const std::string& chipName, int startLineNum, int dropLineNum)
  // chipName друкує чекер на старті ("gpiochip1" у sim / "gpiochip0" на платі)
  : chip(gpiod_chip_open_by_name(chipName.c_str()))
{
  if (chip == nullptr) {
    perror("gpiod_chip_open_by_name");
    return;
  }

  startLine = gpiod_chip_get_line(chip, static_cast<unsigned int>(startLineNum));
  dropLine = gpiod_chip_get_line(chip, static_cast<unsigned int>(dropLineNum));

  if (startLine == nullptr || dropLine == nullptr) {
    perror("gpiod_chip_get_line");
    gpiod_chip_close(chip);
    chip = nullptr;
    return;
  }

  if (gpiod_line_request_output(startLine, "drone", 0) < 0 || gpiod_line_request_output(dropLine, "drone", 0) < 0) {
    perror("gpiod_line_request_output");
    gpiod_chip_close(chip);
    chip = nullptr;
  }
}

GpioSignals::~GpioSignals()
{
  if (chip != nullptr) {
    gpiod_chip_close(chip);
  }
}

bool GpioSignals::isOpen() const
{
  return chip != nullptr;
}

void GpioSignals::assertStart()
{
  gpiod_line_set_value(startLine, 1);  // на старті: «готовий» -> чекер починає симуляцію
}

void GpioSignals::pulseDrop()
{
  if (dropDone) {
    return;  // скид одноразовий
  }

  dropDone = true;

  gpiod_line_set_value(dropLine, 1);  // імпульс скиду
  usleep(80000);
  gpiod_line_set_value(dropLine, 0);
}
