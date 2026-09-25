#pragma once
#include "esp_adc/adc_oneshot.h"
#include "core/Reading.hpp"

// NOLINTNEXTLINE(cppcoreguidelines-special-member-functions)
class EspAdcReader {
public:
  explicit EspAdcReader(const adc_channel_t channel);
  ~EspAdcReader();

  [[nodiscard]] Reading read() const;

private:
  adc_channel_t channel;
  adc_oneshot_unit_handle_t unitHandle = nullptr;
};
