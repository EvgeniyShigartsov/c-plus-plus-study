#include "hal/adc_types.h"

#include "EspAdcReader.hpp"

namespace {

constexpr adc_unit_t kAdcUnit = ADC_UNIT_1;      // який ацп блок використовувати
constexpr adc_atten_t kAtten = ADC_ATTEN_DB_12;  // наскільки стиснути вхідну напругу
constexpr adc_bitwidth_t kBitwidth = ADC_BITWIDTH_DEFAULT;  // роздільна здатність, дати драйверу обрати самостійно

}  // namespace

EspAdcReader::EspAdcReader(const adc_channel_t channel)
  : channel(channel)
{
  // Відкриття самого блоку АЦП
  adc_oneshot_unit_init_cfg_t unitConfig{};
  unitConfig.unit_id = kAdcUnit;
  adc_oneshot_new_unit(&unitConfig, &unitHandle);  // Створити драйвер

  // Налаштування каналу (GP піна)
  const adc_oneshot_chan_cfg_t channelConfig{.atten = kAtten, .bitwidth = kBitwidth};
  adc_oneshot_config_channel(unitHandle, channel, &channelConfig);
}

EspAdcReader::~EspAdcReader()
{
  adc_oneshot_del_unit(unitHandle);  // Звільнити драйвер
}

Reading EspAdcReader::read() const
{
  int raw = 0;
  adc_oneshot_read(unitHandle, channel, &raw);

  return {.value = static_cast<float>(raw)};
}
