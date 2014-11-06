
#ifndef __MAVSTATION_FIRMWARE_ADC_H__
#define __MAVSTATION_FIRMWARE_ADC_H__

#define ADC_CHANNEL_VBATT       4
#define ADC_CHANNEL_IN5         5
#define ADC_CHANNEL_COUNT       2

int adc_init(void);
uint16_t adc_measure(uint8_t channel);

#endif // __MAVSTATION_FIRMWARE_ADC_H__

