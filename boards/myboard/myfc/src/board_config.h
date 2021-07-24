/**
 * @file board_config.h
 *
 * myboardmyfc internal definitions
 */

#pragma once

/****************************************************************************************************
 * Included Files
 ****************************************************************************************************/

#include <px4_platform_common/px4_config.h>
#include <nuttx/compiler.h>
#include <stdint.h>

__BEGIN_DECLS

#ifndef __ASSEMBLY__

#include <px4_platform_common/board_common.h>

/* LEDs */
// LED1 - GPIO 25 - Green
#define GPIO_LED1       PX4_MAKE_GPIO_OUTPUT_CLEAR(25) // Take a look at rpi_common micro_hal.h
#define GPIO_LED_GREEN   GPIO_LED1

#define BOARD_OVERLOAD_LED     LED_GREEN

/*
 * ADC channels
 *
 * These are the channel numbers of the ADCs of the microcontroller that can be used by the Px4 Firmware in the adc driver
 */
#define ADC_CHANNELS (1 << 0) | (1 << 1) | (1 << 2) | (1 << 3)	// Change this later based on the adc channels actually used

// #define ADC_BATTERY_VOLTAGE_CHANNEL  12
// #define ADC_BATTERY_CURRENT_CHANNEL  11
// #define ADC_RC_RSSI_CHANNEL          0

/* High-resolution timer */
#define HRT_TIMER 1
#define HRT_TIMER_CHANNEL 1

#endif /* __ASSEMBLY__ */

__END_DECLS
