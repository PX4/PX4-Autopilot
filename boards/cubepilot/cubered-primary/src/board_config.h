/****************************************************************************
 *
 *   Copyright (c) 2024 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file board_config.h
 *
 * Board internal definitions
 */

#pragma once

#include <px4_platform_common/px4_config.h>
#include <nuttx/compiler.h>
#include <stdint.h>
#include <stm32_gpio.h>

/**
 * If NuttX is built without support for SMPS it can brick the hardware.
 * Therefore, we make sure the NuttX headers are correct.
 */
#include "hardware/stm32h7x3xx_pwr.h"
#if STM32_PWR_CR3_SMPSEXTHP != (1 << 3)
#  error "No SMPS support in NuttX submodule");
#endif


/* CubeRed bridge link to the secondary MCU (UART7) */
#define BOARD_USES_PX4IO_VERSION       2
#define CUBERED_BRIDGE_PRIMARY_DEVICE  "/dev/ttyS4"
#define CUBERED_BRIDGE_PRIMARY_BITRATE 1500000               /* 1.5 Mbps */


/* LEDs */
#define GPIO_nLED_AMBER        /* PG5 */  (GPIO_OUTPUT|GPIO_OPENDRAIN|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTG|GPIO_PIN5)

#define BOARD_HAS_CONTROL_STATUS_LEDS      1
#define BOARD_ARMED_LED  LED_AMBER


/* ADC channels */
#define PX4_ADC_GPIO  \
	/* PF11 */  GPIO_ADC1_INP2,  \
	/* PC0 */  GPIO_ADC123_INP10,  \
	/* PF13 */  GPIO_ADC2_INP2,  \
	/* PB1 */  GPIO_ADC12_INP5,  \
	/* PF12 */  GPIO_ADC1_INP6,   \
	/* PF3 */  GPIO_ADC3_INP5

/* Define Channel numbers must match above GPIO pins */
#define ADC_BATTERY1_VOLTAGE_CHANNEL        2 /* PF11: BATT_VOLTAGE_SENS */
#define ADC_BATTERY1_CURRENT_CHANNEL        10 /* PC0: BATT_CURRENT_SENS */
#define ADC_SCALED_V5_CHANNEL               2 /* PF13: VDD_5V_SENS */
#define ADC_BATTERY2_VOLTAGE_CHANNEL        5 /* PB1: FMU_AUX_POWER_ADC1 */
#define ADC_BATTERY2_CURRENT_CHANNEL         6 /* PF12: FMU_AUX_ADC2 */
#define ADC_AIRSPEED_VOLTAGE_CHANNEL         5 /* PF3: PRESSURE_SENS */

#define ADC_CHANNELS \
	((1 << ADC_BATTERY1_VOLTAGE_CHANNEL)       | \
	 (1 << ADC_BATTERY1_CURRENT_CHANNEL)       | \
	 (1 << ADC_SCALED_V5_CHANNEL)              | \
	 (1 << ADC_BATTERY2_VOLTAGE_CHANNEL)       | \
	 (1 << ADC_BATTERY2_CURRENT_CHANNEL)       | \
	 (1 << ADC_AIRSPEED_VOLTAGE_CHANNEL))

/* HW has to large of R termination on ADC todo:change when HW value is chosen */
#define BOARD_ADC_OPEN_CIRCUIT_V     (5.6f)

/* PWM */
#define DIRECT_PWM_OUTPUT_CHANNELS  6

/* Power supply control and monitoring GPIOs */
#define BOARD_NUMBER_BRICKS             2
#define GPIO_nVDD_BRICK1_VALID          /* PE15  */ (GPIO_INPUT|GPIO_PULLUP|GPIO_PORTE|GPIO_PIN15) // VDD_BRICK_nVALID
#define GPIO_nVDD_BRICK2_VALID          /* PE4  */ (GPIO_INPUT|GPIO_PULLUP|GPIO_PORTE|GPIO_PIN4) // VDD_BRICK2_nVALID
#define GPIO_nVDD_USB_VALID             /* PE3  */ (GPIO_INPUT|GPIO_PULLUP|GPIO_PORTE|GPIO_PIN3) // VBUS_nVALID
#define GPIO_VDD_3V3_SENSORS_EN         /* PG0  */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTG|GPIO_PIN0) // VDD_3V3_SENSORS_EN
#define GPIO_nVDD_5V_PERIPH_EN          /* PF2  */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTF|GPIO_PIN2) // nVDD_5V_PERIPH_EN
//#define GPIO_nVDD_5V_PERIPH_OC          /* PD10 */ (GPIO_INPUT|GPIO_PULLUP|GPIO_PORTD|GPIO_PIN10) // VDD_5V_PERIPH_nOC
//#define GPIO_nVDD_5V_HIPOWER_OC         /* PF4 */ (GPIO_INPUT|GPIO_PULLUP|GPIO_PORTF|GPIO_PIN4) // VDD_5V_HIPOWER_nOC
#define GPIO_ICM42688P_CLKIN_EN         /* PA8 */ (GPIO_INPUT|GPIO_FLOAT|GPIO_PORTA|GPIO_PIN8)

/* Tone alarm output */
#define TONE_ALARM_TIMER        5  /* timer 5 */
#define TONE_ALARM_CHANNEL      1  /* PA0 TIM5_CH1 */

#define GPIO_BUZZER_1           /* PA0 */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTA|GPIO_PIN0) // ALARM

#define GPIO_TONE_ALARM_IDLE    GPIO_BUZZER_1
#define GPIO_TONE_ALARM         GPIO_TIM5_CH1OUT_1

/* PWM input driver. Use FMU AUX5 pins attached to timer3 channel 1 */
#define PWMIN_TIMER                       3
#define PWMIN_TIMER_CHANNEL    /* T3C1 */ 1
#define GPIO_PWM_IN            /* PA6  */ GPIO_TIM3_CH1IN_1

/* 3 timers for PWM out */
#define BOARD_NUM_IO_TIMERS 3

/* USB OTG VBUS is not connected, we use GPIO_nVDD_USB_VALID instead to determine USB being connected. */

/* High-resolution timer */
#define HRT_TIMER               12  /* use timer12 for the HRT */
#define HRT_TIMER_CHANNEL       1  /* use capture/compare channel 1 */

#define BOARD_ADC_USB_VALID     (!px4_arch_gpioread(GPIO_nVDD_USB_VALID))
#define BOARD_ADC_BRICK1_VALID  (!px4_arch_gpioread(GPIO_nVDD_BRICK1_VALID))
#define BOARD_ADC_BRICK2_VALID  (!px4_arch_gpioread(GPIO_nVDD_BRICK2_VALID))
//#define BOARD_ADC_PERIPH_5V_OC  (!px4_arch_gpioread(GPIO_nVDD_5V_PERIPH_OC))
//#define BOARD_ADC_HIPOWER_5V_OC (!px4_arch_gpioread(GPIO_nVDD_5V_HIPOWER_OC))

/* This board provides a DMA pool and APIs */
#define BOARD_DMA_ALLOC_POOL_SIZE 5120

/* This board provides the board_on_reset interface */
#define BOARD_HAS_ON_RESET 1

#define BOARD_HAS_STATIC_MANIFEST 1

/* There is no FRAM/EEPROM */
#define FLASH_BASED_PARAMS

#define BOARD_ENABLE_CONSOLE_BUFFER

#define PX4_GPIO_INIT_LIST { \
		PX4_ADC_GPIO,                     \
		GPIO_CAN1_TX,                     \
		GPIO_CAN1_RX,                     \
		GPIO_CAN2_TX,                     \
		GPIO_CAN2_RX,                     \
		GPIO_nVDD_BRICK1_VALID,           \
		GPIO_nVDD_BRICK1_VALID,           \
		GPIO_nVDD_USB_VALID,              \
		GPIO_VDD_3V3_SENSORS_EN,          \
		GPIO_nVDD_5V_PERIPH_EN,           \
		PX4_MAKE_GPIO_OUTPUT_CLEAR(GPIO_I2C2_SCL), \
		PX4_MAKE_GPIO_OUTPUT_CLEAR(GPIO_I2C2_SDA), \
		PX4_MAKE_GPIO_OUTPUT_CLEAR(GPIO_I2C4_SCL), \
		PX4_MAKE_GPIO_OUTPUT_CLEAR(GPIO_I2C4_SDA), \
		PX4_GPIO_PIN_OFF(GPIO_SDMMC1_D0), \
		PX4_GPIO_PIN_OFF(GPIO_SDMMC1_D1), \
		PX4_GPIO_PIN_OFF(GPIO_SDMMC1_D2), \
		PX4_GPIO_PIN_OFF(GPIO_SDMMC1_D3), \
		PX4_GPIO_PIN_OFF(GPIO_SDMMC1_CMD),\
		GPIO_TONE_ALARM_IDLE,             \
		GPIO_ICM42688P_CLKIN_EN,          \
	}

//GPIO_nVDD_5V_PERIPH_OC,
//GPIO_nVDD_5V_HIPOWER_OC,

__BEGIN_DECLS
#ifndef __ASSEMBLY__

extern void stm32_spiinitialize(void);
extern void board_peripheral_reset(int ms);

#include <px4_platform_common/board_common.h>
#endif /* __ASSEMBLY__ */
__END_DECLS
