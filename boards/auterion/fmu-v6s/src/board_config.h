/****************************************************************************
 *
 *   Copyright (c) 2016-2022 PX4 Development Team. All rights reserved.
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
 * Auterion FMU-v6s internal definitions
 */

#pragma once

/****************************************************************************************************
 * Included Files
 ****************************************************************************************************/

#include <px4_platform_common/px4_config.h>
#include <nuttx/compiler.h>
#include <stdint.h>


#include <stm32_gpio.h>

/****************************************************************************************************
 * Definitions
 ****************************************************************************************************/

/* Configuration ************************************************************************************/

/* FIXME: remove with BOARD_HAS_SIMPLE_HW_VERSIONING */
#define BOARD_HAS_HW_SPLIT_VERSIONING
#define BOARD_HAS_ONLY_EEPROM_VERSIONING

#define HW_INFO_INIT_PREFIX    "V6S"


/* Auterion FMU GPIOs ***********************************************************************************/
/* LEDs are driven with push pull */
#define GPIO_nLED_RED        /* PB14 */  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTB|GPIO_PIN14)
#define GPIO_nLED_BLUE       /* PC13 */  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTC|GPIO_PIN13)
#define GPIO_nLED_GREEN      /* PE3  */  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTE|GPIO_PIN3)

#define BOARD_HAS_CONTROL_STATUS_LEDS 1
#define BOARD_OVERLOAD_LED     LED_RED
#define BOARD_ARMED_STATE_LED  LED_BLUE


/* SPI */
#define BOARD_NUM_SPI_CFG_HW_VERSIONS 1


/* I2C busses
 * Devices on the onboard buses.
 *
 * Note that these are unshifted addresses.
 */
#define BOARD_MTD_NUM_EEPROM 1 /* MTD: imu_eeprom */
#define PX4_I2C_BUS_MTD      4


/* CAN */
#define UAVCAN_NUM_IFACES_RUNTIME  1


/*
 * ADC channels
 *
 * These are the channel numbers of the ADCs of the microcontroller that
 * can be used by the Px4 Firmware in the adc driver
 */
#define ADC1_CH(n)                  (n)
#define SYSTEM_ADC_BASE STM32_ADC1_BASE

/* Define GPIO pins used as ADC N.B. Channel numbers must match below  */
/* PC3_C connected to PC3 internally, which connects to GPIO_ADC12_INP13 */

#define PX4_ADC_GPIO \
	/* PB1 */ GPIO_ADC12_INP5, \
	/* PC0 */ GPIO_ADC123_INP10, \
	/* PC3 */ GPIO_ADC12_INP13, \
	/* PA3 */ GPIO_ADC12_INP15, \
	/* PA0 */ GPIO_ADC1_INP16, \
	/* PA4 */ GPIO_ADC12_INP18

/* Define Channel numbers must match above GPIO pin IN(n)
 * Note: These channel names are special, see board_common.h */
#define ADC_BATTERY_CURRENT_CHANNEL             /* PB1 = 0 */  ADC1_CH(5)
#define ADC_SCALED_V5_EXTERNAL_CHANNEL          /* PC0 = 1 */  ADC1_CH(10)
#define ADC_BATTERY_VOLTAGE_CHANNEL             /* PC3 = 2 */  ADC1_CH(13)
#define ADC_SCALED_V5_CHANNEL                   /* PA3 = 3 */  ADC1_CH(15)
#define ADC_EXTRAS_CHANNEL                      /* PA0 = 4 */  ADC1_CH(16)
#define ADC_SCALED_VDD_3V3_SENSORS1_CHANNEL     /* PA4 = 5 */  ADC1_CH(18)

#define ADC_CHANNELS \
	((1 << ADC_EXTRAS_CHANNEL) | \
	 (1 << ADC_SCALED_V5_EXTERNAL_CHANNEL) | \
	 (1 << ADC_SCALED_V5_CHANNEL) | \
	 (1 << ADC_BATTERY_VOLTAGE_CHANNEL) | \
	 (1 << ADC_BATTERY_CURRENT_CHANNEL) | \
	 (1 << ADC_SCALED_VDD_3V3_SENSORS1_CHANNEL))

// Definitions for drivers/adc/board_adc
#define BOARD_ADC_USB_CONNECTED 0
// a hack to also measure the external 5V rail
#define ADC_SCALED_VDD_3V3_SENSORS2_CHANNEL ADC_SCALED_V5_EXTERNAL_CHANNEL


/* PWM Timers */
#define BOARD_NUM_IO_TIMERS 2
#define DIRECT_PWM_OUTPUT_CHANNELS   8

/* High-resolution timer */
#define HRT_TIMER               8  /* use timer8 for the HRT */
#define HRT_TIMER_CHANNEL       3  /* use capture/compare channel 3 */

#define HRT_PPM_CHANNEL         /* T8C2 */  2  /* use capture/compare channel 1 */
#define GPIO_PPM_IN             /* PC7  */ GPIO_TIM8_CH2IN_1

/* RC Serial port (input only) */
#define RC_SERIAL_PORT          "/dev/ttyS4"

/* This board provides a DMA pool and APIs */
#define BOARD_DMA_ALLOC_POOL_SIZE 5120

/* This board provides the board_on_reset interface */
#define BOARD_HAS_ON_RESET 1

/* PD4 is ARMED
 * The GPIO will be set as input while not armed with internal pull DOWN.
 * While armed it shall be configured at a GPIO OUT set HIGH
 */
#define GPIO_ARMED_INIT  (GPIO_INPUT|GPIO_PULLDOWN|GPIO_PORTD|GPIO_PIN4)
#define GPIO_ARMED       (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTD|GPIO_PIN4)
#define BOARD_INDICATE_EXTERNAL_LOCKOUT_STATE(enabled)  px4_arch_configgpio((enabled) ? GPIO_ARMED : GPIO_ARMED_INIT)
#define BOARD_GET_EXTERNAL_LOCKOUT_STATE() px4_arch_gpioread(GPIO_ARMED)

/* All pins to initialize on boot */
#define PX4_GPIO_INIT_LIST { \
		GPIO_ARMED_INIT, \
		PX4_ADC_GPIO,    \
		GPIO_CAN1_TX,    \
		GPIO_CAN1_RX,    \
	}

/* Enable the buffer for the dmesg command */
#define BOARD_ENABLE_CONSOLE_BUFFER

__BEGIN_DECLS

/****************************************************************************************************
 * Public Types
 ****************************************************************************************************/

/****************************************************************************************************
 * Public data
 ****************************************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************************************
 * Public Functions
 ****************************************************************************************************/

/****************************************************************************************************
 * Name: stm32_spiinitialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the Auterion FMU board.
 *
 ****************************************************************************************************/

extern void stm32_spiinitialize(void);

extern void board_peripheral_reset(int ms);

#include <px4_platform_common/board_common.h>

#endif /* __ASSEMBLY__ */

__END_DECLS
