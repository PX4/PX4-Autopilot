/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
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



/* LEDs are driven with push open drain to support Anode to 5V or 3.3V */

#  define GPIO_nLED_RED         /* PE3   */  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTE|GPIO_PIN3)
#  define GPIO_nLED_GREEN       /* PE2   */  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTE|GPIO_PIN2)
#  define GPIO_nLED_BLUE        /* PE4   */  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTE|GPIO_PIN4)

#  define BOARD_HAS_CONTROL_STATUS_LEDS      1
#  define BOARD_OVERLOAD_LED     LED_RED
#  define BOARD_ARMED_STATE_LED  LED_BLUE

/* Keep all SPI devices deselected during bootloader startup. */
#define GPIO_SPI1_CS_OSD         /* PB12 */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTB|GPIO_PIN12)
#define GPIO_SPI2_CS_BMI088_G    /* PD5  */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTD|GPIO_PIN5)
#define GPIO_SPI2_CS_BMI088_A    /* PD4  */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTD|GPIO_PIN4)
#define GPIO_SPI3_CS_ICM45686    /* PA15 */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTA|GPIO_PIN15)
#define GPIO_SPI4_CS_FRAM        /* PD10 */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTD|GPIO_PIN10)


/*
 * ADC channels
 *
 * These are the channel numbers of the ADCs of the microcontroller that
 * can be used by the Px4 Firmware in the adc driver
 */

/* ADC defines to be used in sensors.cpp to read from a particular channel */
#define SYSTEM_ADC_BASE STM32_ADC1_BASE
#define ADC1_CH(n)                  (n)

/* Define GPIO pins used as ADC N.B. Channel numbers must match below  */
#define PX4_ADC_GPIO  \
	/* PC0  */  GPIO_ADC123_INP10, \
	/* PC1  */  GPIO_ADC123_INP11


/* Define Channel numbers must match above GPIO pin IN(n)*/
#define ADC_BATTERY_VOLTAGE_CHANNEL     /* PC0  */  ADC1_CH(10)
#define ADC_BATTERY_CURRENT_CHANNEL     /* PC1  */  ADC1_CH(11)


#define ADC_CHANNELS \
	((1 << ADC_BATTERY_VOLTAGE_CHANNEL) | \
	 (1 << ADC_BATTERY_CURRENT_CHANNEL))


/* PWM
 */
#define DIRECT_PWM_OUTPUT_CHANNELS   8
#define DIRECT_INPUT_TIMER_CHANNELS  8

#define BOARD_HAS_PWM  DIRECT_PWM_OUTPUT_CHANNELS

/* USB OTG FS
 *
 * PA8  OTG_FS_VBUS VBUS sensing
 */

#define GPIO_OTGFS_VBUS         /* PA8 */ (GPIO_INPUT|GPIO_PULLDOWN|GPIO_SPEED_100MHz|GPIO_PORTA|GPIO_PIN8)


/* High-resolution timer */
#define HRT_TIMER               12 /* use timer12, matching the hardware definition */
#define HRT_TIMER_CHANNEL       1  /* use capture/compare channel 1 */


/* RC Serial port */
#define RC_SERIAL_PORT          "/dev/ttyS5"
#define BOARD_SUPPORTS_RC_SERIAL_PORT_OUTPUT

/* TJA1051T/3 silent input: low selects normal CAN operation. */
#define GPIO_CAN1_SILENT_S0     /* PD11 */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTD|GPIO_PIN11)


/* SD Card */
#define SDIO_SLOTNO             0  /* Only one slot */
#define SDIO_MINOR              0

/* This board provides a DMA pool and APIs */
#define BOARD_DMA_ALLOC_POOL_SIZE 5120

/* This board provides the board_on_reset interface */
#define BOARD_HAS_ON_RESET 1

#define PX4_GPIO_INIT_LIST { \
		PX4_ADC_GPIO, \
		GPIO_CAN1_TX,                      \
		GPIO_CAN1_RX,                      \
		GPIO_CAN1_SILENT_S0,               \
		GPIO_SPI1_CS_OSD,                  \
		GPIO_SPI2_CS_BMI088_G,             \
		GPIO_SPI2_CS_BMI088_A,             \
		GPIO_SPI3_CS_ICM45686,             \
		GPIO_SPI4_CS_FRAM,                 \
	}

#define BOARD_ENABLE_CONSOLE_BUFFER


#define BOARD_NUM_IO_TIMERS 3

/* Onboard 24LC64 EEPROM on I2C2, address 0x51. */
#define PX4_I2C_BUS_MTD       2
#define BOARD_MTD_NUM_EEPROM  1


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

/****************************************************************************
 * Name: stm32_sdio_initialize
 *
 * Description:
 *   Initialize SDIO-based MMC/SD card support
 *
 ****************************************************************************/

int stm32_sdio_initialize(void);

/****************************************************************************************************
 * Name: stm32_spiinitialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the board.
 *
 ****************************************************************************************************/

extern void stm32_spiinitialize(void);

extern void stm32_usbinitialize(void);

extern void board_peripheral_reset(int ms);

#include <px4_platform_common/board_common.h>

#endif /* __ASSEMBLY__ */

__END_DECLS
