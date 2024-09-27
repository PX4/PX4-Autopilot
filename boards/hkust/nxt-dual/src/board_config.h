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

#define FLASH_BASED_PARAMS


/* LEDs are driven with push open drain to support Anode to 5V or 3.3V */

#  define GPIO_nLED_RED         /* PD15 */  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTD|GPIO_PIN15)
#  define GPIO_nLED_GREEN       /* PD11 */  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTD|GPIO_PIN11)
#  define GPIO_nLED_BLUE        /* PB15 */  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTB|GPIO_PIN15)

#  define BOARD_HAS_CONTROL_STATUS_LEDS      1
#  define BOARD_OVERLOAD_LED     LED_RED
#  define BOARD_ARMED_STATE_LED  LED_BLUE

/* I2C busses */
/* Devices on the onboard buses.
 *
 * Note that these are unshifted addresses.
 */
// #define PX4_I2C_OBDEV_SE050         0x48

#define GPIO_SPL_ADDR_SET     /* PB5  */  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTB|GPIO_PIN5)

/*
 * ADC channels
 *
 * These are the channel numbers of the ADCs of the microcontroller that
 * can be used by the Px4 Firmware in the adc driver
 */

/* ADC defines to be used in sensors.cpp to read from a particular channel */

#define SYSTEM_ADC_BASE STM32_ADC1_BASE

#define ADC12_CH(n)		(n)

#define PX4_ADC_GPIO  \
	/* PC4  */  GPIO_ADC12_INP4,   \
	/* PC5  */  GPIO_ADC12_INP8

/* Define GPIO pins used as ADC N.B. Channel numbers must match below  */
/* Define Channel numbers must match above GPIO pin IN(n)*/
#define ADC_BATTERY_VOLTAGE_CHANNEL             ADC12_CH(4)
#define ADC_BATTERY_CURRENT_CHANNEL             ADC12_CH(5)

#define ADC_CHANNELS \
	((1 << ADC_BATTERY_VOLTAGE_CHANNEL) | \
	 (1 << ADC_BATTERY_CURRENT_CHANNEL))

#define BOARD_ADC_OPEN_CIRCUIT_V     (1.6f)



/* Define Battery 1 Voltage Divider and A per V
 */

// #define BOARD_BATTERY1_V_DIV         (11.0f)     /* measured with the provided PM board */
// #define BOARD_BATTERY1_A_PER_V       (40.0f)
// #define BOARD_BATTERY2_V_DIV         (11.0f)     /* measured with the provided PM board */

/* PWM
 */
#define DIRECT_PWM_OUTPUT_CHANNELS   8

#define BOARD_HAS_PWM  DIRECT_PWM_OUTPUT_CHANNELS


/* Spare GPIO */
#define GPIO_PA4                       	/* PA4 */  (GPIO_INPUT|GPIO_PULLUP|GPIO_PORTA|GPIO_PIN4)
#define GPIO_PC0                       	/* PC0 */  (GPIO_INPUT|GPIO_PULLUP|GPIO_PORTC|GPIO_PIN0)
#define GPIO_PC1                       	/* PC1 */  (GPIO_INPUT|GPIO_PULLUP|GPIO_PORTC|GPIO_PIN1)
/* Tone alarm output */

#define TONE_ALARM_TIMER        4 /* Timer 4 */
#define TONE_ALARM_CHANNEL      3  /* PD14 GPIO_TIM4_CH3 NC */
/*NC can be modified with Spare GPIO then connected with hardware */
#define GPIO_BUZZER_1           /* PA4 */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTA|GPIO_PIN4)

#define GPIO_TONE_ALARM_IDLE    GPIO_BUZZER_1
#define GPIO_TONE_ALARM         GPIO_BUZZER_1

/* USB OTG FS
 *
 * PD0  OTG_FS_VBUS VBUS sensing
 */
#define GPIO_OTGFS_VBUS         /* PD0 */ (GPIO_INPUT|GPIO_PULLDOWN|GPIO_SPEED_100MHz|GPIO_PORTD|GPIO_PIN0)
#define BOARD_ADC_USB_CONNECTED (px4_arch_gpioread(GPIO_OTGFS_VBUS))

/* High-resolution timer */
#define HRT_TIMER               8  /* use timer1 for the HRT */
#define HRT_TIMER_CHANNEL       1  /* use capture/compare channel 1 */

/* RC Serial port */
#define RC_SERIAL_PORT                     "/dev/ttyS4"
#define BOARD_SUPPORTS_RC_SERIAL_PORT_OUTPUT

// #define GPIO_SBUS_INV                  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTD|GPIO_PIN14)
// #define RC_INVERT_INPUT(_invert_true)  px4_arch_gpiowrite(GPIO_SBUS_INV, _invert_true);

/* SD card bringup does not work if performed on the IDLE thread because it
 * will cause waiting.  Use either:
 *
 *  CONFIG_LIB_BOARDCTL=y, OR
 *  CONFIG_BOARD_INITIALIZE=y && CONFIG_BOARD_INITTHREAD=y
 */
#define SDIO_SLOTNO             0  /* Only one slot */
#define SDIO_MINOR              0
#if defined(CONFIG_BOARD_INITIALIZE) && !defined(CONFIG_LIB_BOARDCTL) && \
   !defined(CONFIG_BOARD_INITTHREAD)
#  warning SDIO initialization cannot be perfomed on the IDLE thread
#endif

/* This board provides a DMA pool and APIs */
#define BOARD_DMA_ALLOC_POOL_SIZE 5120

/* This board provides the board_on_reset interface */
#define BOARD_HAS_ON_RESET 1

#define PX4_GPIO_INIT_LIST { \
		PX4_ADC_GPIO, \
		GPIO_TONE_ALARM_IDLE, \
		GPIO_SPL_ADDR_SET, \
		GPIO_PC0, \
		GPIO_PC1, \
	}

#define BOARD_ENABLE_CONSOLE_BUFFER

#define BOARD_NUM_IO_TIMERS 5


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
