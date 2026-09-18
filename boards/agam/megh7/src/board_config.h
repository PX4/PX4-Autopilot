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

// #define FLASH_BASED_PARAMS


/* LEDs are driven with push open drain to support Anode to 5V or 3.3V */

#define GPIO_nLED_BLUE          /* PE4 */  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTE|GPIO_PIN4)
#define GPIO_nLED_RED           /* PE3 */  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTE|GPIO_PIN3)

#define BOARD_HAS_CONTROL_STATUS_LEDS   1
#define BOARD_ARMED_STATE_LED            0 // Blue LED
#define BOARD_OVERLOAD_LED               1 // Red LED

/* Neopixel (WS2812/SK6812) LED strip, single data line, any length */

#if defined(USE_S_RGB_LED_DMA)
#define BOARD_HAS_N_S_RGB_LED       1
#define BOARD_MAX_LEDS              BOARD_HAS_N_S_RGB_LED
#define S_RGB_LED_DMA               DMAMAP_DMA12_TIM1CH1_0
#define S_RGB_LED_TIMER             1   /* timer 1    */
#define S_RGB_LED_CHANNEL           1   /* channel 1  */
#define S_RGB_LED_TIM_GPIO          GPIO_TIM1_CH1OUT_1 /* PA8 */
#endif


/*
 * ADC channels
 *
 * These are the channel numbers of the ADCs of the microcontroller that
 * can be used by the Px4 Firmware in the adc driver
 */

/* ADC defines to be used in sensors.cpp to read from a particular channel */
#define ADC1_CH(n)                  (n)

/* Define GPIO pins used as ADC N.B. Channel numbers must match below  */
#define PX4_ADC_GPIO  \
	/* PC0  */  GPIO_ADC123_INP10, \
	/* PC1  */  GPIO_ADC123_INP11, \
	/* PA4  */  GPIO_ADC12_INP18, \
	/* PC4  */  GPIO_ADC12_INP4, \
	/* PC5  */  GPIO_ADC12_INP8

/* Define Channel numbers must match above GPIO pin IN(n)*/
#define ADC_BATTERY_VOLTAGE_CHANNEL     /* PC0  */  ADC1_CH(10)
#define ADC_BATTERY_CURRENT_CHANNEL     /* PC1  */  ADC1_CH(11)
#define ADC_AIRSPEED_VOLTAGE_CHANNEL    /* PC4  */  ADC1_CH(4)
#define ADC_RC_RSSI_CHANNEL             /* PC5  */  ADC1_CH(8)

#define ADC_CHANNELS \
	((1 << ADC_BATTERY_VOLTAGE_CHANNEL) | \
	 (1 << ADC_BATTERY_CURRENT_CHANNEL) | \
	 (1 << ADC_AIRSPEED_VOLTAGE_CHANNEL) | \
	 (1 << ADC_RC_RSSI_CHANNEL))


/* Define Battery Voltage Divider and A per V */
#define BOARD_BATTERY1_V_DIV         (11.0f)     /* measured with the provided PM board */
#define BOARD_BATTERY1_A_PER_V       (40.0f)


/* PWM
 */
#define DIRECT_PWM_OUTPUT_CHANNELS   8
#define DIRECT_INPUT_TIMER_CHANNELS  8

#define BOARD_HAS_PWM  DIRECT_PWM_OUTPUT_CHANNELS


/* Tone alarm output */

#define GPIO_TONE_ALARM_IDLE    /* PA15 */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTA|GPIO_PIN15)
#define GPIO_TONE_ALARM_GPIO    /* PA15 */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTA|GPIO_PIN15)

// #define TONE_ALARM_TIMER        2  /* Timer 2 */
// #define TONE_ALARM_CHANNEL      1  /* PA15 GPIO_TIM2_CH1OUT_2 */
// #define GPIO_TONE_ALARM_IDLE    /* PA15 */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTA|GPIO_PIN15)
// #define GPIO_TONE_ALARM         GPIO_TIM2_CH1OUT_2


/* Active-high switch for the 10V supply to the analog and HD VTX, on at boot */

#define GPIO_VTX_ON             /* PE5 */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTE|GPIO_PIN5)


/* USB OTG FS
 *
 * PE2 senses VBUS through a 10K/10K divider off the USB 5V rail.
 */

#define GPIO_OTGFS_VBUS         /* PE2 */ (GPIO_INPUT|GPIO_PULLDOWN|GPIO_SPEED_100MHz|GPIO_PORTE|GPIO_PIN2)

/* By providing BOARD_ADC_USB_CONNECTED the ADC driver can report the USB
 * cable state in system_power.
 */

#define BOARD_ADC_USB_CONNECTED (px4_arch_gpioread(GPIO_OTGFS_VBUS))


/* High-resolution timer */
#define HRT_TIMER               2  /* use timer2 for the HRT */
#define HRT_TIMER_CHANNEL       1  /* use capture/compare channel 1 */


/* RC Serial port */
#define RC_SERIAL_PORT          "/dev/ttyS4"
#define BOARD_SUPPORTS_RC_SERIAL_PORT_OUTPUT

// #define GPIO_RSSI_IN            /* PC5  */ (GPIO_INPUT|GPIO_PULLUP|GPIO_PORTC|GPIO_PIN5)


/* SD Card */
#define SDIO_SLOTNO             0  /* Only one slot */
#define SDIO_MINOR              0

/* This board provides a DMA pool and APIs */
#define BOARD_DMA_ALLOC_POOL_SIZE 5120

/* This board provides the board_on_reset interface */
#define BOARD_HAS_ON_RESET 1


#define PX4_GPIO_INIT_LIST { \
		PX4_ADC_GPIO, \
		GPIO_CAN1_TX, \
		GPIO_CAN1_RX, \
		GPIO_nLED_BLUE, \
		GPIO_nLED_RED, \
		GPIO_TONE_ALARM_IDLE, \
		GPIO_VTX_ON, \
	}

#define BOARD_ENABLE_CONSOLE_BUFFER

#define BOARD_NUM_IO_TIMERS 3


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
