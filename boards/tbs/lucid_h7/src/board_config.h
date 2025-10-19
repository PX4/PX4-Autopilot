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

#define GPIO_nLED_BLUE          /* PB2  */  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTA|GPIO_PIN4)
#define GPIO_nLED_GREEN         /* PB12 */  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTB|GPIO_PIN12)

#define BOARD_HAS_CONTROL_STATUS_LEDS   1
#define BOARD_ARMED_STATE_LED           0 // Green LED
#define BOARD_OVERLOAD_LED              0 // Blue LED

/*
 * ADC channels
 *
 * These are the channel numbers of the ADCs of the microcontroller that
 * can be used by the Px4 Firmware in the adc driver
 */

/* ADC defines to be used in sensors.cpp to read from a particular channel */
#define ADC1_CH(n)                  (n)
#define ADC3_CH(n)                  (n)

#define BOARD_NUMBER_BRICKS               2
#define BOARD_ADC_BRICK1_VALID            1
#define BOARD_ADC_BRICK2_VALID            1

/* Define GPIO pins used as ADC N.B. Channel numbers must match below  */
#define PX4_ADC_GPIO  \
	/* PC0  */  GPIO_ADC123_INP10, \
	/* PC1  */  GPIO_ADC123_INP11, \
	/* PC2  */  GPIO_ADC123_INP12, \
	/* PC3  */  GPIO_ADC12_INP13,	\
	/* PC4  */  GPIO_ADC12_INP4, \
	/* PC5  */  GPIO_ADC12_INP8

/* Define Channel numbers must match above GPIO pin IN(n)*/
#define ADC_BATTERY1_VOLTAGE_CHANNEL     	/* PC0  */  ADC1_CH(10)
#define ADC_BATTERY1_CURRENT_CHANNEL     	/* PC1  */  ADC1_CH(11)
#define ADC_BATTERY2_VOLTAGE_CHANNEL     	/* PC2  */  ADC1_CH(12)
#define ADC_BATTERY2_CURRENT_CHANNEL     	/* PC3  */  ADC1_CH(13)
#define ADC_SCALED_V5_CHANNEL     	 	/* PC4  */  ADC1_CH(4)
#define ADC_SCALED_VDD_3V3_SENSORS_CHANNEL 	/* PC5  */  ADC1_CH(8)

#define BOARD_ADC_USB_CONNECTED (px4_arch_gpioread(GPIO_OTGFS_VBUS))

#define ADC_CHANNELS \
	((1 << ADC_BATTERY1_VOLTAGE_CHANNEL)	| \
	(1 << ADC_BATTERY1_CURRENT_CHANNEL)	| \
	(1 << ADC_BATTERY2_VOLTAGE_CHANNEL)	| \
	(1 << ADC_BATTERY2_CURRENT_CHANNEL)	| \
	(1 << ADC_SCALED_V5_CHANNEL)		| \
	(1 << ADC_SCALED_VDD_3V3_SENSORS_CHANNEL))

// /* Define Battery 1 Voltage Divider and A per V */

#define BOARD_BATTERY1_V_DIV         (19.1f)     /* measured with the provided PM board */
#define BOARD_BATTERY1_A_PER_V       (0.0f)
#define BOARD_BATTERY2_V_DIV         (19.1f)     /* measured with the provided PM board */
#define BOARD_BATTERY2_A_PER_V       (0.0f)

/* CAN Silence
 *
 * Silent mode control \ ESC Mux select
 */

// #define GPIO_CAN1_SILENT_S0  /* PD3  */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTD|GPIO_PIN3)


/* PWM
 */
#define DIRECT_PWM_OUTPUT_CHANNELS   16
#define DIRECT_INPUT_TIMER_CHANNELS  6

#define BOARD_HAS_PWM  DIRECT_PWM_OUTPUT_CHANNELS

/* Spare GPIO */

// #define GPIO_PG6                        /* PG6  */  (GPIO_INPUT|GPIO_PULLUP|GPIO_PORTG|GPIO_PIN6)
// #define GPIO_PD15                       /* PD15 */  (GPIO_INPUT|GPIO_FLOAT|GPIO_PORTD|GPIO_PIN15)
// #define GPIO_PG15                       /* PG15 */  (GPIO_INPUT|GPIO_PULLUP|GPIO_PORTG|GPIO_PIN15)


/* Tone‑alarm output on PB9 with TIM17_CH1 ---------------------------------- */
#define TONE_ALARM_TIMER             17      /* use TIM17               */
#define TONE_ALARM_CHANNEL           1       /* channel 1               */

/* Idle (GPIO) state: low, push‑pull, 2 MHz */
#define GPIO_TONE_ALARM_IDLE \
        (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|\
         GPIO_OUTPUT_CLEAR|GPIO_PORTB|GPIO_PIN9)

/* Active ALT‑function: TIM17_CH1 on AF14 */
#define GPIO_TONE_ALARM      \
        (GPIO_ALT|GPIO_AF14|GPIO_SPEED_2MHz|GPIO_PUSHPULL|\
         GPIO_PORTB|GPIO_PIN9)


/* HEATER
 * PWM in future
 */
/* HEATER  (simple on/off GPIO ----------------------- */
#define GPIO_HEATER_OUTPUT   \
        (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|\
         GPIO_OUTPUT_CLEAR|GPIO_PORTA|GPIO_PIN8)

#define HEATER_OUTPUT_EN(on_true)  px4_arch_gpiowrite(GPIO_HEATER_OUTPUT, (on_true))

/* USB OTG FS
 *
 * PA9  OTG_FS_VBUS VBUS sensing
 */
// #define GPIO_OTGFS_VBUS         /* PA9 */ (GPIO_INPUT|GPIO_PULLDOWN|GPIO_SPEED_100MHz|GPIO_PORTA|GPIO_PIN9)
#define GPIO_OTGFS_VBUS  	/* PA9 */ (GPIO_INPUT | GPIO_FLOAT | GPIO_SPEED_50MHz | GPIO_PORTA | GPIO_PIN9)

/* High-resolution timer */
/* High‑resolution timer (HRT) --------------------------------------------- */
#define HRT_TIMER               2   /* use TIM2 for the HRT               */
#define HRT_TIMER_CHANNEL       3   /* use capture/compare channel 1      */

/* PPM input on TIM2_CH1 (PA15) -------------------------------------------- */
#define HRT_PPM_CHANNEL		1
#define GPIO_PPM_IN             GPIO_TIM2_CH1IN_2

/* Safety button ----------------------------------------------------------- */
#define GPIO_BTN_SAFETY         (GPIO_INPUT|GPIO_PULLDOWN|GPIO_PORTC|GPIO_PIN15)

/* Safety Switch is HW version dependent on having an PX4IO
 * So we init to a benign state with the _INIT definition
 * and provide the the non _INIT one for the driver to make a run time
 * decision to use it.
 */
#define GPIO_nSAFETY_SWITCH_LED_OUT_INIT   /* PB1 */ (GPIO_INPUT|GPIO_FLOAT|GPIO_PORTB|GPIO_PIN1)
#define GPIO_nSAFETY_SWITCH_LED_OUT        /* PB1 */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTB|GPIO_PIN1)

/* Enable the FMU to control it if there is no px4io fixme:This should be BOARD_SAFETY_LED(__ontrue) */
#define GPIO_LED_SAFETY GPIO_nSAFETY_SWITCH_LED_OUT

/* RC Serial port */
#define RC_SERIAL_PORT          "/dev/ttyS5"
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
		GPIO_CAN2_RX, \
		GPIO_CAN2_TX, \
		GPIO_nLED_BLUE, \
		GPIO_nLED_GREEN, \
		GPIO_TONE_ALARM_IDLE, \
	}

#define BOARD_ENABLE_CONSOLE_BUFFER

#define BOARD_NUM_IO_TIMERS 6

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
