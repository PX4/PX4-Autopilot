/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
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
 * AirBrainH743 (Gear Up) internal definitions
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

/* Enable small flash logging support (for W25N NAND flash) */
#ifdef CONFIG_MTD_W25N
#  define BOARD_SMALL_FLASH_LOGGING 1
#endif

/* LEDs are active low
 * STAT RGB LED:
 *   PB15 = Blue
 *   PD11 = Green
 *   PD15 = Red
 * BAT LED (orange): hardwired to power input
 */
#define GPIO_nLED_BLUE       /* PB15 */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTB|GPIO_PIN15)
#define GPIO_nLED_GREEN      /* PD11 */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTD|GPIO_PIN11)
#define GPIO_nLED_RED        /* PD15 */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTD|GPIO_PIN15)

#define BOARD_HAS_CONTROL_STATUS_LEDS   1
#define BOARD_OVERLOAD_LED              LED_RED
#define BOARD_ARMED_STATE_LED           LED_GREEN

/*
 * ADC channels
 *
 * These are the channel numbers of the ADCs of the microcontroller that
 * can be used by the Px4 Firmware in the adc driver
 */

/* ADC defines to be used in sensors.cpp to read from a particular channel */
#define ADC1_CH(n)                  (n)

/* Define GPIO pins used as ADC N.B. Channel numbers must match below */
#define PX4_ADC_GPIO  \
	/* PC4 */  GPIO_ADC12_INP4, \
	/* PC5 */  GPIO_ADC12_INP8

/* Define Channel numbers must match above GPIO pin IN(n)*/
#define ADC_BATTERY_VOLTAGE_CHANNEL     /* PC4 */  ADC1_CH(4)
#define ADC_BATTERY_CURRENT_CHANNEL     /* PC5 */  ADC1_CH(8)

#define ADC_CHANNELS \
	((1 << ADC_BATTERY_VOLTAGE_CHANNEL) | \
	 (1 << ADC_BATTERY_CURRENT_CHANNEL))


/* Define Battery Voltage Divider and A per V
 */
#define BOARD_BATTERY1_V_DIV         (15.0f)
#define BOARD_BATTERY1_A_PER_V       (101.0f)


/* PWM
 * 8 PWM outputs for motors + 1 for LED strip
 */
#define DIRECT_PWM_OUTPUT_CHANNELS   9
#define DIRECT_INPUT_TIMER_CHANNELS  9

#define BOARD_HAS_PWM  DIRECT_PWM_OUTPUT_CHANNELS


/* Tone alarm output (directly connected to transistor switch of external buzzer)
 *
 * GPIO mode only (active buzzer) - passive buzzer with different tones is not
 * supported because PA15 can only use TIM2, which is also used for motor outputs
 * M7 (PB10, TIM2_CH3) and M8 (PB11, TIM2_CH4). The PWM tone alarm driver changes
 * the timer's prescaler and auto-reload registers (shared across all channels),
 * which would affect M7/M8 PWM frequency during tone playback.
 */
#define GPIO_TONE_ALARM_IDLE    /* PA15 */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTA|GPIO_PIN15)
#define GPIO_TONE_ALARM_GPIO    /* PA15 */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTA|GPIO_PIN15)


/* ICM42688P FSYNC - directly connected to IMU via GPIO (no timer).
 * The driver clears TMST_FSYNC_EN and FIFO_TMST_FSYNC_EN, so FSYNC is unused.
 * This GPIO is kept low to prevent spurious triggers.
 */
#define GPIO_42688P_FSYNC       /* PC7 */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTC|GPIO_PIN7)


/* USB OTG FS
 *
 * PD0 VBUS sensing (active high input)
 */
#define GPIO_OTGFS_VBUS         /* PD0 */ (GPIO_INPUT|GPIO_PULLDOWN|GPIO_SPEED_100MHz|GPIO_PORTD|GPIO_PIN0)


/* High-resolution timer */
#define HRT_TIMER               8  /* use timer8 for the HRT */
#define HRT_TIMER_CHANNEL       3  /* use capture/compare channel 3 */


/*
 * Serial Port Mapping:
 *
 *   UART    Device    Pins          Function
 *   ----    ------    ----          --------
 *   USART1  /dev/ttyS0  PA9/PA10    Console/Debug
 *   USART2  /dev/ttyS1  PD5/PD6     RC Input
 *   USART3  /dev/ttyS2  PD8/PD9     TEL4 (DJI/MSP)
 *   UART4   /dev/ttyS3  PA0/PA1     TEL1
 *   UART5   /dev/ttyS4  PB13/PB12   TEL2
 *   UART7   /dev/ttyS5  PE8/PE7     TEL3 (ESC Telemetry)
 *   UART8   /dev/ttyS6  PE1/PE0     GPS1
 */

/* RC Serial port - USART2 (PD5/PD6) */
#define RC_SERIAL_PORT          "/dev/ttyS1"
#define BOARD_SUPPORTS_RC_SERIAL_PORT_OUTPUT


/* This board provides a DMA pool and APIs */
#define BOARD_DMA_ALLOC_POOL_SIZE 5120

/* This board provides the board_on_reset interface */
#define BOARD_HAS_ON_RESET 1


#define PX4_GPIO_INIT_LIST { \
		PX4_ADC_GPIO, \
		GPIO_nLED_RED, \
		GPIO_nLED_GREEN, \
		GPIO_nLED_BLUE, \
		GPIO_TONE_ALARM_IDLE, \
		GPIO_42688P_FSYNC, \
	}

#define BOARD_ENABLE_CONSOLE_BUFFER

#define BOARD_NUM_IO_TIMERS 4


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
 * Name: stm32_spiinitialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the board.
 *
 ****************************************************************************/

extern void stm32_spiinitialize(void);

extern void stm32_usbinitialize(void);

extern void board_peripheral_reset(int ms);

/* Parameters stored in internal flash */
#define FLASH_BASED_PARAMS

#include <px4_platform_common/board_common.h>

#endif /* __ASSEMBLY__ */

__END_DECLS
