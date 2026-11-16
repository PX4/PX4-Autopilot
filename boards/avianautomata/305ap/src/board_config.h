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
 * avianautomata 305AP internal definitions
 */

#pragma once

/****
 * Included Files
 ****/

#include <px4_platform_common/px4_config.h>
#include <nuttx/compiler.h>
#include <stdint.h>

#include <stm32_gpio.h>

/****
 * Definitions
 ****/

/* Configuration */

#define BOARD_HAS_LTC44XX_VALIDS      0
#define BOARD_HAS_USB_VALID           0
#define BOARD_HAS_NBAT_V              1 // 1 Analog Voltage
#define BOARD_HAS_NBAT_I              1 // 1 Analog Current

/* LEDs - active low (shared line to 3.3V, GPIO LOW = ON, GPIO HIGH = OFF) */

#define GPIO_nLED_RED        /* PD9  */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTD|GPIO_PIN9)
#define GPIO_nLED_GREEN      /* PB12 */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTB|GPIO_PIN12)
#define GPIO_nLED_BLUE       /* PE12 */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTE|GPIO_PIN12)

#define BOARD_HAS_CONTROL_STATUS_LEDS      1

/* ADC channels
 *
 * VSENSE - PC4  - ADC1 INP4  - Battery voltage via 10k/1k divider (1/11 ratio)
 * ASENSE - PC3_C - ADC3 INP1 - Battery current via external sensor
 *
 * PC3_C is the analog-only companion pad for PC3 on the STM32H743.
 * SYSCFG_PMCR PC3SO defaults to 0 (switch closed), connecting PC3_C to PC3.
 * This means ADC3 INP1 (PC3_C) is readable as GPIO_ADC12_INP13 on ADC1 CH13.
 * No ADC3 driver path is required — standard board_adc on ADC1 works.
 */

#define ADC1_CH(n)                  (n)
#define ADC3_CH(n)                  (n)

#define PX4_ADC_GPIO  \
	/* PC4  */  GPIO_ADC12_INP4,   \
	/* PC3  */  GPIO_ADC12_INP13

#define ADC_BATTERY_VOLTAGE_CHANNEL         /* PC4  */  ADC1_CH(4)
#define ADC_BATTERY_CURRENT_CHANNEL         /* PC3_C via PC3 analog switch */  ADC1_CH(13)

#define ADC_CHANNELS \
	((1 << ADC_BATTERY_VOLTAGE_CHANNEL) | \
	 (1 << ADC_BATTERY_CURRENT_CHANNEL))

#define SYSTEM_ADC_BASE STM32_ADC1_BASE

/* CAN transceiver STB pins (SIT1044TK, ACTIVE-LOW: LOW = normal, HIGH = standby).
 * Board has pulldowns so default is normal/on; drive LOW to keep transceivers enabled. */

#define GPIO_CAN1_EN            /* PA10 */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTA|GPIO_PIN10)
#define GPIO_CAN2_EN            /* PE1  */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTE|GPIO_PIN1)

/* PWM */

#define DIRECT_PWM_OUTPUT_CHANNELS   8

/* Tone alarm / Buzzer - TIM5 CH4 on PA3 */

#define TONE_ALARM_TIMER        5  /* Timer 5 - not used for PWM on this board */
#define TONE_ALARM_CHANNEL      4  /* PA3 GPIO_TIM5_CH4OUT */

#define GPIO_BUZZER_1           /* PA3 */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTA|GPIO_PIN3)

#define GPIO_TONE_ALARM_IDLE    GPIO_BUZZER_1
#define GPIO_TONE_ALARM         GPIO_TIM5_CH4OUT_1

/* USB - no VBUS sensing on this board */

/* High-resolution timer */

#define HRT_TIMER               8  /* use timer8 for the HRT */
#define HRT_TIMER_CHANNEL       3  /* use capture/compare channel 3 */

/* RC Serial port - UART5 single-wire (half-duplex) on PB13, the UART5 TX pin. No RX/TX swap:
 * half-duplex runs on the TX signal, and swapping moves it to the RX pin (PC0, not connected). */

#define RC_SERIAL_PORT                     "/dev/ttyS4"
#define RC_SERIAL_SINGLEWIRE

/* SD card */

#define SDIO_SLOTNO                    0  /* Only one slot */
#define SDIO_MINOR                     0

#define GPIO_SDMMC1_NCD            /* PE3 */ (GPIO_INPUT|GPIO_PULLUP|GPIO_SPEED_2MHz|GPIO_PORTE|GPIO_PIN3)

#if defined(CONFIG_BOARD_INITIALIZE) && !defined(CONFIG_BOARDCTL) && \
   !defined(CONFIG_BOARD_INITTHREAD)
#  warning SDIO initialization cannot be performed on the IDLE thread
#endif

/* Power monitoring - no digital brick valid, no USB valid, no overcurrent detection */

#define BOARD_ADC_SERVO_VALID     (1)
#define BOARD_ADC_BRICK1_VALID    (1)
#define BOARD_ADC_BRICK2_VALID    (0)

/* This board provides a DMA pool and APIs */
#define BOARD_DMA_ALLOC_POOL_SIZE 5120

/* This board provides the board_on_reset interface */
#define BOARD_HAS_ON_RESET 1

/* GPIO init list - all GPIOs configured at boot */

/* FDCAN pins (board.h) must be muxed here: neither the uavcan stm32h7 driver nor NuttX (without
 * its own FDCAN driver) configures them, so FDCAN1/2 were never connected to the transceivers. */
#define PX4_GPIO_INIT_LIST { \
		PX4_ADC_GPIO,                     \
		GPIO_CAN1_TX,                     \
		GPIO_CAN1_RX,                     \
		GPIO_CAN2_TX,                     \
		GPIO_CAN2_RX,                     \
		GPIO_CAN1_EN,                     \
		GPIO_CAN2_EN,                     \
		GPIO_TONE_ALARM_IDLE              \
	}

#define BOARD_ENABLE_CONSOLE_BUFFER

#define BOARD_NUM_IO_TIMERS 4

__BEGIN_DECLS

/****
 * Public Types
 ****/

/****
 * Public data
 ****/

#ifndef __ASSEMBLY__

/****
 * Public Functions
 ****/

int stm32_sdio_initialize(void);

extern void stm32_spiinitialize(void);

extern void stm32_usbinitialize(void);

extern void board_peripheral_reset(int ms);

#include <px4_platform_common/board_common.h>

#endif /* __ASSEMBLY__ */

__END_DECLS
