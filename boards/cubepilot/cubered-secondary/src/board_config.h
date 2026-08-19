/****************************************************************************
 *
 *   Copyright (c) 2025 PX4 Development Team. All rights reserved.
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
#  error "No SMPS support in NuttX submodule";
#endif


/* LEDs */
#define GPIO_nLED_AMBER        /* PD10 */ (GPIO_OUTPUT|GPIO_OPENDRAIN|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTD|GPIO_PIN10)
#define GPIO_nLED_SAFETY       /* PE5 */  (GPIO_OUTPUT|GPIO_OPENDRAIN|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTE|GPIO_PIN5)

//#define BOARD_HAS_CONTROL_STATUS_LEDS      1
//#define BOARD_ARMED_LED  LED_AMBER


/* ADC channels */
#define PX4_ADC_GPIO  \
	/* PA6 */  GPIO_ADC12_INP3,  \
	/* PF11 */  GPIO_ADC1_INP2

#define ADC_RSSI_IN_CHANNEL 3
#define ADC_SERVO_SENSE_CHANNEL 2

#define ADC_CHANNELS \
	((1 << ADC_RSSI_IN_CHANNEL) | \
	 (1 << ADC_SERVO_SENSE_CHANNEL))

/* HW has to large of R termination on ADC todo:change when HW value is chosen */
#define BOARD_ADC_OPEN_CIRCUIT_V     (5.6f)

/* PWM */
// For now, just support the basic 8 output channels, see timer_config.cpp for the rest.
#define DIRECT_PWM_OUTPUT_CHANNELS 8

/* Power supply control and monitoring GPIOs */
#define BOARD_NUMBER_BRICKS             2
#define GPIO_VDD_BRICK1_VALID          /* PG0  */ (GPIO_INPUT|GPIO_PULLUP|GPIO_PORTG|GPIO_PIN0)
#define GPIO_VDD_BRICK2_VALID          /* PG5  */ (GPIO_INPUT|GPIO_PULLUP|GPIO_PORTG|GPIO_PIN5) // Backup


/* 3 timers for PWM out */
#define BOARD_NUM_IO_TIMERS 5

/* High-resolution timer */
#define HRT_TIMER               12  /* use timer12 for the HRT */
#define HRT_TIMER_CHANNEL       1  /* use capture/compare channel 1 */

#define BOARD_ADC_BRICK1_VALID  (px4_arch_gpioread(GPIO_VDD_BRICK1_VALID))
#define BOARD_ADC_BRICK2_VALID  (px4_arch_gpioread(GPIO_VDD_BRICK2_VALID))

/* This board provides a DMA pool and APIs */
#define BOARD_DMA_ALLOC_POOL_SIZE 5120

/* This board provides the board_on_reset interface */
#define BOARD_HAS_ON_RESET 1

/* Debug GPIOs for testing - PC7 for UART IDLE debugging */
#define GPIO_UART_IDLE_DEBUG (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTC|GPIO_PIN7)  // PC7
//#define GPIO_UART_IDLE_DEBUG_READ (GPIO_INPUT|GPIO_FLOAT|GPIO_PORTA|GPIO_PIN3)


// #define GPIO_VOLT_SET (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTG|GPIO_PIN1)

// PWM Direction Control - CRITICAL: Must match hardware requirements!
// For driving Opto's (unidirectional output): BIDIR=LOW, UNIDIR=HIGH
// For bidirectional with input monitoring: BIDIR=HIGH, UNIDIR=LOW
// DANGER: Never enable both simultaneously - will damage hardware!
//
// Current config: Unidirectional mode (matches ArduPilot)
// PB0 BIDIR_ENABLED OUTPUT LOW GPIO(4)
// PA7 HP_UNIDIR_ENABLED OUTPUT HIGH GPIO(5)
#define GPIO_BIDIR_DISABLED (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTB|GPIO_PIN0)
#define GPIO_UNIDIR_ENABLED (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTA|GPIO_PIN7)


//#define BOARD_HAS_STATIC_MANIFEST 1

/* There is no FRAM/EEPROM */
#define FLASH_BASED_PARAMS

#define BOARD_ENABLE_CONSOLE_BUFFER

#define PX4_GPIO_INIT_LIST { \
		PX4_ADC_GPIO,                     \
		GPIO_CAN1_TX,                     \
		GPIO_CAN1_RX,                     \
		GPIO_CAN2_TX,                     \
		GPIO_CAN2_RX,                     \
		GPIO_VDD_BRICK1_VALID,           \
		GPIO_VDD_BRICK2_VALID,           \
		GPIO_UART_IDLE_DEBUG, \
		GPIO_BIDIR_DISABLED, \
		GPIO_UNIDIR_ENABLED, \
	}

__BEGIN_DECLS
#ifndef __ASSEMBLY__

extern void stm32_spiinitialize(void);
extern void board_peripheral_reset(int ms);

#include <px4_platform_common/board_common.h>
#endif /* __ASSEMBLY__ */
__END_DECLS
