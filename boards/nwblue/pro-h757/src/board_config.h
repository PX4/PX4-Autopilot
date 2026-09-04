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
 * NWBlue Pro H757 internal definitions.
 * STM32H757 on the CubePilot CubeNode module carried by this board.
 */

#pragma once

#include <px4_platform_common/px4_config.h>
#include <nuttx/compiler.h>
#include <stdint.h>
#include <stm32_gpio.h>

/* If NuttX is built without support for SMPS it can brick the hardware. */
#include "hardware/stm32h7x3xx_pwr.h"
#if STM32_PWR_CR3_SMPSEXTHP != (1 << 3)
#  error "No SMPS support in NuttX submodule");
#endif

#define BOARD_HAS_USB_VALID            1
#define BOARD_HAS_NBAT_V               1
#define BOARD_HAS_NBAT_I               1

/* LEDs (active-low: anode to +3.3V, cathode through resistor to GPIO) */

#define GPIO_nLED_RED       /* PF12 */ (GPIO_OUTPUT | GPIO_OPENDRAIN | GPIO_SPEED_50MHz | GPIO_OUTPUT_SET | GPIO_PORTF | GPIO_PIN12)
#define GPIO_nLED_GREEN     /* PF13 */ (GPIO_OUTPUT | GPIO_OPENDRAIN | GPIO_SPEED_50MHz | GPIO_OUTPUT_SET | GPIO_PORTF | GPIO_PIN13)
#define GPIO_nLED_BLUE      /* PA5  */ (GPIO_OUTPUT | GPIO_OPENDRAIN | GPIO_SPEED_50MHz | GPIO_OUTPUT_SET | GPIO_PORTA | GPIO_PIN5)

/* Status LED (active-low). Driven by the bootloader only; the application
 * leaves it alone.
 */
#define GPIO_LED_STATUS     /* PC0  */ (GPIO_OUTPUT | GPIO_OPENDRAIN | GPIO_SPEED_50MHz | GPIO_OUTPUT_SET | GPIO_PORTC | GPIO_PIN0)

/* Discrete LEDs driven by commander: blue blinks the arming state (1 Hz ready
 * to arm, 10 Hz not ready, 4 Hz failsafe), green is solid when armed, red is
 * the overload/error indication.
 */
#define BOARD_HAS_CONTROL_STATUS_LEDS 1
#define BOARD_ARMED_LED               LED_GREEN
#define BOARD_ARMED_STATE_LED         LED_BLUE
#define BOARD_OVERLOAD_LED            LED_RED

/* ADC channels, per schematic - all on ADC3:
 *   PC2 / ADC3_INP0   CURRENT_SENSE  (ESC connector, R28 = 1k series)
 *   PC3 / ADC3_INP1   3V3_SENSE      (R18 10k pull-down, no divider)
 *   PF5 / ADC3_INP4   VREG_SENSE
 *   PF3 / ADC3_INP5   SYSVIN_SENSE   (200k:10k divider, 21x)
 *   PF4 / ADC3_INP9   5V_SENSE
 *
 * Note the schematic labels PF4 as ADC3_INP0; PF4 is INP9, INP0 is PC2.
 *
 * PC2/PC3 reach ADC3_INP0/INP1 through the PC2_C/PC3_C dual pads, connected by
 * the SYSCFG_PMCR analog switches at their reset default. They have no GPIO
 * configuration (no GPIO_PIN2_C/PIN3_C in the H7 pinmap) and are enabled
 * through the ADC_CHANNELS mask alone.
 */

/* All carrier sense channels live on ADC3, so make ADC3 the primary ADC. */
#define SYSTEM_ADC_BASE             STM32_ADC3_BASE

/* 3V3_SENSE is tied to the 3.3 V rail with only a 10 k pull-down, so the ADC
 * sees the full rail voltage. With VREF also 3.3 V it reads full scale at
 * nominal and only moves if the rail droops.
 */
#define ADC_3V3_SCALE               (1.0f)

#define ADC1_CH(n)                  (n)
#define ADC3_CH(n)                  (n)

#define PX4_ADC_GPIO  \
	/* PF5   */ GPIO_ADC3_INP4,  \
	/* PF3   */ GPIO_ADC3_INP5,  \
	/* PF4   */ GPIO_ADC3_INP9

#define ADC_BATTERY_VOLTAGE_CHANNEL          /* PF3 */ ADC3_CH(5)   /* SYSVIN  */
#define ADC_BATTERY_CURRENT_CHANNEL          /* PC2 */ ADC3_CH(0)   /* CURRENT */
#define ADC_SCALED_V5_CHANNEL                /* PF4 */ ADC3_CH(9)   /* 5V      */
#define ADC_SCALED_VDD_3V3_SENSORS_CHANNEL   /* PC3 */ ADC3_CH(1)   /* 3V3     */
#define ADC_VREG_SENSE_CHANNEL               /* PF5 */ ADC3_CH(4)   /* VREG    */

#define ADC_CHANNELS \
	((1 << ADC_BATTERY_VOLTAGE_CHANNEL)        | \
	 (1 << ADC_BATTERY_CURRENT_CHANNEL)        | \
	 (1 << ADC_SCALED_V5_CHANNEL)              | \
	 (1 << ADC_SCALED_VDD_3V3_SENSORS_CHANNEL) | \
	 (1 << ADC_VREG_SENSE_CHANNEL))

#define BOARD_ADC_OPEN_CIRCUIT_V     (5.6f)

/* PWM */
#define DIRECT_PWM_OUTPUT_CHANNELS  9
#define BOARD_NUM_IO_TIMERS         4

/* Tone alarm on PC8, driven as TIM8_CH3 PWM at the note frequency. Active
 * high, idle off.
 */
#define TONE_ALARM_TIMER        8  /* timer 8 */
#define TONE_ALARM_CHANNEL      3  /* PC8 TIM8_CH3 */

#define GPIO_TONE_ALARM_IDLE    /* PC8 */ (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_2MHz | GPIO_OUTPUT_CLEAR | GPIO_PORTC | GPIO_PIN8)
#define GPIO_TONE_ALARM         GPIO_TIM8_CH3OUT_1

/* RC input on UART4 */
#define RC_SERIAL_PORT          "/dev/ttyS2"

/* USB OTG FS - PA9 OTG_FS_VBUS sensing */
#define GPIO_OTGFS_VBUS         /* PA9 */ (GPIO_INPUT | GPIO_PULLDOWN | GPIO_SPEED_100MHz | GPIO_PORTA | GPIO_PIN9)

/* Timer allocation:
 *   TIM1/2/3/4  PWM/DShot outputs (see timer_config.cpp)
 *   TIM6        uavcan clock (CONFIG_BOARD_UAVCAN_TIMER_OVERRIDE)
 *   TIM8        tone alarm
 *   TIM12       HRT time-base, no GPIO export
 */
#define HRT_TIMER               12
#define HRT_TIMER_CHANNEL       1

/* SDIO */
#define SDIO_SLOTNO             0
#define SDIO_MINOR              0

#if defined(CONFIG_BOARD_INITIALIZE) && !defined(CONFIG_BOARDCTL) && \
   !defined(CONFIG_BOARD_INITTHREAD)
#  warning SDIO initialization cannot be perfomed on the IDLE thread
#endif

#define BOARD_ADC_USB_CONNECTED (px4_arch_gpioread(GPIO_OTGFS_VBUS))
#define BOARD_ADC_SERVO_VALID   (1)
#define BOARD_ADC_BRICK1_VALID  (1)

/* This board provides a DMA pool and APIs */
#define BOARD_DMA_ALLOC_POOL_SIZE 5120

/* This board provides the board_on_reset interface */
#define BOARD_HAS_ON_RESET 1

#define BOARD_HAS_STATIC_MANIFEST 1

#define FLASH_BASED_PARAMS

#define BOARD_ENABLE_CONSOLE_BUFFER

#define PX4_GPIO_INIT_LIST { \
		PX4_ADC_GPIO,        \
		GPIO_TONE_ALARM_IDLE,\
		GPIO_OTGFS_VBUS,     \
		GPIO_CAN1_TX,        \
		GPIO_CAN1_RX,        \
		GPIO_CAN_SHUTDOWN,   \
		GPIO_CAN_SLEEP,      \
	}

__BEGIN_DECLS
#ifndef __ASSEMBLY__

extern void stm32_spiinitialize(void);
extern void board_peripheral_reset(int ms);

#ifdef CONFIG_USBHOST
extern int stm32_usbhost_initialize(void);
#endif

#include <px4_platform_common/board_common.h>
#endif /* __ASSEMBLY__ */
__END_DECLS
