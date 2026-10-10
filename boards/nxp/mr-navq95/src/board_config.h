/****************************************************************************
 *
 *   Copyright (c) 2018-2019 PX4 Development Team. All rights reserved.
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
 * NXP MR-NavQ95 internal definitions
 */

#pragma once

/****************************************************************************************************
 * Included Files
 ****************************************************************************************************/

#include <nuttx/config.h>
#include <px4_boardconfig.h>

#include <px4_platform_common/px4_config.h>
#include <nuttx/compiler.h>
#include <stdint.h>

#include "imx9_gpio.h"
#include "imx9_iomuxc.h"
#include "hardware/imx9_pinmux.h"

#include <arch/board/board.h>

/****************************************************************************************************
 * Definitions
 ****************************************************************************************************/

/* SPI bus pins as GPIOs, parked as pulled-down inputs by board_spi_reset() */

#define _PIN_OFF(def) (((def) & (GPIO_PORT_MASK | GPIO_PIN_MASK)) | (GPIO_INPUT | IOMUXC_PAD_PD_ON))

#define GPIO_LPSPI1_SCK   /* SAI1_TXD0 */  (GPIO_PORT1 | GPIO_PIN13)
#define GPIO_LPSPI1_MISO  /* SAI1_TXC  */  (GPIO_PORT1 | GPIO_PIN12)
#define GPIO_LPSPI1_MOSI  /* SAI1_RXD0 */  (GPIO_PORT1 | GPIO_PIN14)

#define GPIO_LPSPI8_SCK   /* GPIO_IO15 */  (GPIO_PORT2 | GPIO_PIN15)
#define GPIO_LPSPI8_MISO  /* GPIO_IO13 */  (GPIO_PORT2 | GPIO_PIN13)
#define GPIO_LPSPI8_MOSI  /* GPIO_IO14 */  (GPIO_PORT2 | GPIO_PIN14)

/* Define Channel numbers must match above GPIO pin IN(n)*/

#define ADC_BATTERY_VOLTAGE_CHANNEL         /* VIN          ADC_IN0 */  0
#define ADC_BATTERY_CURRENT_CHANNEL         /* Not available        */ -1
#define ADC_5V_RAIL_SENSE                   /* VDD_SYS_5V0  ADC_IN1 */  1
#define ADC_SCALED_VDD_3V3_SENSORS1_CHANNEL /* VDD_SYS_3V3  ADC_IN2 */  2
#define ADC_SCALED_VDD_3V3_SENSORS2_CHANNEL /* PF09 AMUX    ADC_IN3 */  3
#define ADC_ADC_6V6_CHANNEL                 /* VDD_CON_PWM  ADC_IN6 */  6
#define ADC_ADC_3V3_CHANNEL                 /* VDD_USB_UART ADC_IN7 */  7

#define ADC_V5_V_FULL_SCALE                 (13.2f)  // 5 volt, divided by 4

#define ADC_CHANNELS \
	((1 << ADC_BATTERY_VOLTAGE_CHANNEL)  | \
	 (1 << ADC_5V_RAIL_SENSE)  | \
	 (1 << ADC_SCALED_VDD_3V3_SENSORS1_CHANNEL)  | \
	 (1 << ADC_SCALED_VDD_3V3_SENSORS2_CHANNEL)                | \
	 (1 << ADC_ADC_6V6_CHANNEL)                  | \
	 (1 << ADC_ADC_3V3_CHANNEL))

#define SYSTEM_ADC_BASE     IMX9_ADC_BASE

#define BOARD_I2C_LATEINIT 1 /* See Note about SE550 Eanable */

/* PWM
 */

/* GPIO_IO23 (J9 pin 9, TPM6 channel 1) is PWM output 8 by default, or the GPS buzzer
 * driven by the tone alarm (sounds only with switch S1 closed), selected by the board
 * Kconfig choice.
 */

#if defined(CONFIG_BOARD_NAVQ95_IO23_PWM)
#  define DIRECT_PWM_OUTPUT_CHANNELS  8
#else
#  define DIRECT_PWM_OUTPUT_CHANNELS  7
#endif

#if defined(CONFIG_BOARD_NAVQ95_IO23_BUZZER)
#  define TONE_ALARM_TIMER    6 /* TPM6 */
#  define TONE_ALARM_CHANNEL  1 /* TPM6_CH1 on GPIO_IO23 */
#endif

// Input Capture not supported on MVP

#define BOARD_HAS_NO_CAPTURE

#define GENERAL_OUTPUT_IOMUX 0

#define BOARD_NUMBER_BRICKS             1
#define BOARD_ADC_BRICK_VALID           1

/* Safety switch on pad GPIO_IO07 (GPIO2_IO07, net GPIO07_ARM_IO), GPS connector J8 pin 6, no on-board pull */

#define GPIO_BTN_SAFETY  /* GPIO_IO07 J8 pin 6 */ (GPIO_PORT2 | GPIO_PIN7 | GPIO_INPUT | IOMUXC_PAD_PU_ON)

/* The safety LED (J8 pin 7) and the RGB LED sit on the IO-board PCAL6416A I2C expander
 * at LPI2C4 address 0x20, which PX4 does not drive yet.
 */

/* By Providing BOARD_ADC_USB_CONNECTED (using the px4_arch abstraction)
 * this board support the ADC system_power interface, and therefore
 * provides the true logic GPIO BOARD_ADC_xxxx macros.
 */

#define BOARD_ADC_USB_VALID     (1)
#define BOARD_ADC_USB_CONNECTED (1)

/* The servo rail is always powered */

#define BOARD_ADC_SERVO_VALID     (1)

/* This board provides a DMA pool and APIs */
#define BOARD_DMA_ALLOC_POOL_SIZE 5120


#define PX4_GPIO_INIT_LIST { \
	}

#define BOARD_ENABLE_CONSOLE_BUFFER

#define LPTMR2_CLK		   (LPTMR2_CLK_ROOT_OSC_24M_CLK | CLOCK_DIV(24))

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

extern void imx9_spiinitialize(void);
extern void imx9_lpspi1select(FAR struct spi_dev_s *dev, uint32_t devid, bool selected);
extern void imx9_lpspi4select(FAR struct spi_dev_s *dev, uint32_t devid, bool selected);
extern void imx9_lpspi8select(FAR struct spi_dev_s *dev, uint32_t devid, bool selected);

#include <px4_platform_common/board_common.h>

#endif /* __ASSEMBLY__ */

__END_DECLS
