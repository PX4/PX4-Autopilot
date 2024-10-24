/****************************************************************************
 *
 *   Copyright (c) 2021 Technology Innovation Institute. All rights reserved.
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
 * Icicle internal definitions
 */

#pragma once

/****************************************************************************************************
 * Included Files
 ****************************************************************************************************/

#include <px4_platform_common/px4_config.h>
#include <nuttx/compiler.h>
#include <stdint.h>
#include "board_type.h"

/****************************************************************************************************
 * Definitions
 ****************************************************************************************************/

#define BOARD_REVISIONS {{"PolarFire MPFS250T", '1', NULL}, \
		{"PolarFire MPFS250T", '1', NULL}}

/* Configuration ************************************************************************************/

/* Icicle GPIOs *************************************************************************************/

#define BOARD_HAS_HW_VERSIONING
#define BOARD_HAS_MULTIPURPOSE_VERSION_PINS
#define GPIO_HW_VERSION_PIN1    (GPIO_BANK2 | GPIO_PIN20 | GPIO_INPUT)
#define GPIO_HW_VERSION_PIN2    (GPIO_BANK2 | GPIO_PIN21 | GPIO_INPUT)
#define GPIO_HW_VERSION_PIN3    (GPIO_BANK2 | GPIO_PIN22 | GPIO_INPUT)
#define HW_INFO_INIT_PREFIX     "Icicle "

#define GPIO_nSAFETY_SWITCH_LED_OUT (GPIO_BANK2 | GPIO_PIN26 | GPIO_OUTPUT | GPIO_BUFFER_ENABLE) // GPS safety switch LED
#define GPIO_LED_SAFETY GPIO_nSAFETY_SWITCH_LED_OUT

#define GPIO_SAFETY_SWITCH_IN (GPIO_BANK2 | GPIO_PIN25 | GPIO_INPUT) // GPS safety switch
#define GPIO_BTN_SAFETY GPIO_SAFETY_SWITCH_IN

#define TONE_ALARM_PWM_OUT_PATH "/dev/pwm1"

/* LEDS */
//#define GPIO_nSAFETY_SWITCH_LED_OUT (GPIO_BANK2 | GPIO_PIN16 | GPIO_OUTPUT | GPIO_BUFFER_ENABLE) // ICICLE LED 1
#define GPIO_nLED_RED        (GPIO_BANK2 | GPIO_PIN17 | GPIO_OUTPUT | GPIO_BUFFER_ENABLE) // ICICLE LED 2
// NB: These are both yellow on Icicle board:
#define GPIO_nLED_GREEN      (GPIO_BANK2 | GPIO_PIN18 | GPIO_OUTPUT | GPIO_BUFFER_ENABLE) // ICICLE LED 3
#define GPIO_nLED_BLUE       (GPIO_BANK2 | GPIO_PIN19 | GPIO_OUTPUT | GPIO_BUFFER_ENABLE) // ICICLE LED 4

//#define BOARD_HAS_CONTROL_STATUS_LEDS      1
//#define BOARD_OVERLOAD_LED     LED_RED
//#define BOARD_ARMED_STATE_LED  LED_BLUE

/* I2C */
#define I2C_RESET_SPEED         I2C_SPEED_FAST
#define BOARD_I2C_BUS_CLOCK_INIT {I2C_SPEED_FAST, I2C_SPEED_FAST}

/* RC Serial port */

#define RC_SERIAL_PORT                     "/dev/ttyS2"
#define RC_SERIAL_SINGLEWIRE

#define BOARD_HAS_ON_RESET 1

#define PX4_GPIO_INIT_LIST { \
	}

#define BOARD_ENABLE_CONSOLE_BUFFER

#define BOARD_NUM_IO_TIMERS 0

#define BOARD_DSHOT_MOTOR_ASSIGNMENT {3, 2, 1, 0, 4, 5, 6, 7};

/* eMMC/SD */

#define SDIO_SLOTNO 0
#define SDIO_MINOR  0

/* Battery ADC */
#define ADC_CHANNELS (1 << 1) | (1 << 2)
#define ADC_BATTERY_VOLTAGE_CHANNEL 1
#define ADC_BATTERY_CURRENT_CHANNEL 2

/* PX4 Boot image type; 0 to boot directly, 1 through SBI */
#define INFO_BIT_USE_SBI 1

#ifdef CONFIG_BUILD_KERNEL
#define PX4_VENDOR_BOOT_FLAGS INFO_BIT_USE_SBI
#else
#define PX4_VENDOR_BOOT_FLAGS 0
#endif

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

extern void board_peripheral_reset(int ms);
extern int mpfs_board_emmcsd_init(void);
extern int mpfs_board_register_partition(unsigned partition);
extern int mpfs_board_spinor_init(struct spi_dev_s *spinor);
extern int mpfs_pwm_setup(void);
extern void board_spidev_initialize(void);
extern int board_spibus_initialize(void);
extern int board_domains_init(void);

#ifdef CONFIG_USBDEV
extern void mpfs_usbinitialize(void);
#endif

#include <px4_platform_common/board_common.h>

#endif /* __ASSEMBLY__ */

__END_DECLS
