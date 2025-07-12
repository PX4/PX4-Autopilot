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
 * board internal definitions
 */

#pragma once

#ifndef CONFIG_ARCH_ARMV8M
#define CONFIG_ARCH_ARMV8M
#endif


/****************************************************************************************************
 * Included Files
 ****************************************************************************************************/

#include <px4_platform_common/px4_config.h>
#include <nuttx/compiler.h>
#include <stdint.h>


/* LEDs */
#ifdef RP23XX_BOARD_W
	// WL_GPIO0 OP connected to user LED
	#define GPIO_LED1       PX4_MAKE_GPIO_OUTPUT_CLEAR(0) // Take a look at rpi_common micro_hal.h
#else
	// LED1 - GPIO 25 - Green
	// GPIO25 OP Connected to user LED
	#define GPIO_LED1       PX4_MAKE_GPIO_OUTPUT_CLEAR(25) // Take a look at rpi_common micro_hal.h
#endif

#define GPIO_LED_BLUE   GPIO_LED1

#define BOARD_OVERLOAD_LED     LED_BLUE

// FIXME!?
//#ifndef FLASH_BASED_PARAMS
//#define FLASH_BASED_PARAMS

//#endif

//#define APP_RESERVATION_SIZE           (1 * 128 * 1024)
//
//#if !defined(BOARD_FIRST_FLASH_SECTOR_TO_ERASE)
//#  define BOARD_FIRST_FLASH_SECTOR_TO_ERASE 1
//#endif


// Not needed?
#ifdef CONFIG_RP23XX_BOARD_MADFLIGHT_FC1
   //   35		SDIO_CMD/SPI0_MOSI (bbx)
   //   36		SDIO_D0/SPI0_MISO (bbx)
   //   37		SDIO_D1 (bbx)
   //   38		SDIO_D2 (bbx)
   //   39		SDIO_D3/SPI0_CS (bbx)

//	#define CONFIG_MMCSD
//	#define CONFIG_MMCSD_SPI
//	#define CONFIG_NSH_MMCSDSPIPORTNO 0
//	#define CONFIG_NSH_MMCSDSLOTNO 0
//	#define CONFIG_NSH_MMCSDMINOR 0
#endif

/*
 * ADC channels
 *
 * These are the channel numbers of the ADCs of the microcontroller that can be used by the Px4 Firmware in the adc driver
 */
#define ADC_CHANNELS (1 << 0) | (1 << 1) | (1 << 2) | (1 << 3)	// Change this later based on the adc channels actually used

#define ADC_BATTERY_VOLTAGE_CHANNEL  1			// Corresponding GPIO 27. Used in init.c for disabling GPIO_IE
#define ADC_BATTERY_CURRENT_CHANNEL  2			// Corresponding GPIO 28. Used in init.c for disabling GPIO_IE
//#define ADC_RC_RSSI_CHANNEL          0

/* High-resolution timer */
// FIXME HRT_TIMER needed for ADC (linking issue otherwise)? but it implies GPIO_PPM_IN
#define HRT_TIMER 1
#define HRT_TIMER_CHANNEL 1
//#define HRT_PPM_CHANNEL 1	// Number really doesn't matter for this board
//#define GPIO_PPM_IN		(16 | GPIO_FUN(RP23XX_GPIO_FUNC_SIO))  // FIXME: PPM is not needed for me
#define RC_SERIAL_PORT               "/dev/ttyS1" // FIXME !!!
#define BOARD_SUPPORTS_RC_SERIAL_PORT_OUTPUT

/* This board provides a DMA pool and APIs */			// Needs to be figured out
#define BOARD_DMA_ALLOC_POOL_SIZE 2048

#define BOARD_ENABLE_CONSOLE_BUFFER
#define BOARD_CONSOLE_BUFFER_SIZE (1024*3)




#ifdef RP23XX_BOARD_W
	// W: GPIO29 OP/IP wireless SPI CLK/ADC mode (ADC3) to measure VSYS/3
	// W: WL_GPIO2 IP VBUS sense - high if VBUS is present, else low
	// W: GPIO24 OP/IP wireless SPI data/IRQ
	#define GPIO_USB_VBUS_VALID     (2 | GPIO_FUN(RP23XX_GPIO_FUNC_SIO))    // Used in usb.c
	// W: GPIO25 OP wireless SPI CS - when high also enables GPIO29 ADC pin to read VSYS

#else
	/* USB
	 *
	 *  VBUS detection is on 29  ADC_DPM0 and PTE8
	 */
	 #define GPIO_USB_VBUS_VALID     (24 | GPIO_FUN(RP23XX_GPIO_FUNC_SIO))    // Used in usb.c

	// RP2040: 24 - USB OVCUR DET - CLOCK GPOUT2 - f8=PIO2
	// Pico: GPIO24 IP VBUS sense - high if VBUS is present, else low

	// RP23XX:
	// GPIO24 IP VBUS sense - high if VBUS is present, else low
	// GPIO29 IP Used in ADC mode (ADC3) to measure VSYS/3

#endif

// PWM
// Pin 18-21, mapping is defined in timer_config.cpp
#define DIRECT_PWM_OUTPUT_CHANNELS      4

// Has pwm outputs
#define BOARD_HAS_PWM    DIRECT_PWM_OUTPUT_CHANNELS
#define BOARD_NUM_IO_TIMERS 12

/*
 * By Providing BOARD_ADC_USB_CONNECTED (using the px4_arch abstraction)
 * this board support the ADC system_power interface, and therefore
 * provides the true logic GPIO BOARD_ADC_xxxx macros.
 */

#define BOARD_ADC_USB_CONNECTED (px4_arch_gpioread(GPIO_USB_VBUS_VALID))

__BEGIN_DECLS

#ifndef __ASSEMBLY__

/****************************************************************************************************
 * Name: rp23xx_spiinitialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the PX4FMU board.
 *
 ****************************************************************************************************/

extern void rp23xx_spiinitialize(void);


/****************************************************************************************************
 * Name: rp23xx_usbinitialize
 *
 * Description:
 *   Called to configure USB IO.
 *
 ****************************************************************************************************/

extern void rp23xx_usbinitialize(void);

extern void board_peripheral_reset(int ms);

#include <px4_platform_common/board_common.h>

#endif /* __ASSEMBLY__ */

__END_DECLS
