/****************************************************************************
 *
 *   Copyright (c) 2024 PX4 Development Team. All rights reserved.
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
 * NXP Tropic internal definitions
 */

#pragma once

/****************************************************************************************************
 * Included Files
 ****************************************************************************************************/

#include <px4_platform_common/px4_config.h>
#include <nuttx/compiler.h>
#include <stdint.h>

#include "imxrt_gpio.h"
#include "imxrt_iomuxc.h"
#include "hardware/imxrt_pinmux.h"

#include <arch/board/board.h>

/****************************************************************************************************
 * Definitions
 ****************************************************************************************************/

/* Configuration ************************************************************************************/

/* FMURT1062 GPIOs ***********************************************************************************/
/* LEDs */
/* An RGB LED is connected through GPIO as shown below:
 */
#define LED_IOMUX (IOMUX_OPENDRAIN | IOMUX_PULL_NONE | IOMUX_DRIVE_33OHM | IOMUX_SPEED_MEDIUM | IOMUX_SLEW_SLOW)
#define GPIO_nLED_RED   /* GPIO_AD_B0_02 GPIO1_IO02 */ (GPIO_PORT1 | GPIO_PIN2  | GPIO_OUTPUT | GPIO_OUTPUT_ZERO | LED_IOMUX)
#define GPIO_nLED_GREEN /* GPIO_B1_03    GPIO2_IO19 */ (GPIO_PORT2 | GPIO_PIN19 | GPIO_OUTPUT | GPIO_OUTPUT_ZERO | LED_IOMUX)
#define GPIO_nLED_BLUE  /* GPIO_AD_B0_03 GPIO1_IO03 */ (GPIO_PORT1 | GPIO_PIN3  | GPIO_OUTPUT | GPIO_OUTPUT_ZERO | LED_IOMUX)

#define BOARD_HAS_CONTROL_STATUS_LEDS   1
#define BOARD_OVERLOAD_LED              LED_RED
#define BOARD_ARMED_STATE_LED           LED_BLUE

/*
 *  Define the ability to shut off off the sensor signals
 *  by changing the signals to inputs
 */

#define _PIN_OFF(def) (((def) & (GPIO_PORT_MASK | GPIO_PIN_MASK)) | (GPIO_INPUT | IOMUX_PULL_DOWN_100K | IOMUX_CMOS_INPUT))

/*  Define the Chip Selects, Data Ready and Control signals per SPI bus */

#define CS_IOMUX  (IOMUX_CMOS_OUTPUT | IOMUX_PULL_UP_100K | IOMUX_DRIVE_33OHM | IOMUX_SPEED_LOW | IOMUX_SLEW_FAST)
#define OUT_IOMUX (IOMUX_CMOS_OUTPUT | IOMUX_PULL_UP_100K | IOMUX_DRIVE_50OHM | IOMUX_SPEED_MEDIUM | IOMUX_SLEW_FAST)


/* SPI1 off */

#define _GPIO_LPSPI1_SCK   /* GPIO_EMC_27  GPIO4_IO27 */  (GPIO_PORT4 | GPIO_PIN27 | CS_IOMUX)
#define _GPIO_LPSPI1_MISO  /* GPIO_EMC_29  GPIO4_IO29 */  (GPIO_PORT4 | GPIO_PIN29 | CS_IOMUX)
#define _GPIO_LPSPI1_MOSI  /* GPIO_EMC_28  GPIO4_IO28 */  (GPIO_PORT4 | GPIO_PIN28 | CS_IOMUX)

#define GPIO_SPI1_SCK_OFF   _PIN_OFF(_GPIO_LPSPI1_SCK)
#define GPIO_SPI1_MISO_OFF  _PIN_OFF(_GPIO_LPSPI1_MISO)
#define GPIO_SPI1_MOSI_OFF  _PIN_OFF(_GPIO_LPSPI1_MOSI)

#define _GPIO_LPSPI3_SCK   /* GPIO_AD_B1_15 GPIO1_IO27 */  (GPIO_PORT1 | GPIO_PIN31 | CS_IOMUX)
#define _GPIO_LPSPI3_MISO  /* GPIO_AD_B1_13 GPIO1_IO27 */  (GPIO_PORT1 | GPIO_PIN29 | CS_IOMUX)
#define _GPIO_LPSPI3_MOSI  /* GPIO_AD_B1_14 GPIO1_IO27 */  (GPIO_PORT1 | GPIO_PIN30 | CS_IOMUX)

#define GPIO_SPI3_SCK_OFF   _PIN_OFF(_GPIO_LPSPI3_SCK)
#define GPIO_SPI3_MISO_OFF  _PIN_OFF(_GPIO_LPSPI3_MISO)
#define GPIO_SPI3_MOSI_OFF  _PIN_OFF(_GPIO_LPSPI3_MOSI)

/*  Define the SPI4 Data Ready and Control signals */

#define GPIO_SPI4_DRDY7_EXTERNAL1   /* GPIO_EMC_35 GPIO3_IO21*/ (GPIO_PORT3 | GPIO_PIN21 | GPIO_INPUT  | DRDY_IOMUX)

#define GPIO_DRDY_OFF_SPI4_DRDY7_EXTERNAL1   _PIN_OFF(GPIO_SPI4_DRDY7_EXTERNAL1)


#define ADC_IOMUX (IOMUX_CMOS_INPUT | IOMUX_PULL_NONE | IOMUX_DRIVE_HIZ)

#define ADC1_CH(n)                  (n)
#define ADC1_GPIO(n, p)             (GPIO_PORT1 | GPIO_PIN##p | ADC_IOMUX) //

/* Define GPIO pins used as ADC N.B. Channel numbers are for reference, */

#define PX4_ADC_GPIO  \
	/* BATTERY1_VOLTAGE       GPIO_AD_B1_08 GPIO1 Pin 27 */  ADC1_GPIO(0,  27),  \
	/* BATTERY1_CURRENT       GPIO_AD_B1_11 GPIO1 Pin 24 */  ADC1_GPIO(0,  24)

/* Define Channel numbers must match above GPIO pin IN(n)*/

#define ADC_BATTERY_VOLTAGE_CHANNEL        /* GPIO_AD_B1_08 GPIO1 Pin 27 */  ADC1_CH(13)
#define ADC_BATTERY_CURRENT_CHANNEL        /* GPIO_AD_B1_11 GPIO1 Pin 12 */  ADC1_CH(0)

#define ADC_CHANNELS \
	((1 << ADC_BATTERY_VOLTAGE_CHANNEL)       | \
	 (1 << ADC_BATTERY_CURRENT_CHANNEL))

/* HW has to large of R termination on ADC todo:change when HW value is chosen */

#define BOARD_ADC_OPEN_CIRCUIT_V     (5.6f)

/* PWM
 */

#define DIRECT_PWM_OUTPUT_CHANNELS  6
#define BOARD_NUM_IO_TIMERS         4

// Input Capture not supported on MVP

#define BOARD_HAS_NO_CAPTURE

//#define BOARD_HAS_UI_LED_PWM           1  Not ported yet (Still Kinetis driver)
#define BOARD_HAS_LED_PWM              1
#define BOARD_LED_PWM_DRIVE_ACTIVE_LOW 1

/*  UI LEDs are driven by timer 4 the pins have no alternates
 *
 *  nUI_LED_RED   GPIO_B0_10 GPIO2_IO10 QTIMER4_TIMER1
 *  nUI_LED_GREEN GPIO_B0_11 GPIO2_IO11 QTIMER4_TIMER2
 *  nUI_LED_BLUE  GPIO_B1_11 GPIO2_IO27 QTIMER4_TIMER3
 */


/* Power supply control and monitoring GPIOs */

#define GENERAL_INPUT_IOMUX  (IOMUX_CMOS_INPUT |  IOMUX_PULL_UP_47K | IOMUX_DRIVE_HIZ)
#define GENERAL_OUTPUT_IOMUX (IOMUX_CMOS_OUTPUT | IOMUX_PULL_KEEP | IOMUX_DRIVE_33OHM  | IOMUX_SPEED_MEDIUM | IOMUX_SLEW_FAST)

/* Tone alarm output */

//FIXME FlexPWM TONE DRIVER
//#define TONE_ALARM_TIMER        2  /* GPT 2 */
//#define TONE_ALARM_CHANNEL      3  /* GPIO_AD_B1_07 GPT2_COMPARE3 */

#define GPIO_BUZZER_1           /* GPIO_EMC_31  GPIO4_IO31  */ (GPIO_PORT1 | GPIO_PIN23  | GPIO_OUTPUT | GPIO_OUTPUT_ZERO | GENERAL_OUTPUT_IOMUX)

#define GPIO_TONE_ALARM_IDLE    GPIO_BUZZER_1
#define GPIO_TONE_ALARM         (GPIO_FLEXPWM3_PWMA01_1 | GENERAL_OUTPUT_IOMUX)

/* USB OTG FS
 *
 * VBUS_VALID is detected in USB_ANALOG_USB1_VBUS_DETECT_STAT
 */

/* High-resolution timer */
#define HRT_TIMER               1  /* use GPT1 for the HRT */
#define HRT_TIMER_CHANNEL       1  /* use capture/compare channel 1 */

#define RC_SERIAL_PORT                  "/dev/ttyS4"
#define RC_SERIAL_SINGLEWIRE            1 // Suport Single wire wiring
#define RC_SERIAL_SWAP_RXTX             1 // Set Swap (but not supported in HW) to use Single wire
#define RC_SERIAL_SWAP_USING_SINGLEWIRE 1 // Set to use Single wire swap as HW does not support swap

/* Safety Switch is HW version dependent on having an PX4IO
 * So we init to a benign state with the _INIT definition
 * and provide the the non _INIT one for the driver to make a run time
 * decision to use it.
 */
#define SAFETY_INIT_IOMUX (IOMUX_CMOS_INPUT  | IOMUX_PULL_NONE | IOMUX_DRIVE_HIZ)
#define SAFETY_IOMUX      (IOMUX_CMOS_OUTPUT | IOMUX_PULL_NONE | IOMUX_DRIVE_33OHM  | IOMUX_SPEED_MEDIUM | IOMUX_SLEW_SLOW)
#define SAFETY_SW_IOMUX   (IOMUX_CMOS_INPUT  | IOMUX_PULL_UP_22K | IOMUX_DRIVE_HIZ)

#define GPIO_SAFETY_SWITCH_IN              /* GPIO_B0_12 GPIO2_IO12 */ (GPIO_PORT2 | GPIO_PIN12 | GPIO_INPUT | SAFETY_SW_IOMUX)
/* Enable the FMU to use the switch it if there is no px4io fixme:This should be BOARD_SAFTY_BUTTON() */
#define GPIO_BTN_SAFETY GPIO_SAFETY_SWITCH_IN /* Enable the FMU to control it if there is no px4io */

#define SDIO_SLOTNO                    0  /* Only one slot */
#define SDIO_MINOR                     0

/* SD card bringup does not work if performed on the IDLE thread because it
 * will cause waiting.  Use either:
 *
 *  CONFIG_BOARDCTL=y, OR
 *  CONFIG_BOARD_INITIALIZE=y && CONFIG_BOARD_INITTHREAD=y
 */

#if defined(CONFIG_BOARD_INITIALIZE) && !defined(CONFIG_BOARDCTL) && \
   !defined(CONFIG_BOARD_INITTHREAD)
#  warning SDIO initialization cannot be perfomed on the IDLE thread
#endif

/* By Providing BOARD_ADC_USB_CONNECTED (using the px4_arch abstraction)
 * this board support the ADC system_power interface, and therefore
 * provides the true logic GPIO BOARD_ADC_xxxx macros.
 */

//#define BOARD_ADC_USB_VALID     (!px4_arch_gpioread(GPIO_nVDD_USB_VALID))
//#define BOARD_ADC_USB_CONNECTED (board_read_VBUS_state() == 0)

/* FMUv5 never powers odd the Servo rail */

#define BOARD_ADC_SERVO_VALID     (1)


/* This board provides a DMA pool and APIs */
#define BOARD_DMA_ALLOC_POOL_SIZE 5120

/* This board provides the board_on_reset interface */

// FIXME LETS NOT RESET FOR NOW
//#define BOARD_HAS_ON_RESET 1
#define BOARD_HAS_TEENSY_BOOTLOADER

#define PX4_GPIO_INIT_LIST { \
		PX4_ADC_GPIO,                     \
		GPIO_FLEXCAN3_TX,                 \
		GPIO_FLEXCAN3_RX,                 \
		GPIO_TONE_ALARM_IDLE,             \
		GPIO_SAFETY_SWITCH_IN             \
	}

#define BOARD_ENABLE_CONSOLE_BUFFER
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
 * Name: fmurt1062_usdhc_initialize
 *
 * Description:
 *   Initialize SDIO-based MMC/SD card support
 *
 ****************************************************************************/

int fmurt1062_usdhc_initialize(void);

/****************************************************************************************************
 * Name: imxrt_spidev_initialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the PX4FMU board.
 *
 ****************************************************************************************************/

extern void imxrt_spidev_initialize(void);

/************************************************************************************
 * Name: imxrt_spi_bus_initialize
 *
 * Description:
 *   Called to configure SPI Buses.
 *
 ************************************************************************************/

extern int imxrt1062_spi_bus_initialize(void);

/************************************************************************************
 * Name: imxrt_usb_initialize
 *
 * Description:
 *   Called to configure USB.
 *
 ************************************************************************************/

extern int imxrt_usb_initialize(void);

extern void imxrt_usbinitialize(void);

extern void board_peripheral_reset(int ms);

extern void fmurt1062_timer_initialize(void);

#include <px4_platform_common/board_common.h>

#endif /* __ASSEMBLY__ */

__END_DECLS
