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
 * PX4 fmu-v6xrt internal definitions
 */

#pragma once

/****************************************************************************************************
 * Included Files
 ****************************************************************************************************/

#include <nuttx/config.h>

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


/* PX4IO connection configuration */
// This requires serial DMA driver
#define BOARD_USES_PX4IO_VERSION       2
#define PX4IO_SERIAL_DEVICE            "/dev/ttyS4"
#define PX4IO_SERIAL_TX_GPIO           GPIO_LPUART6_TX
#define PX4IO_SERIAL_RX_GPIO           GPIO_LPUART6_RX
#define PX4IO_SERIAL_BASE              IMXRT_LPUART6_BASE
#define PX4IO_SERIAL_VECTOR            IMXRT_IRQ_LPUART6
#define PX4IO_SERIAL_TX_DMAMAP         IMXRT_DMACHAN_LPUART6_TX
#define PX4IO_SERIAL_RX_DMAMAP         IMXRT_DMACHAN_LPUART6_RX
#define PX4IO_SERIAL_CLOCK_OFF         imxrt_clockoff_lpuart6
#define PX4IO_SERIAL_BITRATE           1500000               /* 1.5Mbps -> max rate for IO */

/* Configuration ************************************************************************************/

/* Configuration ************************************************************************************/

#define BOARD_HAS_LTC44XX_VALIDS      2 //  N Bricks
#define BOARD_HAS_USB_VALID           1 // LTC Has USB valid
#define BOARD_HAS_NBAT_V              2d // 2 Digital Voltage
#define BOARD_HAS_NBAT_I              2d // 2 Digital Current


/* FMU-V6XRT GPIOs ***********************************************************************************/
/* LEDs */
/* An RGB LED is connected through GPIO as shown below:
 */

#define LED_IOMUX (IOMUX_OPENDRAIN | IOMUX_PULL_NONE)
#define GPIO_nLED_RED   /* GPIO_DISP_B2_00 GPIO5_IO01 */ (GPIO_PORT5 | GPIO_PIN1  | GPIO_OUTPUT | GPIO_OUTPUT_ZERO | LED_IOMUX)
#define GPIO_nLED_GREEN /* GPIO_DISP_B2_01 GPIO5_IO02 */ (GPIO_PORT5 | GPIO_PIN2  | GPIO_OUTPUT | GPIO_OUTPUT_ZERO | LED_IOMUX)
#define GPIO_nLED_BLUE  /* GPIO_EMC_B1_13  GPIO1_IO13 */ (GPIO_PORT1 | GPIO_PIN13 | GPIO_OUTPUT | GPIO_OUTPUT_ZERO | LED_IOMUX)

#define BOARD_HAS_CONTROL_STATUS_LEDS   1
#define BOARD_OVERLOAD_LED              LED_RED
#define BOARD_ARMED_STATE_LED           LED_BLUE

/* I2C busses */

/* Devices on the onboard buses.
 *
 * Note that these are unshifted addresses.
 */
#define BOARD_MTD_NUM_EEPROM        2 /* MTD: base_eeprom, imu_eeprom*/
#define PX4_I2C_OBDEV_SE050         0x48


/*
 * From the radion souce code
 * // Serial flow control
 * #define SERIAL_RTS PIN_ENABLE  // always an input
 * #define SERIAL_CTS PIN_CONFIG  // input in bootloader, output in app
 *
 * RTS is an out from FMU
 * CTS is in input to the FMU but the booloader on the radion will treat it as an input, and the
 * radion APP as output.
 *
 * To ensure radios do not go into bootloader mode because our CTS is configured with Pull downs
 * We init with pull ups, then enable power, then initalize the CTS will pull downs
 */

#define GPIO_LPUART4_CTS_INIT     PX4_MAKE_GPIO_PULLED_INPUT(GPIO_LPUART4_CTS, IOMUX_PULL_UP)
#define GPIO_LPUART8_CTS_INIT     PX4_MAKE_GPIO_PULLED_INPUT(GPIO_LPUART8_CTS, IOMUX_PULL_UP)
#define GPIO_LPUART10_CTS_INIT    PX4_MAKE_GPIO_PULLED_INPUT(GPIO_LPUART10_CTS,IOMUX_PULL_UP)

/*
 *  Define the ability to shut off off the sensor signals
 *  by changing the signals to inputs
 */

#define _PIN_OFF(def) (((def) & (GPIO_PORT_MASK | GPIO_PIN_MASK)) | (GPIO_INPUT | IOMUX_PULL_DOWN))

/*  Define the Chip Selects, Data Ready and Control signals per SPI bus */

#define CS_IOMUX  (IOMUX_CMOS_OUTPUT | IOMUX_SLEW_FAST)
#define OUT_IOMUX (IOMUX_CMOS_OUTPUT | IOMUX_SLEW_FAST)


/* SPI1 off */

#define _GPIO_LPSPI1_SCK   /* GPIO_EMC_B2_00  GPIO2_IO10 */  (GPIO_PORT2 | GPIO_PIN10 | CS_IOMUX)
#define _GPIO_LPSPI1_MISO  /* GPIO_EMC_B2_03  GPIO2_IO13 */  (GPIO_PORT2 | GPIO_PIN13 | CS_IOMUX)
#define _GPIO_LPSPI1_MOSI  /* GPIO_EMC_B2_02  GPIO2_IO12 */  (GPIO_PORT2 | GPIO_PIN12 | CS_IOMUX)

#define GPIO_SPI1_SCK_OFF   _PIN_OFF(_GPIO_LPSPI1_SCK)
#define GPIO_SPI1_MISO_OFF  _PIN_OFF(_GPIO_LPSPI1_MISO)
#define GPIO_SPI1_MOSI_OFF  _PIN_OFF(_GPIO_LPSPI1_MOSI)

/* SPI2 off */

#define _GPIO_LPSPI2_SCK   /* GPIO_AD_24  GPIO3_IO23 */  (GPIO_PORT3 | GPIO_PIN23 | CS_IOMUX)
#define _GPIO_LPSPI2_MISO  /* GPIO_AD_27  GPIO3_IO26 */  (GPIO_PORT3 | GPIO_PIN26 | CS_IOMUX)
#define _GPIO_LPSPI2_MOSI  /* GPIO_AD_26  GPIO3_IO25 */  (GPIO_PORT3 | GPIO_PIN25 | CS_IOMUX)

#define GPIO_SPI2_SCK_OFF   _PIN_OFF(_GPIO_LPSPI2_SCK)
#define GPIO_SPI2_MISO_OFF  _PIN_OFF(_GPIO_LPSPI2_MISO)
#define GPIO_SPI2_MOSI_OFF  _PIN_OFF(_GPIO_LPSPI2_MOSI)

/* SPI3 off */

#define _GPIO_LPSPI3_SCK   /* GPIO_EMC_B2_04 GPIO2_IO14 */  (GPIO_PORT2 | GPIO_PIN14 | CS_IOMUX)
#define _GPIO_LPSPI3_MISO  /* GPIO_EMC_B2_07 GPIO2_IO17 */  (GPIO_PORT2 | GPIO_PIN17 | CS_IOMUX)
#define _GPIO_LPSPI3_MOSI  /* GPIO_EMC_B2_06 GPIO2_IO16 */  (GPIO_PORT2 | GPIO_PIN16 | CS_IOMUX)

#define GPIO_SPI3_SCK_OFF   _PIN_OFF(_GPIO_LPSPI3_SCK)
#define GPIO_SPI3_MISO_OFF  _PIN_OFF(_GPIO_LPSPI3_MISO)
#define GPIO_SPI3_MOSI_OFF  _PIN_OFF(_GPIO_LPSPI3_MOSI)

/* SPI4 off */

#define _GPIO_LPSPI4_SCK   /* GPIO_DISP_B2_12 GPIO5_IO13 */  (GPIO_PORT5 | GPIO_PIN13 | CS_IOMUX)
#define _GPIO_LPSPI4_MISO  /* GPIO_DISP_B2_13 GPIO5_IO14 */  (GPIO_PORT5 | GPIO_PIN14 | CS_IOMUX)
#define _GPIO_LPSPI4_MOSI  /* GPIO_DISP_B2_14 GPIO5_IO15 */  (GPIO_PORT5 | GPIO_PIN15 | CS_IOMUX)

#define GPIO_SPI4_SCK_OFF   _PIN_OFF(_GPIO_LPSPI4_SCK)
#define GPIO_SPI4_MISO_OFF  _PIN_OFF(_GPIO_LPSPI4_MISO)
#define GPIO_SPI4_MOSI_OFF  _PIN_OFF(_GPIO_LPSPI4_MOSI)

/* SPI6 off */

#define _GPIO_LPSPI6_SCK   /* GPIO_LPSR_10 GPIO6_IO10 */  (GPIO_PORT6 | GPIO_PIN10 | CS_IOMUX)
#define _GPIO_LPSPI6_MISO  /* GPIO_LPSR_12 GPIO6_IO12 */  (GPIO_PORT6 | GPIO_PIN12 | CS_IOMUX)
#define _GPIO_LPSPI6_MOSI  /* GPIO_LPSR_11 GPIO6_IO11 */  (GPIO_PORT6 | GPIO_PIN11 | CS_IOMUX)

#define GPIO_SPI6_SCK_OFF   _PIN_OFF(_GPIO_LPSPI6_SCK)
#define GPIO_SPI6_MISO_OFF  _PIN_OFF(_GPIO_LPSPI6_MISO)
#define GPIO_SPI6_MOSI_OFF  _PIN_OFF(_GPIO_LPSPI6_MOSI)


/*  Define the SPI Data Ready and Control signals */
#define DRDY_IOMUX (IOMUX_PULL_UP)


/*  SPI1 */

#define GPIO_SPI1_DRDY1_SENSOR1   /* GPIO_AD_20      GPIO3_IO19 */ (GPIO_PORT3 | GPIO_PIN19  | GPIO_INPUT  | DRDY_IOMUX)
#define GPIO_SPI2_DRDY1_SENSOR2   /* GPIO_EMC_B1_39  GPIO2_IO07 */ (GPIO_PORT2 | GPIO_PIN07  | GPIO_INPUT  | DRDY_IOMUX)
#define GPIO_SPI3_DRDY1_SENSOR3   /* GPIO_AD_21      GPIO3_IO20 */ (GPIO_PORT3 | GPIO_PIN20  | GPIO_INPUT  | DRDY_IOMUX)
#define GPIO_SPI3_DRDY2_SENSOR3   /* GPIO_EMC_B2_09  GPIO2_IO19 */ (GPIO_PORT2 | GPIO_PIN19  | GPIO_INPUT  | DRDY_IOMUX)
#define GPIO_SPI4_DRDY1_SENSOR4   /* GPIO_EMC_B1_16  GPIO1_IO16 */ (GPIO_PORT1 | GPIO_PIN16  | GPIO_INPUT  | DRDY_IOMUX)
#define GPIO_SPI6_DRDY1_EXTERNAL1 /* GPIO_EMC_B1_05  GPIO1_IO05 */ (GPIO_PORT1 | GPIO_PIN05  | GPIO_INPUT  | DRDY_IOMUX)
#define GPIO_SPI6_DRDY2_EXTERNAL1 /* GPIO_EMC_B1_07  GPIO1_IO07 */ (GPIO_PORT1 | GPIO_PIN07  | GPIO_INPUT  | DRDY_IOMUX)


#define GPIO_SPI6_nRESET_EXTERNAL1  /* GPIO_EMC_B1_11 GPIO1_IO11 */ (GPIO_PORT1 | GPIO_PIN11 | GPIO_OUTPUT | GPIO_OUTPUT_ONE | OUT_IOMUX)
#define GPIO_SPIX_SYNC              /* GPIO_EMC_B1_18 GPIO1_IO18 */ (GPIO_PORT1 | GPIO_PIN18  | GPIO_OUTPUT | GPIO_OUTPUT_ONE | OUT_IOMUX)

#define GPIO_DRDY_OFF_SPI6_DRDY2_EXTERNAL1   _PIN_OFF(GPIO_SPI6_DRDY2_EXTERNAL1)
#define GPIO_SPI6_nRESET_EXTERNAL1_OFF       _PIN_OFF(GPIO_SPI6_nRESET_EXTERNAL1)
#define GPIO_SPIX_SYNC_OFF                   _PIN_OFF(GPIO_SPIX_SYNC)

#define ADC_IOMUX (IOMUX_PULL_NONE)

#define ADC1_CH(n)                  (n)

/* N.B. there is no offset mapping needed for ADC3 because we are only use ADC2 for REV/VER. */
#define ADC2_CH(n)                  (n)

#define ADC_GPIO(n, p)             (GPIO_PORT3 | GPIO_PIN##p | GPIO_INPUT | ADC_IOMUX) //

/* Define GPIO pins used as ADC
 * ADC1 has 12 inputs 0-5A and 0-5B
 *   We represent this as:
 *   0      ADC1 CH0A
 *   1      ADC1 CH0B
 *   ...
 *   10     ADC1 CH5A
 *   11     ADC1 CH5B
 *
 * ADC2 has 14 inputs 0-6A and 0-6B
 *
 *   0      ADC2 CH0A
 *   1      ADC2 CH0B
 *   ...
 *   12     ADC2 CH6A
 *   13     ADC2 CH6B
 *
 *
 *
 *  */

#define PX4_ADC_GPIO  \
	/* SCALED_VDD_3V3_SENSORS1 GPIO_AD_10 GPIO3 Pin 9  ADC1_CH2A */  ADC_GPIO(4,  9),  \
	/* SCALED_VDD_3V3_SENSORS2 GPIO_AD_11 GPIO3 Pin 10 ADC1_CH2B */  ADC_GPIO(5,  10), \
	/* SCALED_VDD_3V3_SENSORS3 GPIO_AD_12 GPIO3 Pin 11 ADC1_CH3A */  ADC_GPIO(6,  11), \
	/* SCALED_V5               GPIO_AD_13 GPIO3 Pin 12 ADC1_CH3B */  ADC_GPIO(7,  12), \
	/* ADC_6V6                 GPIO_AD_14 GPIO3 Pin 13 ADC1_CH4A */  ADC_GPIO(8,  13), \
	/* ADC_3V3                 GPIO_AD_16 GPIO3 Pin 15 ADC1_CH5A */  ADC_GPIO(10, 15), \
	/* SCALED_VDD_3V3_SENSORS4 GPIO_AD_17 GPIO3 Pin 16 ADC1_CH5B */  ADC_GPIO(11, 16),  \
	/* HW_VER_SENSE            GPIO_AD_22 GPIO3 Pin 21 ADC2_CH2A */  ADC_GPIO(4,  21), \
	/* HW_REV_SENSE            GPIO_AD_23 GPIO3 Pin 22 ADC2_CH2B */  ADC_GPIO(5,  22)

/* Define Channel numbers must match above GPIO pin IN(n)*/

#define ADC_SCALED_VDD_3V3_SENSORS1_CHANNEL /* GPIO_AD_10 GPIO3 Pin 9  ADC1_CH2A */  ADC1_CH(4)
#define ADC_SCALED_VDD_3V3_SENSORS2_CHANNEL /* GPIO_AD_11 GPIO3 Pin 10 ADC1_CH2B */  ADC1_CH(5)
#define ADC_SCALED_VDD_3V3_SENSORS3_CHANNEL /* GPIO_AD_12 GPIO3 Pin 11 ADC1_CH3A */  ADC1_CH(6)
#define ADC_SCALED_V5_CHANNEL               /* GPIO_AD_13 GPIO3 Pin 12 ADC1_CH3B */  ADC1_CH(7)
#define ADC_ADC_6V6_CHANNEL                 /* GPIO_AD_14 GPIO3 Pin 13 ADC1_CH4A */  ADC1_CH(8)
#define ADC_ADC_3V3_CHANNEL                 /* GPIO_AD_16 GPIO3 Pin 15 ADC1_CH5A */  ADC1_CH(10)
#define ADC_SCALED_VDD_3V3_SENSORS4_CHANNEL /* GPIO_AD_17 GPIO3 Pin 16 ADC1_CH5B */  ADC1_CH(11)
#define ADC_HW_VER_SENSE_CHANNEL            /* GPIO_AD_22 GPIO3 Pin 21 ADC2_CH2A */  ADC2_CH(4)
#define ADC_HW_REV_SENSE_CHANNEL            /* GPIO_AD_23 GPIO3 Pin 22 ADC2_CH2B */  ADC2_CH(5)

#define ADC_CHANNELS \
	((1 << ADC_SCALED_VDD_3V3_SENSORS1_CHANNEL)  | \
	 (1 << ADC_SCALED_VDD_3V3_SENSORS2_CHANNEL)  | \
	 (1 << ADC_SCALED_VDD_3V3_SENSORS3_CHANNEL)  | \
	 (1 << ADC_SCALED_V5_CHANNEL)                | \
	 (1 << ADC_ADC_6V6_CHANNEL)                  | \
	 (1 << ADC_ADC_3V3_CHANNEL)                  | \
	 (1 << ADC_SCALED_VDD_3V3_SENSORS4_CHANNEL))

// The ADC is used in SCALED mode.
// The V that is converted to a DN is 30/64 of Vin of the pin.
// The DN is therfore 30/64 of the real voltage

#define BOARD_ADC_POS_REF_V (1.825f * 64.0f / 30.0f)

#define HW_REV_VER_ADC_BASE IMXRT_LPADC2_BASE
#define SYSTEM_ADC_BASE     IMXRT_LPADC1_BASE

/* HW has to large of R termination on ADC todo:change when HW value is chosen */

#define BOARD_ADC_OPEN_CIRCUIT_V     (5.6f)

/* HW Version and Revision drive signals Default to 1 to detect */

#define BOARD_HAS_HW_SPLIT_VERSIONING

#define HW_IOMUX (IOMUX_CMOS_OUTPUT | IOMUX_PULL_NONE | IOMUX_SLEW_FAST)

#define GPIO_HW_VER_REV_DRIVE /* GPIO_GPIO_EMC_B1_26 GPIO1_IO26   */  (GPIO_PORT1 | GPIO_PIN26 | GPIO_OUTPUT | GPIO_OUTPUT_ONE | HW_IOMUX)
#define GPIO_HW_REV_SENSE     /* GPIO_AD_22 GPIO9 Pin 21 */  ADC_GPIO(4, 21)
#define GPIO_HW_VER_SENSE     /* GPIO_AD_23 GPIO9 Pin 22 */  ADC_GPIO(5, 22)
#define HW_INFO_INIT_PREFIX   "V6XRT"

#define BOARD_NUM_SPI_CFG_HW_VERSIONS 2 // Rev 0 & 1
#define V6XRT_0             HW_FMUM_ID(0x0)  // First Release
#define V6XRT_1             HW_FMUM_ID(0x1)  // Next Release

#define BOARD_I2C_LATEINIT 1 /* See Note about SE550 Eanable */

/* HEATER
 * PWM in future
 */
#define HEATER_IOMUX (IOMUX_CMOS_OUTPUT | IOMUX_PULL_NONE | IOMUX_SLEW_FAST)
//#define GPIO_HEATER_OUTPUT   /* GPIO_EMC_B2_17 QTIMER3 TIMER0 GPIO2_IO27 */ (GPIO_QTIMER3_TIMER0_3 | HEATER_IOMUX)
#define GPIO_HEATER_OUTPUT     /* GPIO_EMC_B2_17 GPIO2_IO27 */ (GPIO_PORT2 | GPIO_PIN27 | GPIO_OUTPUT | HEATER_IOMUX)
#define HEATER_OUTPUT_EN(on_true) px4_arch_gpiowrite(GPIO_HEATER_OUTPUT, (on_true))

/* nARMED GPIO1_IO17
 *  The GPIO will be set as input while not armed HW will have external HW Pull UP.
 *  While armed it shall be configured at a GPIO OUT set LOW
 */
#define nARMED_INPUT_IOMUX  (IOMUX_PULL_UP)
#define nARMED_OUTPUT_IOMUX (IOMUX_PULL_KEEP | IOMUX_SLEW_FAST)

#define GPIO_nARMED_INIT     /* GPIO1_IO17 */ (GPIO_PORT1 | GPIO_PIN17 | GPIO_INPUT | nARMED_INPUT_IOMUX)
#define GPIO_nARMED          /* GPIO1_IO17 */ (GPIO_PORT1 | GPIO_PIN17 | GPIO_OUTPUT | GPIO_OUTPUT_ZERO | nARMED_OUTPUT_IOMUX)

#define BOARD_INDICATE_EXTERNAL_LOCKOUT_STATE(enabled)  px4_arch_configgpio((enabled) ? GPIO_nARMED : GPIO_nARMED_INIT)
#define BOARD_GET_EXTERNAL_LOCKOUT_STATE() px4_arch_gpioread(GPIO_nARMED)

/* PWM Capture
 *
 * 2  PWM Capture inputs are supported
 */
#define DIRECT_PWM_CAPTURE_CHANNELS  1
#define CAP_IOMUX (IOMUX_PULL_NONE | IOMUX_SLEW_FAST)
#define GPIO_FMU_CAP1 /* GPIO_EMC_B1_20 TMR4_TIMER0 */  (GPIO_QTIMER4_TIMER0_1 | CAP_IOMUX)

/* PWM
 */

#define DIRECT_PWM_OUTPUT_CHANNELS  12
#define BOARD_NUM_IO_TIMERS         12

// Input Capture not supported on MVP

#define BOARD_HAS_NO_CAPTURE

/* Power supply control and monitoring GPIOs */

#define GENERAL_INPUT_IOMUX  (IOMUX_PULL_UP)
#define GENERAL_OUTPUT_IOMUX (IOMUX_PULL_KEEP | IOMUX_SLEW_FAST)

#define GPIO_nPOWER_IN_A                /* GPIO_EMC_B1_28  GPIO1_IO28 */ (GPIO_PORT1 | GPIO_PIN28 | GPIO_INPUT | GENERAL_INPUT_IOMUX)
#define GPIO_nPOWER_IN_B                /* GPIO_EMC_B1_30  GPIO1_IO30 */ (GPIO_PORT1 | GPIO_PIN30 | GPIO_INPUT | GENERAL_INPUT_IOMUX)
#define GPIO_nPOWER_IN_C                /* GPIO_EMC_B1_32  GPIO2_IO00 */ (GPIO_PORT2 | GPIO_PIN0  | GPIO_INPUT | GENERAL_INPUT_IOMUX)


#define GPIO_nVDD_BRICK1_VALID          GPIO_nPOWER_IN_A /* Brick 1 Is Chosen */
#define GPIO_nVDD_BRICK2_VALID          GPIO_nPOWER_IN_B /* Brick 2 Is Chosen */
#define BOARD_NUMBER_BRICKS             2
#define BOARD_NUMBER_DIGITAL_BRICKS     2
#define GPIO_nVDD_USB_VALID             GPIO_nPOWER_IN_C /* USB     Is Chosen */

#define OC_INPUT_IOMUX  (IOMUX_PULL_NONE)

#define GPIO_VDD_5V_PERIPH_nEN          /* GPIO_EMC_B1_34 GPIO2_IO02 */ (GPIO_PORT2 | GPIO_PIN2 | GPIO_OUTPUT | GPIO_OUTPUT_ONE | GENERAL_OUTPUT_IOMUX)
#define GPIO_VDD_5V_PERIPH_nOC          /* GPIO_EMC_B1_15 GPIO1_IO15 */ (GPIO_PORT1 | GPIO_PIN15 | GPIO_INPUT  | OC_INPUT_IOMUX)
#define GPIO_VDD_5V_HIPOWER_nEN         /* GPIO_EMC_B1_37 GPIO2_IO05 */ (GPIO_PORT2 | GPIO_PIN5  | GPIO_OUTPUT | GPIO_OUTPUT_ONE | GENERAL_OUTPUT_IOMUX)
#define GPIO_VDD_5V_HIPOWER_nOC         /* GPIO_EMC_B1_12 GPIO1_IO12 */ (GPIO_PORT1 | GPIO_PIN12 | GPIO_INPUT  | OC_INPUT_IOMUX)
#define GPIO_VDD_3V3_SENSORS1_EN        /* GPIO_EMC_B1_33 GPIO2_IO01 */ (GPIO_PORT2 | GPIO_PIN1  | GPIO_OUTPUT | GPIO_OUTPUT_ZERO | GENERAL_OUTPUT_IOMUX)
#define GPIO_VDD_3V3_SENSORS2_EN        /* GPIO_EMC_B1_22 GPIO1_IO22 */ (GPIO_PORT1 | GPIO_PIN22 | GPIO_OUTPUT | GPIO_OUTPUT_ZERO | GENERAL_OUTPUT_IOMUX)
#define GPIO_VDD_3V3_SENSORS3_EN        /* GPIO_EMC_B1_14 GPIO1_IO14 */ (GPIO_PORT1 | GPIO_PIN14 | GPIO_OUTPUT | GPIO_OUTPUT_ZERO | GENERAL_OUTPUT_IOMUX)
#define GPIO_VDD_3V3_SENSORS4_EN        /* GPIO_EMC_B1_36 GPIO2_IO04 */ (GPIO_PORT2 | GPIO_PIN4  | GPIO_OUTPUT | GPIO_OUTPUT_ZERO | GENERAL_OUTPUT_IOMUX)

#define GPIO_VDD_3V3_SPEKTRUM_POWER_EN  /* GPIO_EMC_B1_38 GPIO2_IO06 */ (GPIO_PORT2 | GPIO_PIN6  | GPIO_OUTPUT | GPIO_OUTPUT_ZERO | GENERAL_OUTPUT_IOMUX)
#define GPIO_VDD_3V3_SD_CARD_EN         /* GPIO_EMC_B1_01 GPIO1_IO1  */ (GPIO_PORT1 | GPIO_PIN1  | GPIO_OUTPUT | GPIO_OUTPUT_ZERO |GENERAL_OUTPUT_IOMUX)

/* ETHERNET GPIO */

#define GPIO_ETH_POWER_EN              /* GPIO_DISP_B2_08 GPIO5_IO09 */ (GPIO_PORT5 | GPIO_PIN9  | GPIO_OUTPUT | GPIO_OUTPUT_ZERO | GENERAL_OUTPUT_IOMUX)

#define GPIO_ETH_PHY_nINT              /* GPIO_DISP_B2_09 GPIO5_IO10 */ (GPIO_PORT5 | GPIO_PIN10  | GPIO_INPUT |  GENERAL_INPUT_IOMUX)

#define GPIO_ENET2_RX_ER_CONFIG1       /* GPIO_DISP_B1_01 GPIO4_IO22 PHYAD18        Open */  (GPIO_PORT4 | GPIO_PIN22 | GPIO_INPUT | OC_INPUT_IOMUX | IOMUX_PULL_NONE)
#define GPIO_ENET2_RX_DATA01_CONFIG4   /* GPIO_EMC_B2_16  GPIO2_IO26 (RMII-Rev)     Low  */  (GPIO_PORT2 | GPIO_PIN26 | GPIO_OUTPUT | GPIO_OUTPUT_ZERO | GENERAL_OUTPUT_IOMUX)
#define GPIO_ENET2_RX_DATA00_CONFIG5   /* GPIO_EMC_B2_15  GPIO2_IO25 SLAVE:Auto     Open */  (GPIO_PORT2 | GPIO_PIN25 | GPIO_INPUT  | OC_INPUT_IOMUX | IOMUX_PULL_NONE)
#define GPIO_ENET2_CRS_DV_CONFIG6      /* GPIO_DISP_B1_00 GPIO4_IO21 SLAVE:POl Corr Low  */  (GPIO_PORT4 | GPIO_PIN21 | GPIO_OUTPUT | GPIO_OUTPUT_ZERO | GENERAL_OUTPUT_IOMUX)


/* NFC GPIO */

#define GPIO_NFC_GPIO                  /* GPIO_EMC_B1_04 GPIO1_IO04 */ (GPIO_PORT1 | GPIO_PIN4  | GPIO_INPUT |  GENERAL_INPUT_IOMUX)

#define GPIO_GPIO_EMC_B2_12           /* GPIO_EMC_B2_12 AKA PD15, PH11 */  (GPIO_PORT2 | GPIO_PIN22 | GPIO_OUTPUT | GPIO_OUTPUT_ZERO | OUT_IOMUX)


/* 10/100 Mbps Ethernet & Gigabit Ethernet */

/* 10/100 Mbps Ethernet Interrupt: GPIO_AD_12
 * Gigabit Ethernet Interrupt: GPIO_DISP_B2_12
 *
 * This pin has a week pull-up within the PHY, is open-drain, and requires
 * an external 1k ohm pull-up resistor (present on the EVK).  A falling
 * edge then indicates a change in state of the PHY.
 */

#define GPIO_ENET_INT  (IOMUX_ENET_INT_DEFAULT | GPIO_OUTPUT | GPIO_PORT3 | GPIO_PIN11)  /* GPIO_AD_12 */
#define GPIO_ENET_IRQ  IMXRT_IRQ_GPIO3_0_15

#define GPIO_ENET1G_INT (IOMUX_ENET_INT_DEFAULT | GPIO_PORT5 | GPIO_PIN13)  /* GPIO_DISP_B2_12 */
#define GPIO_ENET1G_IRQ IMXRT_IRQ_GPIO5_13

/* 10/100 Mbps Ethernet Reset:  GPIO_LPSR_12
 * Gigabit Ethernet Reset: GPIO_DISP_B2_13
 *
 * The #RST uses inverted logic.  The initial value of zero will put the
 * PHY into the reset state.
 */

#define GPIO_ENET_RST   (GPIO_OUTPUT | GPIO_OUTPUT_ZERO | GPIO_PORT6 | GPIO_PIN12 | IOMUX_ENET_RST_DEFAULT)  /* GPIO_LPSR_12 */

#define GPIO_ENET1G_RST (GPIO_OUTPUT | GPIO_OUTPUT_ZERO | GPIO_PORT5 | GPIO_PIN14 | IOMUX_ENET_RST_DEFAULT)  /* GPIO_DISP_B2_13 */


/* Define True logic Power Control in arch agnostic form */

#define VDD_5V_PERIPH_EN(on_true)          px4_arch_gpiowrite(GPIO_VDD_5V_PERIPH_nEN, !(on_true))
#define VDD_5V_HIPOWER_EN(on_true)         px4_arch_gpiowrite(GPIO_VDD_5V_HIPOWER_nEN, !(on_true))
#define VDD_3V3_SENSORS4_EN(on_true)       px4_arch_gpiowrite(GPIO_VDD_3V3_SENSORS4_EN, (on_true))
#define VDD_3V3_SPEKTRUM_POWER_EN(on_true) px4_arch_gpiowrite(GPIO_VDD_3V3_SPEKTRUM_POWER_EN, (on_true))
#define READ_VDD_3V3_SPEKTRUM_POWER_EN()   px4_arch_gpioread(GPIO_VDD_3V3_SPEKTRUM_POWER_EN)
#define VDD_3V3_SD_CARD_EN(on_true)        px4_arch_gpiowrite(GPIO_VDD_3V3_SD_CARD_EN, (on_true))
#define VDD_3V3_ETH_POWER_EN(on_true)      px4_arch_gpiowrite(GPIO_ETH_POWER_EN, (on_true))

/* Tone alarm output */

#define TONE_ALARM_TIMER        3  /* GPT 3 */
#define TONE_ALARM_CHANNEL      2  /* GPIO_EMC_B2_09 GPT3_COMPARE2 */

#define GPIO_BUZZER_1           /* GPIO_EMC_B2_09  GPIO2_IO19  */ (GPIO_PORT2 | GPIO_PIN19  | GPIO_OUTPUT | GPIO_OUTPUT_ZERO | GENERAL_OUTPUT_IOMUX)

#define GPIO_TONE_ALARM_IDLE    GPIO_BUZZER_1
#define GPIO_TONE_ALARM         (GPIO_GPT3_COMPARE2_1 | GENERAL_OUTPUT_IOMUX)

/* USB OTG FS
 *
 * VBUS_VALID is detected in USB_ANALOG_USB1_VBUS_DETECT_STAT
 */

/* High-resolution timer */
#define HRT_TIMER               5  /* use GPT5 for the HRT */
#define HRT_TIMER_CHANNEL       2  /* use capture/compare channel 1 */

#define HRT_PPM_CHANNEL         /* GPIO_EMC_B1_09 GPIO_GPT5_CAPTURE1_1 */  1  /* use capture/compare channel 1 */
#define GPIO_PPM_IN             /* GPIO_EMC_B1_09 GPT1_CAPTURE2 */ (GPIO_GPT5_CAPTURE1_1 | GENERAL_INPUT_IOMUX)

#define RC_SERIAL_PORT                  "/dev/ttyS4"
#define RC_SERIAL_SINGLEWIRE            1 // Suport Single wire wiring
#define RC_SERIAL_SWAP_RXTX             1 // Set Swap (but not supported in HW) to use Single wire
#define RC_SERIAL_SWAP_USING_SINGLEWIRE 1 // Set to use Single wire swap as HW does not support swap
#define BOARD_SUPPORTS_RC_SERIAL_PORT_OUTPUT

/* FLEXSPI4 */

#define GPIO_FLEXSPI2_CS      (GPIO_FLEXSPI2_A_SS0_B_1|IOMUX_FLEXSPI_DEFAULT)
#define GPIO_FLEXSPI2_IO0     (GPIO_FLEXSPI2_A_DATA0_1|IOMUX_FLEXSPI_DEFAULT) /* SOUT */
#define GPIO_FLEXSPI2_IO1     (GPIO_FLEXSPI2_A_DATA1_1|IOMUX_FLEXSPI_DEFAULT) /* SIN */
#define GPIO_FLEXSPI2_SCK     (GPIO_FLEXSPI2_A_SCLK_1|IOMUX_FLEXSPI_CLK_DEFAULT)

/* PWM input driver. Use FMU AUX5 pins attached to GPIO_EMC_B1_08 GPIO1_IO8 FLEXPWM2_PWM1_A */

#define PWMIN_TIMER            /* FLEXPWM2_PWM1_A */  2
#define PWMIN_TIMER_CHANNEL    /* FLEXPWM2_PWM1_A */  1
#define GPIO_PWM_IN            /* GPIO_EMC_B1_08 GPIO1_IO8 */ (GPIO_FLEXPWM3_PWMA02_1 | GENERAL_INPUT_IOMUX)

/* Safety Switch is HW version dependent on having an PX4IO
 * So we init to a benign state with the _INIT definition
 * and provide the the non _INIT one for the driver to make a run time
 * decision to use it.
 */
#define SAFETY_INIT_IOMUX (IOMUX_PULL_NONE )
#define SAFETY_IOMUX      ( IOMUX_PULL_NONE | IOMUX_SLEW_SLOW)
#define SAFETY_SW_IOMUX   ( IOMUX_PULL_UP )

#define GPIO_nSAFETY_SWITCH_LED_OUT_INIT   /* GPIO_EMC_B1_03 GPIO1_IO03 */ (GPIO_PORT1 | GPIO_PIN3 | GPIO_INPUT  | SAFETY_INIT_IOMUX)
#define GPIO_nSAFETY_SWITCH_LED_OUT        /* GPIO_EMC_B1_03 GPIO1_IO03 */ (GPIO_PORT1 | GPIO_PIN3 | GPIO_OUTPUT | GPIO_OUTPUT_ONE | SAFETY_IOMUX)

/* Enable the FMU to control it if there is no px4io fixme:This should be BOARD_SAFETY_LED(__ontrue) */
#define GPIO_LED_SAFETY GPIO_nSAFETY_SWITCH_LED_OUT
#define GPIO_SAFETY_SWITCH_IN              /* GPIO_EMC_B1_24 GPIO1_IO24 */ (GPIO_PORT1 | GPIO_PIN24 | GPIO_INPUT | SAFETY_SW_IOMUX)
/* Enable the FMU to use the switch it if there is no px4io fixme:This should be BOARD_SAFTY_BUTTON() */
#define GPIO_BTN_SAFETY GPIO_SAFETY_SWITCH_IN /* Enable the FMU to control it if there is no px4io */


/* Power switch controls ******************************************************/

#define SPEKTRUM_POWER(_on_true)           VDD_3V3_SPEKTRUM_POWER_EN(_on_true)
/*
 * FMU-V6RT has a separate RC_IN and PPM
 *
 * GPIO PPM_IN on GPIO_EMC_B1_09 GPIO1 Pin 9 GPT5_CAPTURE1
 * SPEKTRUM_RX (it's TX or RX in Bind) on TX UART6_TX_TO_IO__RC_INPUT GPIO_EMC_B1_40 GPIO2 Pin 8
 *   Inversion is possible in the UART and can drive GPIO PPM_IN as an output
 */

#define GPIO_UART_AS_OUT             /* GPIO_EMC_B1_40 GPIO2_IO8 */ (GPIO_PORT2 | GPIO_PIN8 | GPIO_OUTPUT | GPIO_OUTPUT_ONE | GENERAL_OUTPUT_IOMUX)
#define SPEKTRUM_RX_AS_GPIO_OUTPUT()   px4_arch_configgpio(GPIO_UART_AS_OUT)
#define SPEKTRUM_RX_AS_UART()          px4_arch_configgpio(GPIO_LPUART6_TX_1)
#define SPEKTRUM_OUT(_one_true)        px4_arch_gpiowrite(GPIO_UART_AS_OUT, (_one_true))


#define SDIO_SLOTNO                    0  /* Only one slot */
#define SDIO_MINOR                     0

/* SD card bringup does not work if performed on the IDLE thread because it
 * will cause waiting.  Use either:
 *
 *  CONFIG_BOARDCTL=y, OR
 *  CONFIG_BOARD_INITIALIZE=y && CONFIG_BOARD_INITTHREAD=y
 */

#if defined(CONFIG_BOARD_INITIALIZE) && !defined(CONFIG_LIB_BOARDCTL) && \
   !defined(CONFIG_BOARD_INITTHREAD)
#  warning SDIO initialization cannot be perfomed on the IDLE thread
#endif

/* By Providing BOARD_ADC_USB_CONNECTED (using the px4_arch abstraction)
 * this board support the ADC system_power interface, and therefore
 * provides the true logic GPIO BOARD_ADC_xxxx macros.
 */

#define BOARD_ADC_USB_VALID     (!px4_arch_gpioread(GPIO_nVDD_USB_VALID))
#define BOARD_ADC_USB_CONNECTED (board_read_VBUS_state() == 0)

/* FMUv5 never powers odd the Servo rail */

#define BOARD_ADC_SERVO_VALID     (1)

#define BOARD_ADC_BRICK1_VALID  (!px4_arch_gpioread(GPIO_nVDD_BRICK1_VALID))
#define BOARD_ADC_BRICK2_VALID  (!px4_arch_gpioread(GPIO_nVDD_BRICK2_VALID))

#define BOARD_ADC_PERIPH_5V_OC  (!px4_arch_gpioread(GPIO_VDD_5V_PERIPH_nOC))
#define BOARD_ADC_HIPOWER_5V_OC (!px4_arch_gpioread(GPIO_VDD_5V_HIPOWER_nOC))


/* This board provides a DMA pool and APIs */
#define BOARD_DMA_ALLOC_POOL_SIZE 5120

/* This board provides the board_on_reset interface */

#define BOARD_HAS_ISP_BOOTLOADER 1

#define BOARD_HAS_ON_RESET 1

#define PX4_GPIO_INIT_LIST { \
		PX4_ADC_GPIO,                     \
		PX4_MAKE_GPIO_OUTPUT_CLEAR(GPIO_LPI2C1_SCL_RESET), \
		PX4_MAKE_GPIO_OUTPUT_CLEAR(GPIO_LPI2C1_SDA_RESET), \
		PX4_MAKE_GPIO_OUTPUT_CLEAR(GPIO_LPI2C2_SCL_RESET), \
		PX4_MAKE_GPIO_OUTPUT_CLEAR(GPIO_LPI2C2_SDA_RESET), \
		PX4_MAKE_GPIO_OUTPUT_CLEAR(GPIO_LPI2C3_SCL_RESET), \
		PX4_MAKE_GPIO_OUTPUT_CLEAR(GPIO_LPI2C3_SDA_RESET), \
		PX4_MAKE_GPIO_OUTPUT_CLEAR(GPIO_LPI2C6_SCL_RESET), \
		PX4_MAKE_GPIO_OUTPUT_CLEAR(GPIO_LPI2C6_SDA_RESET), \
		GPIO_LPUART4_CTS_INIT ,           \
		GPIO_LPUART8_CTS_INIT ,           \
		GPIO_LPUART10_CTS_INIT,           \
		GPIO_nLED_RED,                    \
		GPIO_nLED_GREEN,                  \
		GPIO_nLED_BLUE,                   \
		GPIO_BUZZER_1,                    \
		GPIO_HW_VER_REV_DRIVE,            \
		GPIO_FLEXCAN1_TX,                 \
		GPIO_FLEXCAN1_RX,                 \
		GPIO_FLEXCAN2_TX,                 \
		GPIO_FLEXCAN2_RX,                 \
		GPIO_FLEXCAN3_TX,                 \
		GPIO_FLEXCAN3_RX,                 \
		GPIO_HEATER_OUTPUT,               \
		GPIO_FMU_CAP1,                    \
		GPIO_nPOWER_IN_A,                 \
		GPIO_nPOWER_IN_B,                 \
		GPIO_nPOWER_IN_C,                 \
		GPIO_VDD_5V_PERIPH_nEN,           \
		GPIO_VDD_5V_PERIPH_nOC,           \
		GPIO_VDD_5V_HIPOWER_nEN,          \
		GPIO_VDD_5V_HIPOWER_nOC,          \
		GPIO_VDD_3V3_SENSORS1_EN,         \
		GPIO_VDD_3V3_SENSORS2_EN,         \
		GPIO_VDD_3V3_SENSORS3_EN,         \
		GPIO_VDD_3V3_SENSORS4_EN,         \
		GPIO_VDD_3V3_SENSORS4_EN,         \
		GPIO_VDD_3V3_SPEKTRUM_POWER_EN,   \
		GPIO_VDD_3V3_SD_CARD_EN,          \
		GPIO_SPIX_SYNC,                   \
		GPIO_SPI6_nRESET_EXTERNAL1,       \
		GPIO_ETH_POWER_EN,                \
		GPIO_ETH_PHY_nINT,                \
		GPIO_GPIO_EMC_B2_12,              \
		GPIO_NFC_GPIO,                    \
		GPIO_TONE_ALARM_IDLE,             \
		GPIO_nSAFETY_SWITCH_LED_OUT_INIT, \
		GPIO_SAFETY_SWITCH_IN,            \
		GPIO_PPM_IN,                      \
		GPIO_nARMED_INIT,                 \
		GPIO_ENET2_RX_ER_CONFIG1,         \
		GPIO_ENET2_RX_DATA01_CONFIG4,     \
		GPIO_ENET2_RX_DATA00_CONFIG5,     \
		GPIO_ENET2_CRS_DV_CONFIG6,        \
	}

#define BOARD_ENABLE_CONSOLE_BUFFER
#define PX4_I2C_BUS_MTD      1

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
 * Name: fmuv6xrt_usdhc_initialize
 *
 * Description:
 *   Initialize SDIO-based MMC/SD card support
 *
 ****************************************************************************/

int fmuv6xrt_usdhc_initialize(void);

/************************************************************************************
 * Name: imxrt_usb_initialize
 *
 * Description:
 *   Called to configure USB.
 *
 ************************************************************************************/

extern int imxrt_usb_initialize(void);

/****************************************************************************************************
 * Name: nxp_spiinitialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the PX4FMU board.
 *
 ****************************************************************************************************/

extern void imxrt_spiinitialize(void);


extern void imxrt_usbinitialize(void);

extern void board_peripheral_reset(int ms);

extern void fmuv6xrt_timer_initialize(void);

#include <px4_platform_common/board_common.h>

int imxrt_flexspi_fram_initialize(void);

#endif /* __ASSEMBLY__ */

__END_DECLS
