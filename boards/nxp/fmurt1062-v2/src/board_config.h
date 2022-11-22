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
 * NXP fmukrt1062-v1 internal definitions
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

/* PX4IO connection configuration */
// This requires serial DMA driver
#define BOARD_USES_PX4IO_VERSION       2
#define PX4IO_SERIAL_DEVICE            "/dev/ttyS5"
#define PX4IO_SERIAL_TX_GPIO           GPIO_LPUART6_TX
#define PX4IO_SERIAL_RX_GPIO           GPIO_LPUART6_RX
#define PX4IO_SERIAL_BASE              IMXRT_LPUART6_BASE
#define PX4IO_SERIAL_VECTOR            IMXRT_IRQ_LPUART6
#define PX4IO_SERIAL_TX_DMAMAP         IMXRT_DMACHAN_LPUART6_TX
#define PX4IO_SERIAL_RX_DMAMAP         IMXRT_DMACHAN_LPUART6_RX
#define PX4IO_SERIAL_CLOCK_OFF         imxrt_clockoff_lpuart6
#define PX4IO_SERIAL_BITRATE           1500000               /* 1.5Mbps -> max rate for IO */

/* Configuration ************************************************************************************/

#define BOARD_HAS_LTC44XX_VALIDS      2 //  N Bricks
#define BOARD_HAS_USB_VALID           1 // LTC Has USB valid
#define BOARD_HAS_NBAT_V              2d // 2 Digital Voltage
#define BOARD_HAS_NBAT_I              2d // 2 Digital Current

/* FMURT1062 GPIOs ***********************************************************************************/
/* LEDs */
/* An RGB LED is connected through GPIO as shown below:
 */
#define LED_IOMUX (IOMUX_OPENDRAIN | IOMUX_PULL_NONE | IOMUX_DRIVE_33OHM | IOMUX_SPEED_MEDIUM | IOMUX_SLEW_SLOW)
#define GPIO_nLED_RED   /* GPIO_B0_10 QTIMER4_TIMER1 GPIO2_IO10  */ (GPIO_PORT2 | GPIO_PIN10  | GPIO_OUTPUT | GPIO_OUTPUT_ZERO | LED_IOMUX)
#define GPIO_nLED_GREEN /* GPIO_B0_11 QTIMER4_TIMER2 GPIO2_IO11  */ (GPIO_PORT2 | GPIO_PIN11  | GPIO_OUTPUT | GPIO_OUTPUT_ZERO | LED_IOMUX)
#define GPIO_nLED_BLUE  /* GPIO_B1_11 QTIMER4_TIMER3 GPIO2_IO27  */ (GPIO_PORT2 | GPIO_PIN27  | GPIO_OUTPUT | GPIO_OUTPUT_ZERO | LED_IOMUX)

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
 *  Define the ability to shut off off the sensor signals
 *  by changing the signals to inputs
 */

#define _PIN_OFF(def) (((def) & (GPIO_PORT_MASK | GPIO_PIN_MASK)) | (GPIO_INPUT | IOMUX_PULL_DOWN_100K | IOMUX_CMOS_INPUT))

/*  Define the Chip Selects, Data Ready and Control signals per SPI bus */

#define CS_IOMUX  (IOMUX_CMOS_OUTPUT | IOMUX_PULL_UP_100K | IOMUX_DRIVE_33OHM | IOMUX_SPEED_LOW | IOMUX_SLEW_FAST)
#define OUT_IOMUX (IOMUX_CMOS_OUTPUT | IOMUX_PULL_UP_100K | IOMUX_DRIVE_50OHM | IOMUX_SPEED_MEDIUM | IOMUX_SLEW_FAST)


/* SPI off
 *
 * SPI1, SPI2 and SPI3 are all on VDD_3V3_SENSORS1 power domain.
 * As is I2C2, I2C4 onboard sensors
 * SPI4 is connected to FRAM and External1 connector.
 * Power domains are FMU3V3 and VDD_5V_PERIPH
 */

#define _GPIO_LPSPI1_MISO  /* GPIO_EMC_29  GPIO4_IO29 */  (GPIO_PORT4 | GPIO_PIN29 | CS_IOMUX)
#define _GPIO_LPSPI1_MOSI  /* GPIO_EMC_28  GPIO4_IO28 */  (GPIO_PORT4 | GPIO_PIN28 | CS_IOMUX)
#define _GPIO_LPSPI1_SCK   /* GPIO_EMC_27  GPIO4_IO27 */  (GPIO_PORT4 | GPIO_PIN27 | CS_IOMUX)

#define GPIO_SPI1_MISO_OFF  _PIN_OFF(_GPIO_LPSPI1_MISO)
#define GPIO_SPI1_MOSI_OFF  _PIN_OFF(_GPIO_LPSPI1_MOSI)
#define GPIO_SPI1_SCK_OFF   _PIN_OFF(_GPIO_LPSPI1_SCK)

#define _GPIO_LPSPI2_MISO  /* GPIO_EMC_03  GPIO4_IO3  */  (GPIO_PORT4 | GPIO_PIN3 | CS_IOMUX)
#define _GPIO_LPSPI2_MOSI  /* GPIO_EMC_02  GPIO4_IO2  */  (GPIO_PORT4 | GPIO_PIN2 | CS_IOMUX)
#define _GPIO_LPSPI2_SCK   /* GPIO_EMC_00  GPIO4_IO0  */  (GPIO_PORT4 | GPIO_PIN0 | CS_IOMUX)

#define GPIO_SPI2_MISO_OFF  _PIN_OFF(_GPIO_LPSPI2_MISO)
#define GPIO_SPI2_MOSI_OFF  _PIN_OFF(_GPIO_LPSPI2_MOSI)
#define GPIO_SPI2_SCK_OFF   _PIN_OFF(_GPIO_LPSPI2_SCK)

#define _GPIO_LPSPI3_MISO  /* GPIO_AD_B1_13 GPIO1_IO29 */  (GPIO_PORT1 | GPIO_PIN29 | CS_IOMUX)
#define _GPIO_LPSPI3_MOSI  /* GPIO_AD_B1_14 GPIO1_IO30 */  (GPIO_PORT1 | GPIO_PIN30 | CS_IOMUX)
#define _GPIO_LPSPI3_SCK   /* GPIO_AD_B1_15 GPIO1_IO31 */  (GPIO_PORT1 | GPIO_PIN31 | CS_IOMUX)

#define GPIO_SPI3_SCK_OFF   _PIN_OFF(_GPIO_LPSPI3_SCK)
#define GPIO_SPI3_MISO_OFF  _PIN_OFF(_GPIO_LPSPI3_MISO)
#define GPIO_SPI3_MOSI_OFF  _PIN_OFF(_GPIO_LPSPI3_MOSI)

/*  Define the SPI4 Data Ready and Control signals */
#define DRDY_IOMUX (IOMUX_SCHMITT_TRIGGER | IOMUX_PULL_UP_47K | IOMUX_DRIVE_HIZ)

#define GPIO_SPI4_DRDY1_EXTERNAL1   /* GPIO_EMC_35 GPIO3_IO21 */ (GPIO_PORT3 | GPIO_PIN21  | GPIO_INPUT  | DRDY_IOMUX)
#define GPIO_SPI4_nRESET_EXTERNAL1  /* GPIO_EMC_11 GPIO4_IO11 */ (GPIO_PORT4 | GPIO_PIN11 | GPIO_OUTPUT | GPIO_OUTPUT_ONE | OUT_IOMUX)
#define GPIO_SYNC                   /* GPIO_EMC_05 GPIO4_IO5  */ (GPIO_PORT4 | GPIO_PIN5  | GPIO_OUTPUT | GPIO_OUTPUT_ONE | OUT_IOMUX)

#define GPIO_DRDY_OFF_SPI4_DRDY1_EXTERNAL1   _PIN_OFF(GPIO_SPI4_DRDY1_EXTERNAL1)
#define GPIO_SPI4_nRESET_EXTERNAL1_OFF       _PIN_OFF(GPIO_SPI4_nRESET_EXTERNAL1)
#define GPIO_SYNC_OFF                        _PIN_OFF(GPIO_SYNC)


#define ADC_IOMUX (IOMUX_CMOS_INPUT | IOMUX_PULL_NONE | IOMUX_DRIVE_HIZ)

#define ADC1_CH(n)                  (n)
#define ADC1_GPIO(n, p)             (GPIO_PORT1 | GPIO_PIN##p | ADC_IOMUX) //

#define PX4_ADC_GPIO  \
	/* ADC1_3V3                GPIO_AD_B1_09 GPIO1 Pin 25 */  ADC1_GPIO(14, 25), \
	/* ADC1_6V6                GPIO_AD_B0_15 GPIO1 Pin 15 */  ADC1_GPIO(4, 15), \
	/* HW_REV_SENSE            GPIO_AD_B1_06 GPIO1 Pin 22 */  ADC1_GPIO(11, 22), \
	/* HW_VER_SENSE            GPIO_AD_B1_04 GPIO1 Pin 20 */  ADC1_GPIO(9, 20), \
	/* SCALED_V5               GPIO_AD_B1_05 GPIO1 Pin 21 */  ADC1_GPIO(10, 21), \
	/* SCALED_VDD_3V3_SENSORS1 GPIO_AD_B1_11 GPIO1 Pin 27 */  ADC1_GPIO(0, 27)

/* Define Channel numbers must match above GPIO pin IN(n)*/

#define ADC_ADC1_3V3_CHANNEL                /* GPIO_AD_B1_09 GPIO1 Pin 25 */  ADC1_CH(14)
#define ADC_ADC1_6V6_CHANNEL                /* GPIO_AD_B0_15 GPIO1 Pin 15 */  ADC1_CH(4)
#define ADC_HW_REV_SENSE_CHANNEL            /* GPIO_AD_B1_06 GPIO1 Pin 22 */  ADC1_CH(11)
#define ADC_HW_VER_SENSE_CHANNEL            /* GPIO_AD_B1_04 GPIO1 Pin 20 */  ADC1_CH(9)
#define ADC_SCALED_V5_CHANNEL               /* GPIO_AD_B1_05 GPIO1 Pin 21 */  ADC1_CH(10)
#define ADC_SCALED_VDD_3V3_SENSORS_CHANNEL  /* GPIO_AD_B1_11 GPIO1 Pin 27 */  ADC1_CH(0)

#define ADC_CHANNELS \
	((1 << ADC_ADC1_3V3_CHANNEL)               | \
	 (1 << ADC_ADC1_6V6_CHANNEL)               | \
	 (1 << ADC_HW_REV_SENSE_CHANNEL)           | \
	 (1 << ADC_HW_VER_SENSE_CHANNEL)           | \
	 (1 << ADC_SCALED_V5_CHANNEL)              | \
	 (1 << ADC_SCALED_VDD_3V3_SENSORS_CHANNEL))

/* HW has to large of R termination on ADC todo:change when HW value is chosen */

#define BOARD_ADC_OPEN_CIRCUIT_V     (5.6f)

/* HW Version and Revision drive signals Default to 1 to detect */

#define BOARD_HAS_HW_VERSIONING

#define HW_IOMUX (IOMUX_CMOS_OUTPUT | IOMUX_PULL_NONE | IOMUX_DRIVE_33OHM | IOMUX_SPEED_MAX | IOMUX_SLEW_FAST)

#define GPIO_HW_VER_REV_DRIVE /* GPIO_AD_B0_01 GPIO1_IO01   */  (GPIO_PORT1 | GPIO_PIN1 | GPIO_OUTPUT | GPIO_OUTPUT_ONE | HW_IOMUX)
#define GPIO_HW_REV_SENSE     /* GPIO_AD_B1_08 GPIO1 Pin 24 */  ADC1_GPIO(13, 24)
#define GPIO_HW_VER_SENSE     /* GPIO_AD_B1_04 GPIO1 Pin 20 */  ADC1_GPIO(9,  20)
#define HW_INFO_INIT_PREFIX   "V5X"

#define UAVCAN_NUM_IFACES_RUNTIME 1

/* HEATER
 * PWM in future
 */
#define HEATER_IOMUX (IOMUX_CMOS_OUTPUT | IOMUX_PULL_NONE | IOMUX_DRIVE_50OHM | IOMUX_SPEED_MEDIUM | IOMUX_SLEW_FAST)
#define GPIO_HEATER_OUTPUT   /* GPIO_B1_09 QTIMER2_TIMER3 GPIO2_IO25 */ (GPIO_QTIMER2_TIMER3_1 | HEATER_IOMUX)
#define HEATER_OUTPUT_EN(on_true)	px4_arch_gpiowrite(GPIO_HEATER_OUTPUT, (on_true))

/* WAKUP is nARMED GPIO5_IO00
 *  The GPIO will be set as input while not armed HW will have external HW Pull UP.
 *  While armed it shall be configured at a GPIO OUT set LOW
 */
#define nARMED_INPUT_IOMUX  (IOMUX_CMOS_INPUT |  IOMUX_PULL_UP_22K | IOMUX_DRIVE_HIZ)
#define nARMED_OUTPUT_IOMUX (IOMUX_CMOS_OUTPUT | IOMUX_PULL_KEEP | IOMUX_DRIVE_33OHM  | IOMUX_SPEED_MEDIUM | IOMUX_SLEW_FAST)

#define GPIO_nARMED_INIT     /* WAKUP GPIO5_IO00 */ (GPIO_PORT5 | GPIO_PIN0 | GPIO_INPUT | nARMED_INPUT_IOMUX)
#define GPIO_nARMED          /* WAKUP GPIO5_IO00 */ (GPIO_PORT5 | GPIO_PIN0 | GPIO_OUTPUT | GPIO_OUTPUT_ZERO | nARMED_OUTPUT_IOMUX)

#define BOARD_INDICATE_EXTERNAL_LOCKOUT_STATE(enabled)  px4_arch_configgpio((enabled) ? GPIO_nARMED : GPIO_nARMED_INIT)
#define BOARD_GET_EXTERNAL_LOCKOUT_STATE() px4_arch_gpioread(GPIO_nARMED)

/* PWM Capture
 *
 * 2  PWM Capture inputs are supported
 */
#define DIRECT_PWM_CAPTURE_CHANNELS  1
#define CAP_IOMUX (IOMUX_CMOS_OUTPUT | IOMUX_PULL_NONE | IOMUX_DRIVE_50OHM | IOMUX_SPEED_MEDIUM | IOMUX_SLEW_FAST)
#define PIN_FLEXPWM2_PWMB0  /* P2:7  PWM2 B0 FMU_CAP1 */ (CAP_IOMUX | GPIO_FLEXPWM2_PWMB00_2)

/* PWM
 */

#define DIRECT_PWM_OUTPUT_CHANNELS  8
#define BOARD_NUM_IO_TIMERS         8

// Input Capture not supported on MVP

#define BOARD_HAS_NO_CAPTURE

/* Power supply control and monitoring GPIOs */

#define GENERAL_INPUT_IOMUX  (IOMUX_CMOS_INPUT |  IOMUX_PULL_UP_47K | IOMUX_DRIVE_HIZ)
#define GENERAL_OUTPUT_IOMUX (IOMUX_CMOS_OUTPUT | IOMUX_PULL_KEEP | IOMUX_DRIVE_33OHM  | IOMUX_SPEED_MEDIUM | IOMUX_SLEW_FAST)

#define GPIO_nPOWER_IN_A                /* GPIO_EMC_14	 GPIO4_IO14 */ (GPIO_PORT4 | GPIO_PIN14 | GPIO_INPUT | GENERAL_INPUT_IOMUX)
#define GPIO_nPOWER_IN_B                /* GPIO_AD_B1_10 GPIO1_IO26 */ (GPIO_PORT1 | GPIO_PIN26 | GPIO_INPUT | GENERAL_INPUT_IOMUX)
#define GPIO_nPOWER_IN_C                /* GPIO_AD_B0_14 GPIO1_IO14 */ (GPIO_PORT1 | GPIO_PIN14 | GPIO_INPUT | GENERAL_INPUT_IOMUX)

#define GPIO_nVDD_BRICK1_VALID          GPIO_nPOWER_IN_A /* Brick 1 Is Chosen */
#define GPIO_nVDD_BRICK2_VALID          GPIO_nPOWER_IN_B /* Brick 2 Is Chosen  */
#define BOARD_NUMBER_BRICKS             2
#define BOARD_NUMBER_DIGITAL_BRICKS     2
#define GPIO_nVDD_USB_VALID             GPIO_nPOWER_IN_C /* USB     Is Chosen */

#define OC_INPUT_IOMUX  (IOMUX_CMOS_INPUT |  IOMUX_PULL_NONE | IOMUX_DRIVE_HIZ)

#define GPIO_VDD_5V_PERIPH_nEN          /* GPIO_EMC_09   GPIO4_IO09 */ (GPIO_PORT4 | GPIO_PIN9  | GPIO_OUTPUT | GPIO_OUTPUT_ONE | GENERAL_OUTPUT_IOMUX)
#define GPIO_VDD_5V_PERIPH_nOC          /* GPIO_B1_04    GPIO2_IO20 */ (GPIO_PORT2 | GPIO_PIN20 | GPIO_INPUT  | OC_INPUT_IOMUX)
#define GPIO_VDD_5V_HIPOWER_nEN         /* GPIO_AD_B0_10 GPIO1_IO10 */ (GPIO_PORT1 | GPIO_PIN10 | GPIO_OUTPUT | GPIO_OUTPUT_ONE | GENERAL_OUTPUT_IOMUX)
#define GPIO_VDD_5V_HIPOWER_nOC         /* GPIO_EMC_06   GPIO4_IO06 */ (GPIO_PORT4 | GPIO_PIN6  | GPIO_INPUT  | OC_INPUT_IOMUX)
#define GPIO_VDD_3V3_SENSORS1_EN        /* GPIO_EMC_41   GPIO3_IO27 */ (GPIO_PORT3 | GPIO_PIN27 | GPIO_OUTPUT | GPIO_OUTPUT_ZERO | GENERAL_OUTPUT_IOMUX)
#define GPIO_VDD_3V3_SPEKTRUM_POWER_EN  /* GPIO_AD_B0_00 GPIO1_IO00 */ (GPIO_PORT1 | GPIO_PIN0  | GPIO_OUTPUT | GPIO_OUTPUT_ZERO | GENERAL_OUTPUT_IOMUX)
#define GPIO_VDD_3V3_SD_CARD_EN         /* GPIO_EMC_13   GPIO4_IO13 */ (GPIO_PORT4 | GPIO_PIN13 | GPIO_OUTPUT | GPIO_OUTPUT_ZERO |GENERAL_OUTPUT_IOMUX)


/* ETHERNET GPIO */

#define GPIO_ETH_POWER_EN              /* PMIC_ON_REQ GPIO5_IO01 */ (GPIO_PORT5 | GPIO_PIN1  | GPIO_OUTPUT | GPIO_OUTPUT_ZERO | GENERAL_OUTPUT_IOMUX)
//GPIO_INPUT  | IOMUX_CMOS_INPUT | IOMUX_PULL_NONE | IOMUX_DRIVE_HIZ)
#define GPIO_ENET2_RX_DATA01_CONFIG4 /* GPIO_B1_02 GPIO2_IO18 (RMII-Rev)     Low  */  (GPIO_PORT2 | GPIO_PIN18 | GPIO_OUTPUT | GPIO_OUTPUT_ZERO | GENERAL_OUTPUT_IOMUX)
#define GPIO_ENET2_RX_DATA00_CONFIG5 /* GPIO_B1_01 GPIO2_IO17 SLAVE:Auto     Open */  (GPIO_PORT2 | GPIO_PIN17 | GPIO_INPUT  | IOMUX_CMOS_INPUT | IOMUX_PULL_NONE | IOMUX_DRIVE_HIZ)
#define GPIO_ENET2_RX_EN_CONFIG6     /* GPIO_B1_03 GPIO2_IO19 SLAVE:POl Corr Low  */  (GPIO_PORT2 | GPIO_PIN19 | GPIO_OUTPUT | GPIO_OUTPUT_ZERO | GENERAL_OUTPUT_IOMUX)

/* NFC GPIO */

#define GPIO_NFC_GPIO                  /* PMIC_STBY_REQ GPIO5_IO02 */ (GPIO_PORT5 | GPIO_PIN2  | GPIO_INPUT |  GENERAL_INPUT_IOMUX)


/* Define True logic Power Control in arch agnostic form */

#define VDD_5V_PERIPH_EN(on_true)          px4_arch_gpiowrite(GPIO_VDD_5V_PERIPH_nEN, !(on_true))
#define VDD_5V_HIPOWER_EN(on_true)         px4_arch_gpiowrite(GPIO_VDD_5V_HIPOWER_nEN, !(on_true))
#define VDD_3V3_SENSORS1_EN(on_true)       px4_arch_gpiowrite(GPIO_VDD_3V3_SENSORS1_EN, (on_true))
#define VDD_3V3_SPEKTRUM_POWER_EN(on_true) px4_arch_gpiowrite(GPIO_VDD_3V3_SPEKTRUM_POWER_EN, (on_true))
#define READ_VDD_3V3_SPEKTRUM_POWER_EN()   px4_arch_gpioread(GPIO_VDD_3V3_SPEKTRUM_POWER_EN)
#define VDD_3V3_SD_CARD_EN(on_true)        px4_arch_gpiowrite(GPIO_VDD_3V3_SD_CARD_EN, (on_true))
#define VDD_3V3_ETH_POWER_EN(on_true)      px4_arch_gpiowrite(GPIO_ETH_POWER_EN, (on_true))


/* Tone alarm output */

#define TONE_ALARM_TIMER        2  /* GPT 2 */
#define TONE_ALARM_CHANNEL      3  /* GPIO_AD_B1_07 GPT2_COMPARE3 */

#define GPIO_BUZZER_1           /* GPIO_AD_B1_07  GPIO1_IO23  */ (GPIO_PORT1 | GPIO_PIN23  | GPIO_OUTPUT | GPIO_OUTPUT_ZERO | GENERAL_OUTPUT_IOMUX)

#define GPIO_TONE_ALARM_IDLE    GPIO_BUZZER_1
#define GPIO_TONE_ALARM         (GPIO_GPT2_COMPARE3_2 | GENERAL_OUTPUT_IOMUX)

/* USB OTG FS
 *
 * VBUS_VALID is detected in USB_ANALOG_USB1_VBUS_DETECT_STAT
 */

/* High-resolution timer */
#define HRT_TIMER               1  /* use GPT1 for the HRT */
#define HRT_TIMER_CHANNEL       1  /* use capture/compare channel 1 */

#define HRT_PPM_CHANNEL         /* GPIO_B1_06 GPT1_CAPTURE2 */  2  /* use capture/compare channel 2 */
#define GPIO_PPM_IN             /* GPIO_B1_06 GPT1_CAPTURE2 */ (GPIO_GPT1_CAPTURE2_2 | GENERAL_INPUT_IOMUX)

/* RC Serial port (When No IO) */

#define RC_SERIAL_PORT                     "/dev/ttyS5"
#define RC_SERIAL_SINGLEWIRE

/* PWM input driver. Use FMU AUX5 pins attached to GPIO_EMC_33 GPIO3_IO19 FLEXPWM3_PWMA2 */

#define PWMIN_TIMER            /* FLEXPWM3_PWMA2 */  3
#define PWMIN_TIMER_CHANNEL    /* FLEXPWM3_PWMA2 */  2
#define GPIO_PWM_IN            /* GPIO_EMC_33 GPIO3_IO19 */ (GPIO_FLEXPWM3_PWMA02_1 | GENERAL_INPUT_IOMUX)

/* Safety Switch is HW version dependent on having an PX4IO
 * So we init to a benign state with the _INIT definition
 * and provide the the non _INIT one for the driver to make a run time
 * decision to use it.
 */
#define SAFETY_INIT_IOMUX (IOMUX_CMOS_INPUT  | IOMUX_PULL_NONE | IOMUX_DRIVE_HIZ)
#define SAFETY_IOMUX      (IOMUX_CMOS_OUTPUT | IOMUX_PULL_NONE | IOMUX_DRIVE_33OHM  | IOMUX_SPEED_MEDIUM | IOMUX_SLEW_SLOW)
#define SAFETY_SW_IOMUX   (IOMUX_CMOS_INPUT  | IOMUX_PULL_UP_22K | IOMUX_DRIVE_HIZ)

#define GPIO_nSAFETY_SWITCH_LED_OUT_INIT   /* GPIO_B1_08 GPIO2_IO24 */  (GPIO_PORT2 | GPIO_PIN24 | GPIO_INPUT  | SAFETY_INIT_IOMUX)
#define GPIO_nSAFETY_SWITCH_LED_OUT        /* GPIO_B1_08 GPIO2_IO24 */ (GPIO_PORT2 | GPIO_PIN24 | GPIO_OUTPUT | GPIO_OUTPUT_ONE | SAFETY_IOMUX)

/* Enable the FMU to control it if there is no px4io fixme:This should be BOARD_SAFETY_LED(__ontrue) */
#define GPIO_LED_SAFETY GPIO_nSAFETY_SWITCH_LED_OUT
#define GPIO_SAFETY_SWITCH_IN              /* GPIO_AD_B1_12 GPIO1_IO28 */ (GPIO_PORT1 | GPIO_PIN28 | GPIO_INPUT | SAFETY_SW_IOMUX)
/* Enable the FMU to use the switch it if there is no px4io fixme:This should be BOARD_SAFTY_BUTTON() */
#define GPIO_BTN_SAFETY GPIO_SAFETY_SWITCH_IN /* Enable the FMU to control it if there is no px4io */

/* Power switch controls ******************************************************/

#define SPEKTRUM_POWER(_on_true)           VDD_3V3_SPEKTRUM_POWER_EN(_on_true)

/*
 * FMUv5 has a separate RC_IN
 *
 * GPIO PPM_IN on GPIO_B1_06 GPIO2 Pin 23 GPT1_CAPTURE2
 * SPEKTRUM_RX (it's TX or RX in Bind) on TX UART6 GPIO_EMC_25 GPIO4 Pin 25
 *   Inversion is possible in the UART and can drive GPIO PPM_IN as an output
 */

#define GPIO_PPM_IN_AS_OUT             /* GPIO_B1_06 GPIO2_IO23 GPT1_CAPTURE2 GPT1_CAPTURE2 */ (GPIO_PORT2 | GPIO_PIN23 | GPIO_OUTPUT | GPIO_OUTPUT_ONE | GENERAL_OUTPUT_IOMUX)
#define SPEKTRUM_RX_AS_GPIO_OUTPUT()   px4_arch_configgpio(GPIO_PPM_IN_AS_OUT)
#define SPEKTRUM_RX_AS_UART()          /* Can be left as uart */
#define SPEKTRUM_OUT(_one_true)        px4_arch_gpiowrite(GPIO_PPM_IN_AS_OUT, (_one_true))

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

#define BOARD_HAS_ON_RESET 1

#define PX4_GPIO_INIT_LIST { \
		GPIO_nARMED_INIT,                 \
		GPIO_ETH_POWER_EN,                \
		GPIO_ENET2_RX_DATA01_CONFIG4,     \
		GPIO_ENET2_RX_DATA00_CONFIG5,     \
		GPIO_ENET2_RX_EN_CONFIG6,         \
		PX4_ADC_GPIO,                     \
		GPIO_HW_VER_REV_DRIVE,            \
		GPIO_FLEXCAN1_TX,                 \
		GPIO_FLEXCAN1_RX,                 \
		GPIO_FLEXCAN2_TX,                 \
		GPIO_FLEXCAN2_RX,                 \
		GPIO_FLEXCAN3_TX,                 \
		GPIO_FLEXCAN3_RX,                 \
		GPIO_HEATER_OUTPUT,               \
		GPIO_nPOWER_IN_A,                 \
		GPIO_nPOWER_IN_B,                 \
		GPIO_nPOWER_IN_C,                 \
		GPIO_VDD_5V_PERIPH_nEN,           \
		GPIO_VDD_5V_PERIPH_nOC,           \
		GPIO_VDD_5V_HIPOWER_nEN,          \
		GPIO_VDD_5V_HIPOWER_nOC,          \
		GPIO_VDD_3V3_SENSORS1_EN,         \
		GPIO_VDD_3V3_SD_CARD_EN,          \
		GPIO_VDD_3V3_SPEKTRUM_POWER_EN,   \
		GPIO_TONE_ALARM_IDLE,             \
		GPIO_nSAFETY_SWITCH_LED_OUT_INIT, \
		GPIO_SPI4_DRDY1_EXTERNAL1,        \
		GPIO_SPI4_nRESET_EXTERNAL1,       \
		GPIO_SYNC,                        \
		GPIO_SAFETY_SWITCH_IN,            \
		GPIO_NFC_GPIO                     \
	}

#define BOARD_ENABLE_CONSOLE_BUFFER

#define PX4_I2C_BUS_MTD      4,5

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
