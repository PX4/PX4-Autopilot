/****************************************************************************
 *
 *   Copyright (c) 2016 PX4 Development Team. All rights reserved.
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
 * PX4FMU-v5 internal definitions
 */

#pragma once

/****************************************************************************************************
 * Included Files
 ****************************************************************************************************/

#include <px4_config.h>
#include <nuttx/compiler.h>
#include <stdint.h>

#include <stm32_gpio.h>

/****************************************************************************************************
 * Definitions
 ****************************************************************************************************/

/* PX4IO connection configuration */

#define BOARD_USES_PX4IO_VERSION       2
#define PX4IO_SERIAL_DEVICE            "/dev/ttyS6"
#define PX4IO_SERIAL_TX_GPIO           GPIO_UART8_TX
#define PX4IO_SERIAL_RX_GPIO           GPIO_UART8_RX
#define PX4IO_SERIAL_BASE              STM32_UART8_BASE
#define PX4IO_SERIAL_VECTOR            STM32_IRQ_UART8
#define PX4IO_SERIAL_TX_DMAMAP         DMAMAP_UART8_TX
#define PX4IO_SERIAL_RX_DMAMAP         DMAMAP_UART8_RX
#define PX4IO_SERIAL_RCC_REG           STM32_RCC_APB1ENR
#define PX4IO_SERIAL_RCC_EN            RCC_APB1ENR_UART8EN
#define PX4IO_SERIAL_CLOCK             STM32_PCLK1_FREQUENCY
#define PX4IO_SERIAL_BITRATE           1500000               /* 1.5Mbps -> max rate for IO */

/* Configuration ************************************************************************************/

/* Un-comment to support some RC00 polarities inversions
 * on test HW as well as R and G LEDs on UI LED are swapped
 */
//#define PX4_FMUV5_RC00

#define PX4_FMUV5_RC01
#define BOARD_HAS_LTC4417

#if defined(BOARD_HAS_LTC4417)
#  define BOARD_HAS_LTC44XX_VALIDS      2 // No LTC or N Bricks
#  define BOARD_HAS_USB_VALID           1 // LTC Has No USB valid
#  define BOARD_HAS_NBAT_V              2 // Only one Vbat to ADC
#  define BOARD_HAS_NBAT_I              2 // No Ibat ADC
#else
#  define BOARD_HAS_LTC44XX_VALIDS      0 // No LTC or N Bricks
#  define BOARD_HAS_USB_VALID           0 // LTC Has No USB valid
#  define BOARD_HAS_NBAT_V              1 // Only one Vbat to ADC
#  define BOARD_HAS_NBAT_I              0 // No Ibat ADC
#endif

/* Revision specific config */

/* Power supply control and monitoring */

#if defined(PX4_FMUV5_RC00)
#  define GPIO_VDD_5V_PERIPH_EN         /* PG4  */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTG|GPIO_PIN4)
#  define GPIO_xVDD_5V_PERIPH_EN                   GPIO_VDD_5V_PERIPH_EN
#endif
#if defined(PX4_FMUV5_RC01)
#  define GPIO_nVDD_5V_PERIPH_EN         /* PG4  */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTG|GPIO_PIN4)
#  define GPIO_xVDD_5V_PERIPH_EN                    GPIO_nVDD_5V_PERIPH_EN
#endif

/* Define True logic Power Control in arch agnostic form */

#if defined(PX4_FMUV5_RC00)
#define VDD_5V_PERIPH_EN(on_true)          px4_arch_gpiowrite(GPIO_VDD_5V_PERIPH_EN, (on_true))
#endif
#if defined(PX4_FMUV5_RC01)
#define VDD_5V_PERIPH_EN(on_true)          px4_arch_gpiowrite(GPIO_nVDD_5V_PERIPH_EN, !(on_true))
#endif

#if defined(PX4_FMUV5_RC00)
# define BOARD_UI_LED_SWAP_RG           1
#else
#  define BOARD_UI_LED_PWM_DRIVE_ACTIVE_LOW 1
#endif

/* PX4FMU GPIOs ***********************************************************************************/
/* Include the pin adn bus configuration **********************************************************/

#include "gpio_config.h"

/* Define Battery 1 Voltage Divider and A per V */

#define BOARD_BATTERY1_V_DIV         (18.1f)     /* measured with the provided PM board */
#define BOARD_BATTERY1_A_PER_V       (36.367515152f)

/* HW has to large of R termination on ADC todo:change when HW value is chosen */

#define BOARD_ADC_OPEN_CIRCUIT_V     (5.6f)

/* HW verision init */

#define HW_INFO_INIT         {'V','5','x', 'x',0}

/* RC serial port */

#define RC_SERIAL_PORT       "/dev/ttyS4"

/* This board provides a DMA pool and APIs */

#define BOARD_DMA_ALLOC_POOL_SIZE 5120

/* This board provides the board_on_reset interface */

#define BOARD_HAS_ON_RESET 1

/* The list of GPIO that will be initialized */

#define PX4_GPIO_PWM_INIT_LIST { \
		GPIO_GPIO7_INPUT, \
		GPIO_GPIO6_INPUT, \
		GPIO_GPIO5_INPUT, \
		GPIO_GPIO4_INPUT, \
		GPIO_GPIO3_INPUT, \
		GPIO_GPIO2_INPUT, \
		GPIO_GPIO1_INPUT, \
		GPIO_GPIO0_INPUT, \
	}

#define PX4_GPIO_INIT_LIST { \
		PX4_ADC_GPIO,                     \
		GPIO_HW_REV_DRIVE,                \
		GPIO_HW_VER_DRIVE,                \
		GPIO_CAN1_TX,                     \
		GPIO_CAN1_RX,                     \
		GPIO_CAN2_TX,                     \
		GPIO_CAN2_RX,                     \
		GPIO_CAN3_TX,                     \
		GPIO_CAN3_RX,                     \
		GPIO_CAN1_SILENT_S0,              \
		GPIO_CAN2_SILENT_S1,              \
		GPIO_CAN3_SILENT_S2,              \
		GPIO_HEATER_OUTPUT,               \
		GPIO_nPOWER_IN_A,                 \
		GPIO_nPOWER_IN_B,                 \
		GPIO_nPOWER_IN_C,                 \
		GPIO_xVDD_5V_PERIPH_EN,           \
		GPIO_nVDD_5V_PERIPH_OC,           \
		GPIO_nVDD_5V_HIPOWER_EN,          \
		GPIO_nVDD_5V_HIPOWER_OC,          \
		GPIO_VDD_3V3_SENSORS_EN,          \
		GPIO_VDD_3V3_SPEKTRUM_POWER_EN,   \
		GPIO_VDD_5V_RC_EN,                \
		GPIO_VDD_5V_WIFI_EN,              \
		GPIO_VDD_3V3_SD_CARD_EN,          \
		GPIO_TONE_ALARM_IDLE,             \
		GPIO_RSSI_IN_INIT,                \
		GPIO_nSAFETY_SWITCH_LED_OUT_INIT, \
		GPIO_SAFETY_SWITCH_IN             \
	}

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
 * Name: stm32_sdio_initialize
 *
 * Description:
 *   Initialize SDIO-based MMC/SD card support
 *
 ****************************************************************************/

int stm32_sdio_initialize(void);

/****************************************************************************************************
 * Name: stm32_spiinitialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the PX4FMU board.
 *
 ****************************************************************************************************/

extern void stm32_spiinitialize(void);

/************************************************************************************
 * Name: stm32_spi_bus_initialize
 *
 * Description:
 *   Called to configure SPI Buses.
 *
 ************************************************************************************/

extern int stm32_spi_bus_initialize(void);

void board_spi_reset(int ms);

extern void stm32_usbinitialize(void);

extern void board_peripheral_reset(int ms);


/****************************************************************************
 * Name: nsh_archinitialize
 *
 * Description:
 *   Perform architecture specific initialization for NSH.
 *
 *   CONFIG_NSH_ARCHINIT=y :
 *     Called from the NSH library
 *
 *   CONFIG_BOARD_INITIALIZE=y, CONFIG_NSH_LIBRARY=y, &&
 *   CONFIG_NSH_ARCHINIT=n :
 *     Called from board_initialize().
 *
 ****************************************************************************/

#ifdef CONFIG_NSH_LIBRARY
int nsh_archinitialize(void);
#endif

#include <drivers/boards/common/board_common.h>

#endif /* __ASSEMBLY__ */

__END_DECLS
