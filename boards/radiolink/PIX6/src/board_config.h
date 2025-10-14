/****************************************************************************
 *
 *   Copyright (c) 2016-2022 PX4 Development Team. All rights reserved.
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

/****************************************************************************************************
 * Included Files
 ****************************************************************************************************/

#include <px4_platform/board_ctrl.h>
#include <px4_platform_common/px4_config.h>
#include <nuttx/compiler.h>
#include <stdint.h>

#include <stm32_gpio.h>

/****************************************************************************************************
 * Definitions
 ****************************************************************************************************/

/* PX4IO connection configuration */

#define BOARD_USES_PX4IO_VERSION       2
#define PX4IO_SERIAL_DEVICE            "/dev/ttyS5"
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

#define BOARD_HAS_STATIC_MANIFEST 1

#define BOARD_HAS_LTC44XX_VALIDS      0 // No LTC or N Bricks
#define BOARD_HAS_USB_VALID           0 // LTC Has No USB valid
#define BOARD_HAS_NBAT_V              1 // Only one Vbat to ADC
#define BOARD_HAS_NBAT_I              0 // No Ibat ADC

/* LEDs */
#define GPIO_LED1                    (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_25MHz|GPIO_OUTPUT_SET|GPIO_PORTD|GPIO_PIN14)
#define GPIO_LED2                    (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_25MHz|GPIO_OUTPUT_SET|GPIO_PORTC|GPIO_PIN1)
#define GPIO_LED3                    (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_25MHz|GPIO_OUTPUT_SET|GPIO_PORTC|GPIO_PIN7)

#define GPIO_LED_RED                 GPIO_LED1
#define GPIO_LED_GREEN               GPIO_LED2
#define GPIO_LED_BLUE                GPIO_LED3

#define BOARD_HAS_CONTROL_STATUS_LEDS 1
#define BOARD_OVERLOAD_LED     LED_RED
#define BOARD_ARMED_LED        LED_BLUE
#define BOARD_ARMED_STATE_LED  LED_GREEN

/* Safety Switch is HW version dependent on having an PX4IO
 * So we init to a benign state with the _INIT definition
 * and provide the the non _INIT one for the driver to make a run time
 * decision to use it.
 */
#define GPIO_nSAFETY_SWITCH_LED_OUT_INIT   /* PB0 */ (GPIO_INPUT|GPIO_FLOAT|GPIO_PORTB|GPIO_PIN0)
/*
 * ADC channels
 *
 * These are the channel numbers of the ADCs of the microcontroller that
 * can be used by the Px4 Firmware in the adc driver
 */

/* ADC defines to be used in sensors.cpp to read from a particular channel */

#define ADC1_CH(n)                  (n)
#define ADC1_GPIO(n)                GPIO_ADC1_IN##n

/* Define GPIO pins used as ADC N.B. Channel numbers must match below */

#define PX4_ADC_GPIO  \
	/* PA2 */  ADC1_GPIO(2),  \
	/* PA4 */  ADC1_GPIO(4),  \
	/* PA5 */  ADC1_GPIO(5),  \
	/* PC0 */  ADC1_GPIO(10), \
	/* PC2 */  ADC1_GPIO(12), \
	/* PC3 */  ADC1_GPIO(13)

/* Define Channel numbers must match above GPIO pin IN(n)*/

#define ADC_BATTERY_VOLTAGE_CHANNEL     	/* PA2 */  ADC1_CH(2)
#define ADC_BATTERY_CURRENT_CHANNEL     	/* PA5 */  ADC1_CH(5)
#define ADC_SCALED_V5_CHANNEL                   /* PC0 */  ADC1_CH(10)
#define ADC_ADC1_6V6_CHANNEL  			/* PC2 */  ADC1_CH(12)
#define ADC_ADC1_3V3_CHANNEL1                   /* PC3 */  ADC1_CH(13)
#define ADC_ADC1_3V3_CHANNEL2			/* PA4 */  ADC1_CH(4)

#define ADC_CHANNELS \
	((1 << ADC_BATTERY_VOLTAGE_CHANNEL) | \
	 (1 << ADC_BATTERY_CURRENT_CHANNEL) | \
	 (1 << ADC_SCALED_V5_CHANNEL) | \
	 (1 << ADC_ADC1_3V3_CHANNEL2)               | \
	 (1 << ADC_ADC1_6V6_CHANNEL)                | \
	 (1 << ADC_ADC1_3V3_CHANNEL1))

/* HW has to large of R termination on ADC todo:change when HW value is chosen */

#define SYSTEM_ADC_BASE STM32_ADC1_BASE

#define BOARD_ADC_OPEN_CIRCUIT_V     (5.6f)

//#define UAVCAN_NUM_IFACES_RUNTIME 1

/* PWM
 */
#define DIRECT_PWM_OUTPUT_CHANNELS  8


#define GPIO_VDD_5V_PERIPH_nEN          /* PC4  */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTC|GPIO_PIN4)
#define GPIO_VDD_5V_PERIPH_nOC          /* PE15 */ (GPIO_INPUT |GPIO_PULLUP|GPIO_PORTE|GPIO_PIN15)
#define GPIO_VDD_5V_HIPOWER_nEN         /* PD13 */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTD|GPIO_PIN13)
#define GPIO_VDD_5V_HIPOWER_nOC         /* PE10 */ (GPIO_INPUT |GPIO_PULLUP|GPIO_PORTE|GPIO_PIN10)
#define GPIO_VDD_3V3_SENSORS_EN        /* PE3  */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTE|GPIO_PIN3)
#define GPIO_VDD_3V3_SPEKTRUM_POWER_EN  /* PE4 */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTE|GPIO_PIN4)

/* Define True logic Power Control in arch agnostic form */

#define VDD_5V_PERIPH_EN(on_true)          px4_arch_gpiowrite(GPIO_VDD_5V_PERIPH_nEN, !(on_true))
#define VDD_5V_HIPOWER_EN(on_true)         px4_arch_gpiowrite(GPIO_VDD_5V_HIPOWER_nEN, !(on_true))
#define VDD_3V3_SENSORS4_EN(on_true)       px4_arch_gpiowrite(GPIO_VDD_3V3_SENSORS_EN, (on_true))
#define VDD_3V3_SPEKTRUM_POWER_EN(on_true) px4_arch_gpiowrite(GPIO_VDD_3V3_SPEKTRUM_POWER_EN, (on_true))
#define READ_VDD_3V3_SPEKTRUM_POWER_EN()   px4_arch_gpioread(GPIO_VDD_3V3_SPEKTRUM_POWER_EN)


/* Tone alarm output */

#define TONE_ALARM_TIMER        9  /* Timer 14 */
#define TONE_ALARM_CHANNEL      1  /* PE5 GPIO_TIM14_CH1OUT_2 */

#define GPIO_BUZZER_1           /* PE5 */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTE|GPIO_PIN5)

#define GPIO_TONE_ALARM_IDLE    GPIO_BUZZER_1
#define GPIO_TONE_ALARM         GPIO_TIM9_CH1OUT_2

#define HRT_TIMER               5  /* use timer5 for the HRT */
#define HRT_TIMER_CHANNEL       4  /* use capture/compare channel 4 */
/* USB OTG FS
 *
 * PA9  OTG_FS_VBUS VBUS sensing
 */
#define GPIO_nVDD_USB_VALID         /* PB2 */ (GPIO_INPUT|GPIO_PULLUP|GPIO_SPEED_100MHz|GPIO_PORTB|GPIO_PIN2)

/* PWM input driver. Use FMU AUX5 pins attached to timer4 channel 2 */
#define PWMIN_TIMER                       2
#define PWMIN_TIMER_CHANNEL    /* T2C1 */ 1
#define GPIO_PWM_IN            /* PA15 */ GPIO_TIM2_CH1IN_2
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
#define BOARD_ADC_USB_CONNECTED (px4_arch_gpioread(GPIO_nVDD_USB_VALID))
#define BOARD_ADC_USB_VALID     BOARD_ADC_USB_CONNECTED

#define BOARD_ADC_PERIPH_5V_OC  (!px4_arch_gpioread(GPIO_VDD_5V_PERIPH_nOC))
#define BOARD_ADC_HIPOWER_5V_OC (!px4_arch_gpioread(GPIO_VDD_5V_HIPOWER_nOC))


/* This board provides a DMA pool and APIs */
#define BOARD_DMA_ALLOC_POOL_SIZE 5120

/* This board provides the board_on_reset interface */

#define BOARD_HAS_ON_RESET 1

#define PX4_GPIO_INIT_LIST { \
		PX4_ADC_GPIO,                     \
		GPIO_CAN1_TX,                     \
		GPIO_CAN1_RX,                     \
		GPIO_CAN2_TX,                     \
		GPIO_CAN2_RX,                     \
		GPIO_VDD_5V_PERIPH_nEN,           \
		GPIO_VDD_5V_PERIPH_nOC,           \
		GPIO_VDD_5V_HIPOWER_nEN,          \
		GPIO_VDD_5V_HIPOWER_nOC,          \
		GPIO_VDD_3V3_SENSORS_EN,         \
		GPIO_VDD_3V3_SPEKTRUM_POWER_EN,   \
		GPIO_TONE_ALARM_IDLE,             \
		GPIO_nVDD_USB_VALID,                 \
		PX4_MAKE_GPIO_OUTPUT_CLEAR(GPIO_I2C1_SCL), \
		PX4_MAKE_GPIO_OUTPUT_CLEAR(GPIO_I2C1_SDA), \
		PX4_MAKE_GPIO_OUTPUT_CLEAR(GPIO_I2C2_SCL), \
		PX4_MAKE_GPIO_OUTPUT_CLEAR(GPIO_I2C2_SDA), \
	}

#define BOARD_ENABLE_CONSOLE_BUFFER

#define BOARD_NUM_IO_TIMERS 3


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
 *   Called to configure SPI chip select GPIO pins for the board.
 *
 ****************************************************************************************************/

extern void stm32_spiinitialize(void);

extern void stm32_usbinitialize(void);

extern void board_peripheral_reset(int ms);

#include <px4_platform_common/board_common.h>

#endif /* __ASSEMBLY__ */

__END_DECLS
