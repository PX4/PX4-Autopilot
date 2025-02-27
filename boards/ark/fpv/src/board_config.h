/****************************************************************************
 *
 *   Copyright (c) 2016-2024 PX4 Development Team. All rights reserved.
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
 * ARK FPV internal definitions
 */

#pragma once

/****************************************************************************************************
 * Included Files
 ****************************************************************************************************/

#include <px4_platform_common/px4_config.h>
#include <nuttx/compiler.h>
#include <stdint.h>


#include <stm32_gpio.h>

/****************************************************************************************************
 * Definitions
 ****************************************************************************************************/

#undef TRACE_PINS

/* Configuration ************************************************************************************/

#  define BOARD_HAS_USB_VALID           1
#  define BOARD_HAS_NBAT_V              1
#  define BOARD_HAS_NBAT_I              1

/* PX4FMU GPIOs ***********************************************************************************/

/* Trace Clock and D0-D3 are available on the trace connector
 *
 * TRACECLK PE2  - Dedicated       - Trace Connector Pin 1
 * TRACED0  PE3  - nLED_RED        - Trace Connector Pin 3
 * TRACED1  PE4  - nLED_GREEN      - Trace Connector Pin 5
 * TRACED2  PE5  - nLED_BLUE       - Trace Connector Pin 7
 * TRACED3  PE6  - nARMED          - Trace Connector Pin 8

 */

/* LEDs are driven with push open drain to support Anode to 5V or 3.3V or used as TRACE0-2 */

#if !defined(TRACE_PINS)
#  define GPIO_nLED_RED        /* PE3 */  (GPIO_OUTPUT|GPIO_OPENDRAIN|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTE|GPIO_PIN3)
#  define GPIO_nLED_GREEN      /* PE4 */  (GPIO_OUTPUT|GPIO_OPENDRAIN|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTE|GPIO_PIN4)
#  define GPIO_nLED_BLUE       /* PE5 */  (GPIO_OUTPUT|GPIO_OPENDRAIN|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTE|GPIO_PIN5)

#  define BOARD_HAS_CONTROL_STATUS_LEDS      1
#  define BOARD_OVERLOAD_LED     LED_RED
#  define BOARD_ARMED_STATE_LED  LED_BLUE

#else

#  define GPIO_TRACECLK1 (GPIO_TRACECLK |GPIO_PULLUP|GPIO_SPEED_100MHz|GPIO_PUSHPULL)  //(GPIO_ALT|GPIO_AF0|GPIO_PORTE|GPIO_PIN2)
#  define GPIO_TRACED0   (GPIO_TRACED0_2|GPIO_PULLUP|GPIO_SPEED_100MHz|GPIO_PUSHPULL)  //(GPIO_ALT|GPIO_AF0|GPIO_PORTE|GPIO_PIN3)
#  define GPIO_TRACED1   (GPIO_TRACED1_2|GPIO_PULLUP|GPIO_SPEED_100MHz|GPIO_PUSHPULL)  //(GPIO_ALT|GPIO_AF0|GPIO_PORTE|GPIO_PIN4)
#  define GPIO_TRACED2   (GPIO_TRACED2_2|GPIO_PULLUP|GPIO_SPEED_100MHz|GPIO_PUSHPULL)  //(GPIO_ALT|GPIO_AF0|GPIO_PORTE|GPIO_PIN5)
#  define GPIO_TRACED3   (GPIO_TRACED3_2|GPIO_PULLUP|GPIO_SPEED_100MHz|GPIO_PUSHPULL)  //(GPIO_ALT|GPIO_AF0|GPIO_PORTE|GPIO_PIN6)
//#define GPIO_TRACESWO                        //(GPIO_ALT|GPIO_AF0|GPIO_PORTB|GPIO_PIN3)

#  undef  BOARD_HAS_CONTROL_STATUS_LEDS
#  undef  BOARD_OVERLOAD_LED
#  undef  BOARD_ARMED_STATE_LED

#  define GPIO_nLED_RED    GPIO_TRACED0
#  define GPIO_nLED_GREEN  GPIO_TRACED1
#  define GPIO_nLED_BLUE   GPIO_TRACED2
#  define GPIO_nARMED      GPIO_TRACED3
#  define GPIO_nARMED_INIT GPIO_TRACED3
#endif

/* SPI */

#define SPI6_nRESET_EXTERNAL1       /* PF10 */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTF|GPIO_PIN10)
#define SPI6_RESET(on_true)          px4_arch_gpiowrite(SPI6_nRESET_EXTERNAL1, !(on_true))

/* I2C busses */

/* Devices on the onboard buses.
 *
 * Note that these are unshifted addresses.
 */
#define PX4_I2C_OBDEV_SE050         0x48

/*
 * ADC channels
 *
 * These are the channel numbers of the ADCs of the microcontroller that
 * can be used by the Px4 Firmware in the adc driver
 */

/* ADC defines to be used in sensors.cpp to read from a particular channel */

#define ADC1_CH(n)                  (n)

/* N.B. there is no offset mapping needed for ADC3 because */
#define ADC3_CH(n)                  (n)

/* We are only use ADC3 for REV/VER.
 * ADC3_6V6 and ADC3_3V3 are mapped back to ADC1
 * To do this We are relying on PC2_C, PC3_C being connected to PC2, PC3
 * respectively by the SYSCFG_PMCR default of setting for PC3SO PC2SO PA1SO
 * PA0SO of 0.
 *
 *  0 Analog switch closed (pads are connected through the analog switch)
 *
 * So ADC3_INP0 is GPIO_ADC123_INP12
 *    ADC3_INP1 is GPIO_ADC12_INP13
 */

/* Define GPIO pins used as ADC N.B. Channel numbers must match below  */

#define PX4_ADC_GPIO  \
	/* PA0  */  GPIO_ADC1_INP16,   \
	/* PA4  */  GPIO_ADC12_INP18,  \
	/* PB0  */  GPIO_ADC12_INP9,   \
	/* PB1  */  GPIO_ADC12_INP5,   \
	/* PC2  */  GPIO_ADC123_INP12, \
	/* PC3  */  GPIO_ADC12_INP13,  \
	/* PF12 */  GPIO_ADC1_INP6,    \
	/* PH3  */  GPIO_ADC3_INP14,   \
	/* PH4  */  GPIO_ADC3_INP15

/* Define Channel numbers must match above GPIO pin IN(n)*/
#define ADC_BATTERY_VOLTAGE_CHANNEL        	/* PB0 */  ADC1_CH(9)
#define ADC_BATTERY_CURRENT_CHANNEL        	/* PC2 */  ADC3_CH(12)
#define ADC_SCALED_12V_CHANNEL	     		/* PA4 */  ADC1_CH(18)
#define ADC_SCALED_VDD_3V3_SENSORS_CHANNEL     /* PA0 */  ADC1_CH(16)
#define ADC_SCALED_V5_CHANNEL                   /* PB1 */  ADC1_CH(5)
#define ADC_HW_VER_SENSE_CHANNEL                /* PH3 */  ADC3_CH(14)
#define ADC_HW_REV_SENSE_CHANNEL                /* PH4 */  ADC3_CH(15)

#define ADC_CHANNELS \
	((1 << ADC_BATTERY_VOLTAGE_CHANNEL)         | \
	 (1 << ADC_BATTERY_CURRENT_CHANNEL)         | \
	 (1 << ADC_SCALED_VDD_3V3_SENSORS_CHANNEL)  | \
	 (1 << ADC_SCALED_V5_CHANNEL)               | \
	 (1 << ADC_SCALED_12V_CHANNEL))


#define BOARD_BATTERY1_V_DIV	 (21.0f) // (20k + 1k) / 1k = 21

#define BOARD_BATTERY_ADC_VOLTAGE_FILTER_S 0.075f
#define BOARD_BATTERY_ADC_CURRENT_FILTER_S 0.125f

#define ADC_SCALED_PAYLOAD_SENSE ADC_SCALED_12V_CHANNEL

/* HW has to large of R termination on ADC todo:change when HW value is chosen */

#define HW_REV_VER_ADC_BASE STM32_ADC3_BASE

#define SYSTEM_ADC_BASE STM32_ADC1_BASE

/* HW has to large of R termination on ADC todo:change when HW value is chosen */
#define BOARD_ADC_OPEN_CIRCUIT_V     (5.6f)

/* HW Version and Revision drive signals Default to 1 to detect */
#define BOARD_HAS_HW_SPLIT_VERSIONING

#define GPIO_HW_VER_REV_DRIVE  /* PG0 */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTG|GPIO_PIN0)
#define GPIO_HW_REV_SENSE      /* PH4 */  GPIO_ADC3_INP15
#define GPIO_HW_VER_SENSE      /* PH3 */  GPIO_ADC3_INP14
#define HW_INFO_INIT_PREFIX    "ARKFPV"

#define BOARD_NUM_SPI_CFG_HW_VERSIONS 2
//                 Base/FMUM
#define ARKFPV_0   HW_FMUM_ID(0x0) // ARKFPV,     Sensor Set  Rev 0
#define ARKFPV_1   HW_FMUM_ID(0x1) // ARKFPV,     Sensor Set  Rev 1

#define UAVCAN_NUM_IFACES_RUNTIME  1

/* HEATER
 * PWM in future
 */
#define GPIO_HEATER_OUTPUT   /* PB10  T2CH3 */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTB|GPIO_PIN10)
#define HEATER_OUTPUT_EN(on_true)	       px4_arch_gpiowrite(GPIO_HEATER_OUTPUT, (on_true))

/* PE6 is nARMED
 *  The GPIO will be set as input while not armed HW will have external HW Pull UP.
 *  While armed it shall be configured at a GPIO OUT set LOW
 */
#if !defined(TRACE_PINS)
#define GPIO_nARMED_INIT     /* PE6 */  (GPIO_INPUT|GPIO_PULLUP|GPIO_PORTE|GPIO_PIN6)
#define GPIO_nARMED          /* PE6 */  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTE|GPIO_PIN6)
#define BOARD_INDICATE_EXTERNAL_LOCKOUT_STATE(enabled)  px4_arch_configgpio((enabled) ? GPIO_nARMED : GPIO_nARMED_INIT)
#define BOARD_GET_EXTERNAL_LOCKOUT_STATE() px4_arch_gpioread(GPIO_nARMED)
#endif

/* PWM
 */
#define DIRECT_PWM_OUTPUT_CHANNELS   9

#define GPIO_FMU_CH1                    /* PI0  */ (GPIO_INPUT|GPIO_PULLDOWN|GPIO_PORTI|GPIO_PIN0)
#define GPIO_FMU_CH2                    /* PH12 */ (GPIO_INPUT|GPIO_PULLDOWN|GPIO_PORTH|GPIO_PIN12)
#define GPIO_FMU_CH3                    /* PH11 */ (GPIO_INPUT|GPIO_PULLDOWN|GPIO_PORTH|GPIO_PIN11)
#define GPIO_FMU_CH4                    /* PH10 */ (GPIO_INPUT|GPIO_PULLDOWN|GPIO_PORTH|GPIO_PIN10)
#define GPIO_FMU_CH5                    /* PI5  */ (GPIO_INPUT|GPIO_PULLDOWN|GPIO_PORTI|GPIO_PIN5)
#define GPIO_FMU_CH6                    /* PI6  */ (GPIO_INPUT|GPIO_PULLDOWN|GPIO_PORTI|GPIO_PIN6)
#define GPIO_FMU_CH7                    /* PI7  */ (GPIO_INPUT|GPIO_PULLDOWN|GPIO_PORTI|GPIO_PIN7)
#define GPIO_FMU_CH8                    /* PI2  */ (GPIO_INPUT|GPIO_PULLDOWN|GPIO_PORTI|GPIO_PIN2)
#define GPIO_FMU_CH9                    /* PD12 */ (GPIO_INPUT|GPIO_PULLDOWN|GPIO_PORTD|GPIO_PIN12)

#define GPIO_SPIX_SYNC                  /* PE9  */ (GPIO_INPUT|GPIO_PULLDOWN|GPIO_PORTE|GPIO_PIN9)

/* Power supply control and monitoring GPIOs */

#define GPIO_VDD_5V_PGOOD         	/* PF13 */ (GPIO_INPUT |GPIO_FLOAT|GPIO_PORTF|GPIO_PIN13)
#define GPIO_VDD_12V_PGOOD	 	/* PE15 */ (GPIO_INPUT |GPIO_FLOAT|GPIO_PORTE|GPIO_PIN15)
#define GPIO_5V_ON_BATTERY	 	/* PG1  */ (GPIO_INPUT |GPIO_FLOAT|GPIO_PORTG|GPIO_PIN1)
#define GPIO_VDD_12V_EN			/* PG4  */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTG|GPIO_PIN4)

#define GPIO_VDD_3V3_SD_CARD_EN         /* PC13 */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTC|GPIO_PIN13)


#define BOARD_NUMBER_BRICKS             1

/* Define True logic Power Control in arch agnostic form */

#define VDD_3V3_SD_CARD_EN(on_true)        px4_arch_gpiowrite(GPIO_VDD_3V3_SD_CARD_EN, (on_true))
#define PAYLOAD_POWER_EN(on_true)          px4_arch_gpiowrite(GPIO_VDD_12V_EN, (on_true))

/* USB OTG FS
 *
 * PA9  OTG_FS_VBUS VBUS sensing
 */
#define GPIO_OTGFS_VBUS         /* PA9 */ (GPIO_INPUT|GPIO_PULLDOWN|GPIO_SPEED_100MHz|GPIO_PORTA|GPIO_PIN9)

#define FLASH_BASED_PARAMS

/* High-resolution timer */
#define HRT_TIMER               3  /* use timer3 for the HRT */
#define HRT_TIMER_CHANNEL       1 /* use capture/compare channel 1 */

/* RC Serial port */

#define RC_SERIAL_PORT                     "/dev/ttyS4"

/* PWM input driver. Use FMU AUX5 pins attached to timer4 channel 2 */
#define PWMIN_TIMER                       4
#define PWMIN_TIMER_CHANNEL    /* T4C2 */ 2
#define GPIO_PWM_IN            /* PD13 */ GPIO_TIM4_CH2IN_2

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
#define BOARD_ADC_USB_CONNECTED (px4_arch_gpioread(GPIO_OTGFS_VBUS))

/* ARKFPV never powers off the Servo rail */

#define BOARD_ADC_SERVO_VALID     (1)

#define BOARD_ADC_BRICK_VALID   (px4_arch_gpioread(GPIO_VDD_5V_PGOOD))
#define BOARD_GPIO_PAYOLOAD_V_VALID (px4_arch_gpioread(GPIO_VDD_12V_PGOOD))

/* This board provides a DMA pool and APIs */
#define BOARD_DMA_ALLOC_POOL_SIZE 5120

/* This board has 3 DMA channels available for bidirectional dshot */
#define BOARD_DMA_NUM_DSHOT_CHANNELS 3

/* This board provides the board_on_reset interface */

#define BOARD_HAS_ON_RESET 1

#if defined(TRACE_PINS)
#define GPIO_TRACE                          \
	GPIO_TRACECLK1,                           \
	GPIO_TRACED0,                             \
	GPIO_TRACED1,                             \
	GPIO_TRACED2,                             \
	GPIO_TRACED3
#else
#define GPIO_TRACE (GPIO_OUTPUT|GPIO_OUTPUT_SET|GPIO_PORTE|GPIO_PIN2)
#endif

#define PX4_GPIO_INIT_LIST { \
		GPIO_TRACE,                       \
		PX4_ADC_GPIO,                     \
		GPIO_HW_VER_REV_DRIVE,            \
		GPIO_CAN1_TX,                     \
		GPIO_CAN1_RX,                     \
		GPIO_HEATER_OUTPUT,               \
		GPIO_VDD_5V_PGOOD,                \
		GPIO_VDD_12V_PGOOD,               \
		GPIO_VDD_12V_EN,                  \
		GPIO_5V_ON_BATTERY,               \
		GPIO_VDD_3V3_SD_CARD_EN,          \
		GPIO_nARMED_INIT,                 \
		SPI6_nRESET_EXTERNAL1,            \
		GPIO_FMU_CH1,     	          \
		GPIO_FMU_CH2,     	          \
		GPIO_FMU_CH3,     	          \
		GPIO_FMU_CH4,     	          \
		GPIO_FMU_CH5,     	          \
		GPIO_FMU_CH6,     	          \
		GPIO_FMU_CH7,     	          \
		GPIO_FMU_CH8,     	          \
		GPIO_FMU_CH9,     	          \
		GPIO_SPIX_SYNC                    \
	}

#define BOARD_ENABLE_CONSOLE_BUFFER

#define BOARD_NUM_IO_TIMERS  3
#define BOARD_SPIX_SYNC_FREQ 32000

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

extern void stm32_usbinitialize(void);

extern void board_peripheral_reset(int ms);

#include <px4_platform_common/board_common.h>

#endif /* __ASSEMBLY__ */

__END_DECLS
