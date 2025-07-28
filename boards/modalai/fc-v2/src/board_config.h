/****************************************************************************
 *
 *   Copyright (c) 2016, 2020 PX4 Development Team. All rights reserved.
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
 * ModalAI FC v2 internal definitions
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

/* PX4IO connection configuration */

#define BOARD_USES_PX4IO_VERSION       2
#define PX4IO_SERIAL_DEVICE            "/dev/ttyS5"
#define PX4IO_SERIAL_TX_GPIO           GPIO_USART6_TX
#define PX4IO_SERIAL_RX_GPIO           GPIO_USART6_RX
#define PX4IO_SERIAL_BASE              STM32_USART6_BASE
#define PX4IO_SERIAL_VECTOR            STM32_IRQ_USART6
#define PX4IO_SERIAL_TX_DMAMAP         DMAMAP_USART6_TX
#define PX4IO_SERIAL_RX_DMAMAP         DMAMAP_USART6_RX
#define PX4IO_SERIAL_RCC_REG           STM32_RCC_APB2ENR
#define PX4IO_SERIAL_RCC_EN            RCC_APB2ENR_USART6EN
#define PX4IO_SERIAL_CLOCK             STM32_PCLK2_FREQUENCY
#define PX4IO_SERIAL_BITRATE           1500000               /* 1.5Mbps -> max rate for IO */

/* Configuration ************************************************************************************/


/* PX4FMU GPIOs ***********************************************************************************/

/* Trace Clock and D0-D3 are available on the trace connector
 *
 * TRACECLK PE2  - Dedicated       - Trace Connector Pin 1
 * TRACED0  PE3  - nLED_RED        - Trace Connector Pin 3
 * TRACED1  PE4  - nLED_GREEN      - Trace Connector Pin 5
 * TRACED2  PE5  - nLED_BLUE       - Trace Connector Pin 7
 * TRACED3  PC12 - UART5_TX_TELEM2 - Trace Connector Pin 8

 */
#undef TRACE_PINS

/* LEDs are driven with push open drain to support Anode to 5V or 3.3V or used as TRACE0-2 */

#if !defined(TRACE_PINS)
#  define GPIO_nLED_RED        /* PE3 */  (GPIO_OUTPUT|GPIO_OPENDRAIN|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTE|GPIO_PIN3)
#  define GPIO_nLED_GREEN      /* PE4 */  (GPIO_OUTPUT|GPIO_OPENDRAIN|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTE|GPIO_PIN4)
#  define GPIO_nLED_BLUE       /* PE5 */  (GPIO_OUTPUT|GPIO_OPENDRAIN|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTE|GPIO_PIN5)

// GPIO_nLED_2_RED/ GPIO_nLED_2_GREEN /GPIO_nLED_2_BLUE are for v1 LED tests

#  define GPIO_nLED_2_RED      /* PI0 */  (GPIO_OUTPUT|GPIO_OPENDRAIN|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTI|GPIO_PIN0)
#  define GPIO_nLED_2_GREEN    /* PH11 */ (GPIO_OUTPUT|GPIO_OPENDRAIN|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTH|GPIO_PIN11)
#  define GPIO_nLED_2_BLUE     /* PA2 */  (GPIO_OUTPUT|GPIO_OPENDRAIN|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTA|GPIO_PIN2)

#  define BOARD_HAS_CONTROL_STATUS_LEDS      1
#  define BOARD_OVERLOAD_LED     LED_RED
#  define BOARD_ARMED_STATE_LED  LED_BLUE

#else
#  if defined(CONFIG_STM32H7_UART5) && (GPIO_UART5_TX == GPIO_UART5_TX_3)
#    error Need to disable CONFIG_STM32H7_UART5 for Trace 3 (Retarget to CAN2 if need be)
#  endif
#endif


/* SPI */

#define GPIO_SPI4_MAG_INT           /* PD12 */   (GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI|GPIO_PORTD|GPIO_PIN12)

#define GPIO_SYNC                   /* PE9  */   (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_100MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTE|GPIO_PIN9)

/* I2C busses */

/* Devices on the onboard buses.
 *
 * Note that these are unshifted addresses.
 */
#define PX4_I2C_OBDEV_BMP388         0x76

#define GPIO_I2C4_DRDY1_BMP388      /* PG5  */  (GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI|GPIO_PORTG|GPIO_PIN5)

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
	/* PB1  */  GPIO_ADC12_INP5,   \
	/* PC2  */  GPIO_ADC123_INP12, \
	/* PC3  */  GPIO_ADC12_INP13,  \
	/* PH3  */  GPIO_ADC3_INP14,   \
	/* PH4  */  GPIO_ADC3_INP15

/* Define Channel numbers must match above GPIO pin IN(n)*/
#define ADC_SCALED_V5_CHANNEL                   /* PB1  */  ADC1_CH(5)
#define ADC_ADC3_6V6_CHANNEL                    /* PC2  */  ADC3_CH(12)
#define ADC_ADC3_3V3_CHANNEL                    /* PC3  */  ADC3_CH(13)
#define ADC_HW_VER_SENSE_CHANNEL                /* PH3  */  ADC3_CH(14)
#define ADC_HW_REV_SENSE_CHANNEL                /* PH4  */  ADC3_CH(15)

#define ADC_CHANNELS \
	((1 << ADC_SCALED_V5_CHANNEL)               | \
	 (1 << ADC_ADC3_6V6_CHANNEL)                | \
	 (1 << ADC_ADC3_3V3_CHANNEL))

/* HW has to large of R termination on ADC todo:change when HW value is chosen */

#define HW_REV_VER_ADC_BASE STM32_ADC3_BASE

#define SYSTEM_ADC_BASE STM32_ADC1_BASE

/* HW has to large of R termination on ADC todo:change when HW value is chosen */

#define BOARD_ADC_OPEN_CIRCUIT_V     (5.6f)

/* HW Version and Revision drive signals Default to 1 to detect */

#define BOARD_HAS_HW_VERSIONING

#define GPIO_HW_VER_REV_DRIVE  /* PG0 */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTG|GPIO_PIN0)
#define GPIO_HW_REV_SENSE      /* PH4 */  GPIO_ADC3_INP15
#define GPIO_HW_VER_SENSE      /* PH3 */  GPIO_ADC3_INP14
#define HW_INFO_INIT_PREFIX           "V2"
#define V20300   HW_VER_REV(0x3,0x0)

/* PE6 is nARMED --> FCv2 this goes to TP13
 *  The GPIO will be set as input while not armed HW will have external HW Pull UP.
 *  While armed it shall be configured at a GPIO OUT set LOW
 */
#define GPIO_nARMED_INIT     /* PE6 */  (GPIO_INPUT|GPIO_PULLUP|GPIO_PORTE|GPIO_PIN6)
#define GPIO_nARMED          /* PE6 */  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTE|GPIO_PIN6)

#define BOARD_INDICATE_ARMED_STATE(enabled)  px4_arch_configgpio((enabled) ? GPIO_nARMED : GPIO_nARMED_INIT)

/* PWM
 */
#define DIRECT_PWM_OUTPUT_CHANNELS   8


/* Power supply control and monitoring GPIOs */
#define GPIO_nVDD_USB_VALID             /* PF13 */ (GPIO_INPUT |GPIO_FLOAT|GPIO_PORTF|GPIO_PIN13) /* Low for USB power, High for DC power */

#define GPIO_VDD_3V3_SPEKTRUM_POWER_EN  /* PH2  */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTH|GPIO_PIN2)

/* Spare GPIO */

#define CAN1_SILENT                       /* PD15 */  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTD|GPIO_PIN15)

/* For primary/backup signaling with VOXL, 2 pins on J4 are exposed */
// GPIO_VOXL_STATUS_OUT/ GPIO_VOXL_STATUS_IN are for v1 Spare MSS Communications Interface and J4 tests

#define GPIO_VOXL_STATUS_OUT            /* PE4 */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTE|GPIO_PIN4)
#define GPIO_VOXL_STATUS_IN             /* PE3 */ (GPIO_INPUT|GPIO_FLOAT|GPIO_PORTE|GPIO_PIN3)

/* Define True logic Power Control in arch agnostic form */

#define VDD_3V3_SPEKTRUM_POWER_EN(on_true) px4_arch_gpiowrite(GPIO_VDD_3V3_SPEKTRUM_POWER_EN, (on_true))
#define READ_VDD_3V3_SPEKTRUM_POWER_EN()   px4_arch_gpioread(GPIO_VDD_3V3_SPEKTRUM_POWER_EN)

/* Future Use - IMU FSYNC */

#define IMU_FYSNC_TIMER        14  /* Timer 14 */
#define IMU_FYSNC_CHANNEL      1   /* PF9 GPIO_TIM14_CH1OUT_2 */

#define GPIO_IMU_FYSNC_1           /* PF9 */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTF|GPIO_PIN9)

#define GPIO_IMU_FYSNC_IDLE    GPIO_IMU_FYSNC_1
#define GPIO_IMU_FYSNC         GPIO_TIM14_CH1OUT_2

/* USB OTG FS
 *
 * PA9  OTG_FS_VBUS VBUS sensing
 */
#define GPIO_OTGFS_VBUS         /* PA9 */ (GPIO_INPUT|GPIO_PULLDOWN|GPIO_SPEED_100MHz|GPIO_PORTA|GPIO_PIN9)

/* High-resolution timer */
#define HRT_TIMER               8  /* use timer8 for the HRT */
#define HRT_TIMER_CHANNEL       3  /* use capture/compare channel 3 */

//#define HRT_PPM_CHANNEL         /* T8C1 */  1  /* use capture/compare channel 1 */
//#define GPIO_PPM_IN             /* PI5 T8C1 */ GPIO_TIM8_CH1IN_2


/* Input Capture Channels. */
#define INPUT_CAP1_TIMER                  1
#define INPUT_CAP1_CHANNEL     /* T1C2 */ 2
#define GPIO_INPUT_CAP1        /*  PE11 */ GPIO_TIM1_CH2IN

/* PWM input driver. Use FMU AUX5 pins attached to timer4 channel 2 */
#define PWMIN_TIMER                       4
#define PWMIN_TIMER_CHANNEL    /* T4C2 */ 2
#define GPIO_PWM_IN            /* PD13 */ GPIO_TIM4_CH2IN_2

/* Power switch controls ******************************************************/

#define SPEKTRUM_POWER(_on_true)           VDD_3V3_SPEKTRUM_POWER_EN(_on_true)

/*
 * ModalAI FC v2 has NO separate RC_IN
 *
 * SPEKTRUM_RX (it's TX or RX in Bind) on UART6 PC7
 *   Inversion is possible in the UART
 */

#define SPEKTRUM_RX_AS_OUT             (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTC|GPIO_PIN7)
#define SPEKTRUM_RX_AS_GPIO_OUTPUT()   px4_arch_configgpio(SPEKTRUM_RX_AS_OUT)
#define SPEKTRUM_RX_AS_UART()          /* Can be left as uart */
#define SPEKTRUM_OUT(_one_true)        px4_arch_gpiowrite(SPEKTRUM_RX_AS_OUT, (_one_true))

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
#define BOARD_ADC_USB_VALID     (!px4_arch_gpioread(GPIO_nVDD_USB_VALID))

/* ModalAI FC v2 never powers off the Servo rail */

#define BOARD_ADC_SERVO_VALID     (1)

#if !defined(BOARD_HAS_LTC44XX_VALIDS) || BOARD_HAS_LTC44XX_VALIDS == 0
#  define BOARD_ADC_BRICK1_VALID  (1)
#  define BOARD_ADC_BRICK2_VALID  (0)
#elif BOARD_HAS_LTC44XX_VALIDS == 1
#  define BOARD_ADC_BRICK1_VALID  (!px4_arch_gpioread(GPIO_nVDD_BRICK1_VALID))
#  define BOARD_ADC_BRICK2_VALID  (0)
#elif BOARD_HAS_LTC44XX_VALIDS == 2
#  define BOARD_ADC_BRICK1_VALID  (!px4_arch_gpioread(GPIO_nVDD_BRICK1_VALID))
#  define BOARD_ADC_BRICK2_VALID  (!px4_arch_gpioread(GPIO_nVDD_BRICK2_VALID))
#elif BOARD_HAS_LTC44XX_VALIDS == 3
#  define BOARD_ADC_BRICK1_VALID  (!px4_arch_gpioread(GPIO_nVDD_BRICK1_VALID))
#  define BOARD_ADC_BRICK2_VALID  (!px4_arch_gpioread(GPIO_nVDD_BRICK2_VALID))
#  define BOARD_ADC_BRICK3_VALID  (!px4_arch_gpioread(GPIO_nVDD_BRICK3_VALID))
#elif BOARD_HAS_LTC44XX_VALIDS == 4
#  define BOARD_ADC_BRICK1_VALID  (!px4_arch_gpioread(GPIO_nVDD_BRICK1_VALID))
#  define BOARD_ADC_BRICK2_VALID  (!px4_arch_gpioread(GPIO_nVDD_BRICK2_VALID))
#  define BOARD_ADC_BRICK3_VALID  (!px4_arch_gpioread(GPIO_nVDD_BRICK3_VALID))
#  define BOARD_ADC_BRICK4_VALID  (!px4_arch_gpioread(GPIO_nVDD_BRICK4_VALID))
#else
#  error Unsupported BOARD_HAS_LTC44XX_VALIDS value
#endif


/* This board provides a DMA pool and APIs */
#define BOARD_DMA_ALLOC_POOL_SIZE 5120

/* This board provides the board_on_reset interface */

#define BOARD_HAS_ON_RESET 1

#define PX4_GPIO_INIT_LIST { \
		PX4_ADC_GPIO,                     \
		GPIO_HW_VER_REV_DRIVE,            \
		GPIO_CAN1_TX,                     \
		GPIO_CAN1_RX,                     \
		GPIO_VDD_3V3_SPEKTRUM_POWER_EN,   \
		CAN1_SILENT,                      \
		GPIO_SYNC,                        \
		GPIO_IMU_FYSNC_IDLE,              \
		GPIO_IMU_FYSNC,                   \
	}

#define BOARD_ENABLE_CONSOLE_BUFFER

#define BOARD_NUM_IO_TIMERS 5

// J5 USART5 TELEM2 Port next to PWM connector
#define VOXL_ESC_DEFAULT_PORT                     "/dev/ttyS4"

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
