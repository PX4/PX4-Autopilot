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
 * CUAV 7-Nano internal definitions
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

// /* Configuration ************************************************************************************/

/* 7-Nano GPIOs ***********************************************************************************/

/* LEDs */
#  define GPIO_nLED_RED        /* PE3 */  (GPIO_OUTPUT|GPIO_OPENDRAIN|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTE|GPIO_PIN3)
#  define GPIO_nLED_GREEN      /* PE4 */  (GPIO_OUTPUT|GPIO_OPENDRAIN|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTE|GPIO_PIN4)
#  define GPIO_nLED_BLUE       /* PE5 */  (GPIO_OUTPUT|GPIO_OPENDRAIN|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTE|GPIO_PIN5)

#  define BOARD_HAS_CONTROL_STATUS_LEDS      1
#  define BOARD_OVERLOAD_LED     LED_RED
#  define BOARD_ARMED_STATE_LED  LED_BLUE

/* CAN Silence Silent mode control */
#define GPIO_CAN1_SILENT_S0  /* PE2  */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTE|GPIO_PIN2)
#define GPIO_CAN2_SILENT_S1  /* PI8  */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTI|GPIO_PIN8)

/* SPI */

#define SPI6_nRESET_EXTERNAL1		/* PF10 */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTF|GPIO_PIN10)

/* I2C busses */
#define BOARD_MTD_NUM_EEPROM        1 /* imu_eeprom */

#define GPIO_I2C4_DRDY1_ICP20100      /* PG5  */  (GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI|GPIO_PORTG|GPIO_PIN5)

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
	/* PC0	*/  GPIO_ADC123_INP10, \
	/* PC2  */  GPIO_ADC123_INP12, \
	/* PC3  */  GPIO_ADC12_INP13,  \
	/* PF12 */  GPIO_ADC1_INP6,    \
	/* PH3  */  GPIO_ADC3_INP14,   \
	/* PH4  */  GPIO_ADC3_INP15

/* Define Channel numbers must match above GPIO pin IN(n)*/
#define ADC_SCALED_VDD_3V3_SENSORS1_CHANNEL     /* PA0  */  ADC1_CH(16)
#define ADC_SCALED_VDD_3V3_SENSORS2_CHANNEL     /* PA4  */  ADC1_CH(18)
#define ADC_BATTERY_CURRENT_CHANNEL		/* PB0	*/  ADC1_CH(9)
#define ADC_SCALED_V5_CHANNEL			/* PB1  */  ADC1_CH(5)
#define ADC_RSSI_IN_CHANNEL			/* PC0  */  ADC1_CH(10)
#define ADC_BATTERY_VOLTAGE_CHANNEL		/* PC2  */  ADC1_CH(12)
#define ADC_ADC3_3V3_CHANNEL             	/* PC3  */  ADC1_CH(13)
#define ADC_SERVO_VDD_SENSORS_CHANNEL           /* PF12 */  ADC1_CH(6)
#define ADC_HW_VER_SENSE_CHANNEL                /* PH3  */  ADC3_CH(14)
#define ADC_HW_REV_SENSE_CHANNEL                /* PH4  */  ADC3_CH(15)

#define ADC_CHANNELS \
	((1 << ADC_SCALED_VDD_3V3_SENSORS1_CHANNEL) | \
	 (1 << ADC_SCALED_VDD_3V3_SENSORS2_CHANNEL) | \
	 (1 << ADC_BATTERY_CURRENT_CHANNEL)         | \
	 (1 << ADC_SCALED_V5_CHANNEL)               | \
	 (1 << ADC_RSSI_IN_CHANNEL)		    | \
	 (1 << ADC_BATTERY_VOLTAGE_CHANNEL)         | \
	 (1 << ADC_ADC3_3V3_CHANNEL)                | \
	 (1 << ADC_SERVO_VDD_SENSORS_CHANNEL))


/* HW has to large of R termination on ADC todo:change when HW value is chosen */

#define HW_REV_VER_ADC_BASE STM32_ADC3_BASE

#define SYSTEM_ADC_BASE STM32_ADC1_BASE

/* HW has to large of R termination on ADC todo:change when HW value is chosen */
#define BOARD_ADC_OPEN_CIRCUIT_V     (5.6f)

#define GPIO_HW_VER_REV_DRIVE  /* PG0 */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTG|GPIO_PIN0)
#define GPIO_HW_REV_SENSE      /* PH4 */  GPIO_ADC3_INP15
#define GPIO_HW_VER_SENSE      /* PH3 */  GPIO_ADC3_INP14

// #define UAVCAN_NUM_IFACES_RUNTIME  1

/* PE7 is nARMED
 *  The GPIO will be set as input while not armed HW will have external HW Pull UP.
 *  While armed it shall be configured at a GPIO OUT set LOW
 */
#if !defined(TRACE_PINS)
#define GPIO_nARMED_INIT     /* PE7 */  (GPIO_INPUT|GPIO_PULLUP|GPIO_PORTE|GPIO_PIN7)
#define GPIO_nARMED          /* PE7 */  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTE|GPIO_PIN7)
#define BOARD_INDICATE_EXTERNAL_LOCKOUT_STATE(enabled)  px4_arch_configgpio((enabled) ? GPIO_nARMED : GPIO_nARMED_INIT)
#define BOARD_GET_EXTERNAL_LOCKOUT_STATE() px4_arch_gpioread(GPIO_nARMED)
#endif


/* PWM
 */
#define DIRECT_PWM_OUTPUT_CHANNELS   14

/* PWM Power */
#define GPIO_PWM_LEVEL_CONTROL		/* PG8  */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTG|GPIO_PIN8)
#define VDD_5V_PWM_EN(on_true)		px4_arch_gpiowrite(GPIO_PWM_LEVEL_CONTROL, (on_true))

/* Power supply control and monitoring GPIOs */

#define GPIO_VDD_3V3_SD_CARD_EN		/* PC13 */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTC|GPIO_PIN13)
#define GPIO_VDD_5V_PERIPH_nOC		/* PE15 */ (GPIO_INPUT|GPIO_PULLUP|GPIO_PORTE|GPIO_PIN15)
#define GPIO_VDD_5V_HIPOWER_nOC		/* PF13 */ (GPIO_INPUT|GPIO_PULLUP|GPIO_PORTF|GPIO_PIN13)
#define GPIO_VDD_5V_PERIPH_EN		/* PG4  */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTG|GPIO_PIN4)
#define GPIO_VDD_PWM_POWER_EN		/* PH2  */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTH|GPIO_PIN2)
#define GPIO_VDD_5V_HIPOWER_EN		/* PG10 */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET| GPIO_PORTG|GPIO_PIN10)

/* ETHERNET GPIO */

#define GPIO_ETH_POWER_EN              /* PG15 */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTG|GPIO_PIN15)

/* Define True logic Power Control in arch agnostic form */

#define VDD_3V3_SD_CARD_EN(on_true)		px4_arch_gpiowrite(GPIO_VDD_3V3_SD_CARD_EN, (on_true))
#define VDD_5V_PERIPH_EN(on_true)		px4_arch_gpiowrite(GPIO_VDD_5V_PERIPH_EN, (on_true))
#define VDD_PWM_POWER_EN(on_true)		px4_arch_gpiowrite(GPIO_VDD_PWM_POWER_EN, (on_true))
#define VDD_5V_HIPOWER_EN(on_true)		px4_arch_gpiowrite(GPIO_VDD_5V_HIPOWER_EN, (on_true))
#define VDD_3V3_ETH_POWER_EN(on_true)	        px4_arch_gpiowrite(GPIO_ETH_POWER_EN, (on_true))



/* Tone alarm output */

#define TONE_ALARM_TIMER        14  /* Timer 14 */
#define TONE_ALARM_CHANNEL      1  /* PF9 GPIO_TIM14_CH1OUT_2 */

#define GPIO_BUZZER_1           /* PF9 */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTF|GPIO_PIN9)

#define GPIO_TONE_ALARM_IDLE    GPIO_BUZZER_1
#define GPIO_TONE_ALARM         GPIO_TIM14_CH1OUT_2

/* USB OTG FS
 *
 * PA9  OTG_FS_VBUS VBUS sensing
 */
#define GPIO_OTGFS_VBUS         /* PA9 */ (GPIO_INPUT|GPIO_PULLDOWN|GPIO_SPEED_100MHz|GPIO_PORTA|GPIO_PIN9)

/* High-resolution timer */
#define HRT_TIMER               3  /* use timer8 for the HRT */
#define HRT_TIMER_CHANNEL       2 /* use capture/compare channel 3 */

#define HRT_PPM_CHANNEL         /* T3C1 */  1  /* use capture/compare channel 1 */
#define GPIO_PPM_IN       	/* PC6 T8C1 */ GPIO_TIM3_CH1IN_3

/* RC Serial port */

#define RC_SERIAL_PORT                     "/dev/ttyS4"
#define RC_SERIAL_SINGLEWIRE

/* Input Capture Channels. */
#define INPUT_CAP1_TIMER	8
#define INPUT_CAP1_CHANNEL	2 /* T8C2 */
#define GPIO_INPUT_CAP1		GPIO_TIM8_CH2IN_2 /* PI6 */

#define INPUT_CAP2_TIMER	8
#define INPUT_CAP2_CHANNEL	3 /* T8C3 */
#define GPIO_INPUT_CAP2		GPIO_TIM8_CH3IN_2 /* PI7 */

#define INPUT_CAP3_TIMER	8
#define INPUT_CAP3_CHANNEL	1 /* T8C1 */
#define GPIO_INPUT_CAP3		GPIO_TIM8_CH1IN_2 /* PI5 */

#define INPUT_CAP4_TIMER	15
#define INPUT_CAP4_CHANNEL	2 /* T4C1 */
#define GPIO_INPUT_CAP4		GPIO_TIM4_CH1IN_2 /* PD12 */

#define INPUT_CAP5_TIMER	12
#define INPUT_CAP5_CHANNEL	1 /* T12C1 */
#define GPIO_INPUT_CAP5		GPIO_TIM12_CH1IN_2 /* PH6 */

#define INPUT_CAP6_TIMER	12
#define INPUT_CAP6_CHANNEL	2 /* T12C2 */
#define GPIO_INPUT_CAP6		GPIO_TIM12_CH2IN_2 /* PH9 */

/* PWM input driver. Use FMU AUX5 pins attached to timer4 channel 2 */
#define PWMIN_TIMER                       4
#define PWMIN_TIMER_CHANNEL    /* T4C2 */ 2
#define GPIO_PWM_IN            /* PD13 */ GPIO_TIM4_CH2IN_2

/* Safety Switch is HW version dependent on having an PX4IO
 * So we init to a benign state with the _INIT definition
 * and provide the the non _INIT one for the driver to make a run time
 * decision to use it.
 */
// #define GPIO_nSAFETY_SWITCH_LED_OUT_INIT   /* PD10 */ (GPIO_INPUT|GPIO_FLOAT|GPIO_PORTD|GPIO_PIN10)
#define GPIO_nSAFETY_SWITCH_LED_OUT        /* PD10 */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTD|GPIO_PIN10)

/* Enable the FMU to control it if there is no px4io fixme:This should be BOARD_SAFETY_LED(__ontrue) */
#define GPIO_LED_SAFETY GPIO_nSAFETY_SWITCH_LED_OUT

#define GPIO_SAFETY_SWITCH_IN              /* PF5 */ (GPIO_INPUT|GPIO_PULLUP|GPIO_PORTF|GPIO_PIN5)
/* Enable the FMU to use the switch it if there is no px4io fixme:This should be BOARD_SAFTY_BUTTON() */
#define GPIO_BTN_SAFETY GPIO_SAFETY_SWITCH_IN /* Enable the FMU to control it if there is no px4io */

/* Power switch controls ******************************************************/

// #define SPEKTRUM_POWER(_on_true)           VDD_3V3_SPEKTRUM_POWER_EN(_on_true)

/*
 * 7-nano has a separate RC_IN
 *
 * GPIO PPM_IN on PI5 T8CH1
 * SPEKTRUM_RX (it's TX or RX in Bind) on UART6 PC7
 *   Inversion is possible in the UART and can drive  GPIO PPM_IN as an output
 */

#define GPIO_PPM_IN_AS_OUT             (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTC|GPIO_PIN6)
#define SPEKTRUM_RX_AS_GPIO_OUTPUT()   px4_arch_configgpio(GPIO_PPM_IN_AS_OUT)
#define SPEKTRUM_RX_AS_UART()          /* Can be left as uart */
#define SPEKTRUM_OUT(_one_true)        px4_arch_gpiowrite(GPIO_PPM_IN_AS_OUT, (_one_true))

// /* RSSI_IN */
#define GPIO_RSSI_IN                   /* PC0  */ (GPIO_INPUT|GPIO_PULLUP|GPIO_PORTC|GPIO_PIN0)

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

#define BOARD_ADC_BRICK_VALID          1
#define BOARD_NUMBER_BRICKS            1

#define BOARD_ADC_PERIPH_5V_OC  (!px4_arch_gpioread(GPIO_VDD_5V_PERIPH_nOC))
#define BOARD_ADC_HIPOWER_5V_OC (!px4_arch_gpioread(GPIO_VDD_5V_HIPOWER_nOC))


/* This board provides a DMA pool and APIs */
#define BOARD_DMA_ALLOC_POOL_SIZE 5120

/* This board provides the board_on_reset interface */

#define BOARD_HAS_ON_RESET 1

#define PX4_GPIO_INIT_LIST { \
		PX4_ADC_GPIO,                   	\
		GPIO_HW_VER_REV_DRIVE,          	\
		GPIO_CAN1_TX,				\
		GPIO_CAN1_RX,                   	\
		GPIO_CAN2_TX,                   	\
		GPIO_CAN2_RX,				\
		GPIO_CAN1_SILENT_S0,			\
		GPIO_CAN2_SILENT_S1,			\
		GPIO_VDD_3V3_SD_CARD_EN,		\
		GPIO_VDD_5V_PERIPH_nOC,			\
		GPIO_VDD_5V_HIPOWER_nOC,		\
		GPIO_VDD_5V_PERIPH_EN,			\
		GPIO_VDD_PWM_POWER_EN,			\
		GPIO_VDD_5V_HIPOWER_EN,			\
		GPIO_ETH_POWER_EN,			\
		SPI6_nRESET_EXTERNAL1,			\
		GPIO_TONE_ALARM_IDLE,			\
		GPIO_nSAFETY_SWITCH_LED_OUT,	        \
		GPIO_SAFETY_SWITCH_IN,			\
		GPIO_nARMED,				\
	}

#define BOARD_ENABLE_CONSOLE_BUFFER

#define PX4_I2C_BUS_MTD     1


#define BOARD_NUM_IO_TIMERS 6

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
