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
 * SIYI-UniFC-6-PICO internal definitions
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

/* SIYI-UniFC-6-PICO GPIOs ***********************************************************************************/

/* LEDs */
#  define GPIO_nLED_RED        /* PH2  */  (GPIO_OUTPUT|GPIO_OPENDRAIN|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTH|GPIO_PIN2)
#  define GPIO_nLED_GREEN      /* PH3  */  (GPIO_OUTPUT|GPIO_OPENDRAIN|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTH|GPIO_PIN3)
#  define GPIO_nLED_BLUE       /* PH4  */  (GPIO_OUTPUT|GPIO_OPENDRAIN|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTH|GPIO_PIN4)

#  define BOARD_HAS_CONTROL_STATUS_LEDS      1
#  define BOARD_OVERLOAD_LED     LED_RED
#  define BOARD_ARMED_STATE_LED  LED_BLUE

/* CAN Silence Silent mode control */
#define GPIO_CAN1_SILENT_S0  /* PG15  */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTG|GPIO_PIN15)
#define GPIO_CAN2_SILENT_S1  /* PA15  */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTA|GPIO_PIN15)


#define ADC1_CH(n)                  (n)

/* N.B. there is no offset mapping needed for ADC3 because */
#define ADC3_CH(n)                  (n)

#define PX4_ADC_GPIO  \
	/* PF12  */  GPIO_ADC1_INP6,   \
	/* PF11  */  GPIO_ADC1_INP2,   \
	/* PC2   */  GPIO_ADC123_INP12,   \
	/* PA5   */  GPIO_ADC12_INP19, \
	/* PF3   */  GPIO_ADC3_INP5, \
	/* PF4   */  GPIO_ADC3_INP9

/* Define Channel numbers must match above GPIO pin IN(n)*/

#define ADC_BATTERY_CURRENT_CHANNEL         /* PF11  */  2
#define ADC_SCALED_V5_CHANNEL               /* PC2 */    12
#define ADC_HW_REV_SENSE_CHANNEL            /* PF3 */    5
#define ADC_BATTERY_VOLTAGE_CHANNEL         /* PF12  */  6
#define ADC_HW_VER_SENSE_CHANNEL            /* PF4 */    9
#define ADC_SCALED_VDD_3V3_SENSORS1_CHANNEL /* PA5 */    19

#define ADC_CHANNELS \
	((1 << ADC_BATTERY_VOLTAGE_CHANNEL)       | \
	 (1 << ADC_BATTERY_CURRENT_CHANNEL)       | \
	 (1 << ADC_SCALED_V5_CHANNEL)              | \
	 (1 << ADC_SCALED_VDD_3V3_SENSORS1_CHANNEL) | \
	 (1 << ADC_HW_VER_SENSE_CHANNEL)           | \
	 (1 << ADC_HW_REV_SENSE_CHANNEL))

/* HW has to large of R termination on ADC todo:change when HW value is chosen */

#define HW_REV_VER_ADC_BASE STM32_ADC3_BASE

#define SYSTEM_ADC_BASE STM32_ADC1_BASE
#define BOARD_ADC_OPEN_CIRCUIT_V     (5.6f)



#define GPIO_HW_REV_DRIVE    /* PH5  */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTH|GPIO_PIN5)
#define GPIO_HW_REV_SENSE    /* PF3   */ GPIO_ADC3_INP5
#define GPIO_HW_VER_DRIVE    /* PF5   */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTF|GPIO_PIN5)
#define GPIO_HW_VER_SENSE    /* PF4   */ GPIO_ADC3_INP9




/* HEATER
 * PWM in future
 */
#define GPIO_HEATER_OUTPUT   /* PG7   */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTG|GPIO_PIN7)
#define HEATER_OUTPUT_EN(on_true)              px4_arch_gpiowrite(GPIO_HEATER_OUTPUT, (on_true))


/* PWM
 */
#define DIRECT_PWM_OUTPUT_CHANNELS   14

/* PWM Power */
#define GPIO_PWM_LEVEL_CONTROL		/* PE12  */ (GPIO_OUTPUT|GPIO_PULLUP|GPIO_PORTE|GPIO_PIN12)


/* Power supply control and monitoring GPIOs */

#define GPIO_VDD_3V3_SPI1_BMI088_EN	/* PE10   */     (GPIO_OUTPUT|GPIO_PULLUP|GPIO_PORTE|GPIO_PIN10)
#define GPIO_VDD_3V3_SPI4_ICM45686_EN	/* PG0   */     (GPIO_OUTPUT|GPIO_PULLUP|GPIO_PORTG|GPIO_PIN0)
#define GPIO_VDD_3V3_ICP20100_1_EN     /* PI4   */      (GPIO_INPUT|GPIO_PULLUP|GPIO_PORTI|GPIO_PIN4)
#define GPIO_VDD_5V_PERIPH_nOC          /* PI10 */ (GPIO_INPUT |GPIO_FLOAT|GPIO_PORTI|GPIO_PIN10)
#define GPIO_VDD_5V_HIPOWER_nOC         /* PF2 */ (GPIO_INPUT |GPIO_FLOAT|GPIO_PORTF|GPIO_PIN2)
#define GPIO_VDD_3V3_ICP20100_2_EN	/* PI11  */      (GPIO_OUTPUT|GPIO_PULLUP|GPIO_PORTI|GPIO_PIN11)
#define GPIO_VDD_3V3_IST8310_EN		/* PG8  */       (GPIO_OUTPUT|GPIO_PULLUP|GPIO_PORTG|GPIO_PIN8)
#define GPIO_VDD_3V3_SD_CARD_EN		/* PB1   */      (GPIO_INPUT|GPIO_PULLUP|GPIO_PORTB|GPIO_PIN1)

/* Define True logic Power Control in arch agnostic form */

#define VDD_5V_PWM_EN(on_true)		        px4_arch_gpiowrite(GPIO_PWM_LEVEL_CONTROL, (on_true))
#define VDD_3V3_SPI4_45686_EN(on_true)		px4_arch_gpiowrite(GPIO_VDD_3V3_SPI4_ICM45686_EN, (on_true))
#define VDD_3V3_SPI1_BMI088_EN(on_true)		px4_arch_gpiowrite(GPIO_VDD_3V3_SPI1_BMI088_EN, (on_true))
#define VDD_3V3_SD_CARD_EN(on_true)		px4_arch_gpiowrite(GPIO_VDD_3V3_SD_CARD_EN, (on_true))
#define VDD_3V3_ICP20100_1_EN(on_true)		px4_arch_gpiowrite(GPIO_VDD_3V3_ICP20100_1_EN, (on_true))
#define VDD_3V3_ICP20100_2_EN(on_true)		px4_arch_gpiowrite(GPIO_VDD_3V3_ICP20100_2_EN, (on_true))
#define VDD_3V3_IST8310_EN(on_true)		px4_arch_gpiowrite(GPIO_VDD_3V3_IST8310_EN, (on_true))

/* USB OTG FS
 *
 * PA9  OTG_FS_VBUS VBUS sensing
 */
#define GPIO_OTGFS_VBUS         /* PA9 */ (GPIO_INPUT|GPIO_PULLDOWN|GPIO_SPEED_100MHz|GPIO_PORTA|GPIO_PIN9)

/* High-resolution timer */
#define HRT_TIMER               8  /* use timer12 for the HRT */
#define HRT_TIMER_CHANNEL       3/* use capture/compare channel 3 */

#define HRT_PPM_CHANNEL         /* T8C1 */  1  /* use capture/compare channel 1 */
#define GPIO_PPM_IN       	/* PI5 T8C1 */ GPIO_TIM8_CH1IN_2
/* RC Serial port */

#define RC_SERIAL_PORT          "/dev/ttyS5"

#define TONE_ALARM_TIMER        16  /* Timer 16 */
#define TONE_ALARM_CHANNEL      1  /* PF6 GPIO_TIM16_CH1OUT_2 */

#define GPIO_BUZZER_1           /* PF6 */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTF|GPIO_PIN6)

#define GPIO_TONE_ALARM_IDLE    GPIO_BUZZER_1
#define GPIO_TONE_ALARM         GPIO_TIM16_CH1OUT_2



/* Safety Switch is HW version dependent on having an PX4IO
 * So we init to a benign state with the _INIT definition
 * and provide the the non _INIT one for the driver to make a run time
 * decision to use it.
 */

#define GPIO_nSAFETY_SWITCH_LED_OUT        /* PD10 */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTD|GPIO_PIN10)
/* Enable the FMU to control it if there is no px4io fixme:This should be BOARD_SAFETY_LED(__ontrue) */
#define GPIO_LED_SAFETY GPIO_nSAFETY_SWITCH_LED_OUT

#define GPIO_SAFETY_SWITCH_IN              /* PD11 */ (GPIO_INPUT|GPIO_PULLDOWN|GPIO_PORTD|GPIO_PIN11)
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
		GPIO_CAN1_TX,				\
		GPIO_CAN1_RX,                   	\
		GPIO_CAN2_TX,                   	\
		GPIO_CAN2_RX,				\
		GPIO_CAN1_SILENT_S0,			\
		GPIO_CAN2_SILENT_S1,			\
		GPIO_HEATER_OUTPUT,                     \
		GPIO_VDD_3V3_SPI4_ICM45686_EN,		\
		GPIO_VDD_3V3_SPI1_BMI088_EN,		\
		GPIO_VDD_3V3_ICP20100_1_EN,		\
		GPIO_VDD_3V3_ICP20100_2_EN,		\
		GPIO_VDD_3V3_IST8310_EN,                \
		GPIO_VDD_3V3_SD_CARD_EN,		\
		GPIO_nSAFETY_SWITCH_LED_OUT,		\
		GPIO_SAFETY_SWITCH_IN,			\
		GPIO_HW_REV_DRIVE,			\
		GPIO_HW_VER_DRIVE,			\
		GPIO_VDD_5V_PERIPH_nOC,			\
		GPIO_VDD_5V_HIPOWER_nOC,		\
		GPIO_TONE_ALARM_IDLE,                   \
		GPIO_PWM_LEVEL_CONTROL,			\
	}




#define BOARD_ENABLE_CONSOLE_BUFFER

#define PX4_I2C_BUS_MTD     1


#define BOARD_NUM_IO_TIMERS 8

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
