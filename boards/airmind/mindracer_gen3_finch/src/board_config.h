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

#include <px4_platform_common/px4_config.h>
#include <nuttx/compiler.h>
#include <stdint.h>

#include <stm32_gpio.h>

/****************************************************************************************************
 * Definitions
 ****************************************************************************************************/

/* Configuration ************************************************************************************/

//#define BOARD_HAS_LTC4417

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

/* PX4FMU GPIOs ***********************************************************************************/

/* LEDs are driven with push open drain to support Anode to 5V or 3.3V */

#define GPIO_nLED_RED        /* PB1 */  (GPIO_OUTPUT|GPIO_OPENDRAIN|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTB|GPIO_PIN1)
#define GPIO_nLED_GREEN      /* PC6 */  (GPIO_OUTPUT|GPIO_OPENDRAIN|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTC|GPIO_PIN6)
#define GPIO_nLED_BLUE       /* PC7 */  (GPIO_OUTPUT|GPIO_OPENDRAIN|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTC|GPIO_PIN7)

//#define BOARD_HAS_CONTROL_STATUS_LEDS      1
#define BOARD_OVERLOAD_LED     LED_RED
#define BOARD_ARMED_STATE_LED  LED_BLUE

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
	/* PA0 */  ADC1_GPIO(0),  \
	/* PA2 */  ADC1_GPIO(2),  \
	/* PA4 */  ADC1_GPIO(4),  \
	/* PA5 */  ADC1_GPIO(5),  \
	/* PA6 */  ADC1_GPIO(6),  \
	/* PB0 */  ADC1_GPIO(8),  \
	/* PC0 */  ADC1_GPIO(10), \
	/* PC1 */  ADC1_GPIO(11)

/* Board 5V supply monitor*/
#define ADC_SCALED_V5_CHANNEL               /* PA0 */  ADC1_CH(0)

/* Define Channel numbers must match above GPIO pin IN(n)*/
#define ADC_BATTERY_VOLTAGE_CHANNEL        /* PA5 */  ADC1_CH(5)
#define ADC_BATTERY_CURRENT_CHANNEL        /* PB0 */  ADC1_CH(8)

#define ADC_BATTERY1_VOLTAGE_CHANNEL        /* PA5 */  ADC1_CH(5)
#define ADC_BATTERY1_CURRENT_CHANNEL        /* PB0 */  ADC1_CH(8)
//#define ADC_BATTERY2_VOLTAGE_CHANNEL        /* PA2 */  ADC1_CH(2)
//#define ADC_BATTERY2_CURRENT_CHANNEL        /* PA3 */  ADC1_CH(3)
#define ADC1_SPARE_2_CHANNEL                /* PC0 */  ADC1_CH(10)
#define ADC_RSSI_IN_CHANNEL                 /* PA4 */  ADC1_CH(4)
//#define ADC_SCALED_V5_CHANNEL               /* PC0 */  ADC1_CH(10)
//#define ADC_SCALED_VDD_3V3_SENSORS_CHANNEL  /* PC1 */  ADC1_CH(11)
#define ADC_HW_VER_SENSE_CHANNEL            /* PA2 */  ADC1_CH(2)
#define ADC_HW_REV_SENSE_CHANNEL            /* PC1 */  ADC1_CH(11)

#define SYSTEM_ADC_BASE STM32_ADC1_BASE

#if BOARD_HAS_NBAT_V == 2 && BOARD_HAS_NBAT_I == 2
#define ADC_CHANNELS \
	((1 << ADC_BATTERY1_VOLTAGE_CHANNEL)       | \
	 (1 << ADC_BATTERY1_CURRENT_CHANNEL)       | \
	 (1 << ADC_BATTERY2_VOLTAGE_CHANNEL)       | \
	 (1 << ADC_BATTERY2_CURRENT_CHANNEL)       | \
	 (1 << ADC1_SPARE_2_CHANNEL)               | \
	 (1 << ADC_RSSI_IN_CHANNEL)                | \
	 (1 << ADC_HW_VER_SENSE_CHANNEL)           | \
	 (1 << ADC_HW_REV_SENSE_CHANNEL)           | \
	 (1 << ADC_SCALED_V5_CHANNEL))
#elif BOARD_HAS_NBAT_V == 1 && BOARD_HAS_NBAT_I == 1
#define ADC_CHANNELS \
	((1 << ADC_BATTERY1_VOLTAGE_CHANNEL)       | \
	 (1 << ADC_BATTERY1_CURRENT_CHANNEL)       | \
	 (1 << ADC1_SPARE_2_CHANNEL)               | \
	 (1 << ADC_RSSI_IN_CHANNEL)                | \
	 (1 << ADC_HW_VER_SENSE_CHANNEL)           | \
	 (1 << ADC_HW_REV_SENSE_CHANNEL)           | \
	 (1 << ADC_SCALED_V5_CHANNEL))
#elif BOARD_HAS_NBAT_V == 1 && BOARD_HAS_NBAT_I == 0
#define ADC_CHANNELS \
	((1 << ADC_BATTERY1_VOLTAGE_CHANNEL)       | \
	 (1 << ADC1_SPARE_2_CHANNEL)               | \
	 (1 << ADC_RSSI_IN_CHANNEL)                | \
	 (1 << ADC_HW_VER_SENSE_CHANNEL)           | \
	 (1 << ADC_HW_REV_SENSE_CHANNEL)           | \
	 (1 << ADC_SCALED_V5_CHANNEL))
#endif

/* HW has to large of R termination on ADC todo:change when HW value is chosen */

#define BOARD_ADC_OPEN_CIRCUIT_V     (5.6f)

/* HW Version and Revision drive signals Default to 1 to detect */

#define BOARD_HAS_HW_VERSIONING
/* HW Ver/Rev drive use the same pin **/
#define GPIO_HW_VER_REV_DRIVE    /* PB4  */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTB|GPIO_PIN4)
#define GPIO_HW_REV_SENSE    /* PC1   */ ADC1_GPIO(11)
#define GPIO_HW_VER_SENSE    /* PA2   */ ADC1_GPIO(2)
#define HW_INFO_INIT_PREFIX         "MINDRACER_GEN3_FINCH"

#define MINDRACERGEN300   HW_VER_REV(0x0,0x0) // Mindracer gen.3 f4           Rev 0
#define MINDRACERGEN310   HW_VER_REV(0x1,0x0) // Mindracer gen.3 f7

/* CAN Silence
 *
 * Silent mode control \ ESC Mux select
 */

//#define GPIO_CAN1_SILENT_S0  /* PH2  */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTH|GPIO_PIN2)
//#define GPIO_CAN2_SILENT_S1  /* PH3  */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTH|GPIO_PIN3)
//#define GPIO_CAN3_SILENT_S2  /* PH4  */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTH|GPIO_PIN4)

#define UAVCAN_NUM_IFACES_RUNTIME 1

/* HEATER
 * PWM in future
 */
#define GPIO_HEATER_OUTPUT   /* PA8  T1CH1 */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTA|GPIO_PIN8)
#define HEATER_OUTPUT_EN(on_true)              px4_arch_gpiowrite(GPIO_HEATER_OUTPUT, (on_true))

/* PI0 is nARMED
 *  The GPIO will be set as input while not armed HW will have external HW Pull UP.
 *  While armed it shall be configured at a GPIO OUT set LOW
 */
#define GPIO_nARMED_INIT     /* PE15 */  (GPIO_INPUT|GPIO_PULLUP|GPIO_PORTE|GPIO_PIN15)
#define GPIO_nARMED          /* PE15 */  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTE|GPIO_PIN15)

#define BOARD_INDICATE_EXTERNAL_LOCKOUT_STATE(enabled)  px4_arch_configgpio((enabled) ? GPIO_nARMED : GPIO_nARMED_INIT)

/* PWM
 */
#define DIRECT_PWM_OUTPUT_CHANNELS  15

//#define BOARD_HAS_LED_PWM              1
//#define BOARD_LED_PWM_DRIVE_ACTIVE_LOW 1

//#define BOARD_HAS_UI_LED_PWM            1

//#define BOARD_UI_LED_PWM_DRIVE_ACTIVE_LOW 1


/* Power supply control and monitoring GPIOs */

#define GPIO_nPOWER_IN_A                /* PD10  */ (GPIO_INPUT|GPIO_PULLUP|GPIO_PORTD|GPIO_PIN10)
//#define GPIO_nPOWER_IN_B                /* PG2  */ (GPIO_INPUT|GPIO_PULLUP|GPIO_PORTG|GPIO_PIN2)
//#define GPIO_nPOWER_IN_C                /* PB4  */ (GPIO_INPUT|GPIO_PULLUP|GPIO_PORTB|GPIO_PIN4)

#define GPIO_nVDD_BRICK1_VALID          GPIO_nPOWER_IN_A /* Brick 1 Is Chosen */
//#define GPIO_nVDD_BRICK2_VALID          GPIO_nPOWER_IN_B /* Brick 2 Is Chosen  */
#define BOARD_NUMBER_BRICKS             1
//#define GPIO_nVDD_USB_VALID             GPIO_nPOWER_IN_C /* USB     Is Chosen */

/* By Providing BOARD_ADC_USB_CONNECTED (using the px4_arch abstraction)
 * this board support the ADC system_power interface, and therefore
 * provides the true logic GPIO BOARD_ADC_xxxx macros.
 */
#define BOARD_ADC_USB_CONNECTED (px4_arch_gpioread(GPIO_OTGFS_VBUS))

#if BOARD_HAS_USB_VALID == 1
#  define BOARD_ADC_USB_VALID     (!px4_arch_gpioread(GPIO_nVDD_USB_VALID))
#else
#  define BOARD_ADC_USB_VALID     BOARD_ADC_USB_CONNECTED
#endif

#if 0
#define GPIO_nVDD_5V_PERIPH_EN          /* PG4  */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTG|GPIO_PIN4)
#define GPIO_nVDD_5V_PERIPH_OC          /* PE15 */ (GPIO_INPUT |GPIO_FLOAT|GPIO_PORTE|GPIO_PIN15)
#define GPIO_nVDD_5V_HIPOWER_EN         /* PF12 */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTF|GPIO_PIN12)
#define GPIO_nVDD_5V_HIPOWER_OC         /* PG13 */ (GPIO_INPUT |GPIO_FLOAT|GPIO_PORTF|GPIO_PIN13)
#define GPIO_VDD_5V_RC_EN               /* PG5  */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTG|GPIO_PIN5)
#define GPIO_VDD_5V_WIFI_EN             /* PG6  */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTG|GPIO_PIN5)

/* Define True logic Power Control in arch agnostic form */

#define VDD_5V_PERIPH_EN(on_true)          px4_arch_gpiowrite(GPIO_nVDD_5V_PERIPH_EN, !(on_true))
#define VDD_5V_HIPOWER_EN(on_true)         px4_arch_gpiowrite(GPIO_nVDD_5V_HIPOWER_EN, !(on_true))
#define VDD_5V_RC_EN(on_true)              px4_arch_gpiowrite(GPIO_VDD_5V_RC_EN, (on_true))
#define VDD_5V_WIFI_EN(on_true)            px4_arch_gpiowrite(GPIO_VDD_5V_WIFI_EN, (on_true))
#endif

#define GPIO_VDD_3V3_SD_CARD_EN         /* PD6  */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTD|GPIO_PIN6)
#define VDD_3V3_SD_CARD_EN(on_true)        px4_arch_gpiowrite(GPIO_VDD_3V3_SD_CARD_EN, (on_true))


#define GPIO_VDD_3V3_SPEKTRUM_POWER_EN  /* PD7  */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTD|GPIO_PIN7)
#define READ_VDD_3V3_SPEKTRUM_POWER_EN()   px4_arch_gpioread(GPIO_VDD_3V3_SPEKTRUM_POWER_EN)
#define VDD_3V3_SPEKTRUM_POWER_EN(on_true) px4_arch_gpiowrite(GPIO_VDD_3V3_SPEKTRUM_POWER_EN, (on_true))

/* Tone alarm output */

#define TONE_ALARM_TIMER        13  /* timer 13 */
#define TONE_ALARM_CHANNEL      1  /* PA6 TIM13_CH1 */

#define GPIO_BUZZER_1           /* PA3 */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTA|GPIO_PIN6)

#define GPIO_TONE_ALARM_IDLE    GPIO_BUZZER_1
#define GPIO_TONE_ALARM         GPIO_TIM13_CH1OUT_1

/* USB OTG FS
 *
 * PA9  OTG_FS_VBUS VBUS sensing
 */
#define GPIO_OTGFS_VBUS         /* PA9 */ (GPIO_INPUT|GPIO_PULLDOWN|GPIO_SPEED_100MHz|GPIO_PORTA|GPIO_PIN9)

/* High-resolution timer */
#define HRT_TIMER               5  /* use timer5 for the HRT */
#define HRT_TIMER_CHANNEL       1  /* use capture/compare channel 4 */

#if 0
#define HRT_PPM_CHANNEL         /* T5CH4 */  4  /* use capture/compare channel 4 */
#define GPIO_PPM_IN             /* PA3 T5CH4 */ GPIO_TIM5_CH4IN_1
#endif

/* RC Serial port */
#define RC_SERIAL_PORT                     "/dev/ttyS1"
//#define RC_SERIAL_SINGLEWIRE

/* PWM input driver. Use FMU AUX5 pins attached to timer4 channel 2 */
#define PWMIN_TIMER                       4
#define PWMIN_TIMER_CHANNEL    /* T4C2 */ 2
#define GPIO_PWM_IN            /* PD13 */ GPIO_TIM4_CH2IN_2

/* Shared pins Both FMU and PX4IO control/monitor
 * FMU Initializes these pins to passive input until it is known
 * if we have and PX4IO on board
 */

#define GPIO_RSSI_IN                       /* PA4  */ (GPIO_INPUT|GPIO_PULLUP|GPIO_PORTA|GPIO_PIN4)
#define GPIO_RSSI_IN_INIT                  /* PA4  */ 0 /* Leave as ADC RSSI_IN */


/* Power switch controls ******************************************************/

#define SPEKTRUM_POWER(_on_true)           VDD_3V3_SPEKTRUM_POWER_EN(_on_true)

/*
 * FMUv5 has a separate RC_IN
 *
 * GPIO PPM_IN on PB5 T8CH1
 * SPEKTRUM_RX (it's TX or RX in Bind) on UART6 PG9 (NOT FMUv5 test HW ONLY)
 *   In version is possible in the UART
 * and can drive  GPIO PPM_IN as an output
 */

#define GPIO_PPM_IN_AS_OUT             (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTA|GPIO_PIN3)
#define SPEKTRUM_RX_AS_GPIO_OUTPUT()   px4_arch_configgpio(GPIO_PPM_IN_AS_OUT)
#define SPEKTRUM_RX_AS_UART()          /* Can be left as uart */
#define SPEKTRUM_OUT(_one_true)        px4_arch_gpiowrite(GPIO_PPM_IN_AS_OUT, (_one_true))

#define SDIO_SLOTNO                    1  /* Only one slot */
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

/* FMUv5 never powers odd the Servo rail */

#define BOARD_ADC_SERVO_VALID     (1)

#if !defined(BOARD_HAS_LTC44XX_VALIDS) || BOARD_HAS_LTC44XX_VALIDS == 0
#  define BOARD_ADC_BRICK_VALID  (1)
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

//#define BOARD_ADC_PERIPH_5V_OC  (!px4_arch_gpioread(GPIO_nVDD_5V_PERIPH_OC))
//#define BOARD_ADC_HIPOWER_5V_OC (!px4_arch_gpioread(GPIO_nVDD_5V_HIPOWER_OC))


/* This board provides a DMA pool and APIs */
#define BOARD_DMA_ALLOC_POOL_SIZE 5120

/* This board provides the board_on_reset interface */

#define BOARD_HAS_ON_RESET 1

#define PX4_GPIO_INIT_LIST { \
		PX4_ADC_GPIO,                     \
		GPIO_HW_VER_REV_DRIVE,            \
		GPIO_CAN1_TX,                     \
		GPIO_CAN1_RX,                     \
		GPIO_HEATER_OUTPUT,               \
		GPIO_nPOWER_IN_A,                 \
		GPIO_VDD_3V3_SD_CARD_EN,          \
		GPIO_VDD_3V3_SPEKTRUM_POWER_EN,   \
		GPIO_RSSI_IN_INIT,                \
		GPIO_nARMED_INIT,                 \
		GPIO_TONE_ALARM_IDLE              \
	}

#define BOARD_ENABLE_CONSOLE_BUFFER

#define BOARD_NUM_IO_TIMERS 8
#define FLASH_BASED_PARAMS

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
