/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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
 * Board internal definitions
 */

#pragma once

#include <px4_platform_common/px4_config.h>
#include <nuttx/compiler.h>
#include <stdint.h>
#include <stm32_gpio.h>

/* LEDs */
#define GPIO_nLED_RED        /* PE3 */  (GPIO_OUTPUT|GPIO_OPENDRAIN|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTE|GPIO_PIN3)
#define GPIO_nLED_GREEN      /* PE4 */  (GPIO_OUTPUT|GPIO_OPENDRAIN|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTE|GPIO_PIN4)
#define GPIO_nLED_BLUE       /* PE5 */  (GPIO_OUTPUT|GPIO_OPENDRAIN|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTE|GPIO_PIN5)

#define BOARD_HAS_CONTROL_STATUS_LEDS      1
#define BOARD_OVERLOAD_LED     LED_RED
#define BOARD_ARMED_STATE_LED  LED_BLUE

/* ADC channels */
#define GPIO_ADC1_INP16           (GPIO_ANALOG|GPIO_PORTA|GPIO_PIN0)
#define GPIO_ADC1_INP6            (GPIO_ANALOG|GPIO_PORTF|GPIO_PIN12)
#define GPIO_ADC1_INP9            (GPIO_ANALOG|GPIO_PORTB|GPIO_PIN0)
#define GPIO_ADC1_INP18           (GPIO_ANALOG|GPIO_PORTA|GPIO_PIN4)
#define GPIO_ADC1_INP5            (GPIO_ANALOG|GPIO_PORTB|GPIO_PIN1)
#define GPIO_ADC1_INP11           (GPIO_ANALOG|GPIO_PORTC|GPIO_PIN11)
#define GPIO_ADC1_INP10           (GPIO_ANALOG|GPIO_PORTC|GPIO_PIN10)
#define GPIO_ADC1_INP12           (GPIO_ANALOG|GPIO_PORTC|GPIO_PIN12)
#define GPIO_ADC1_INP13           (GPIO_ANALOG|GPIO_PORTC|GPIO_PIN13)
#define GPIO_ADC1_INP4            (GPIO_ANALOG|GPIO_PORTC|GPIO_PIN4)
#define GPIO_ADC1_INP8            (GPIO_ANALOG|GPIO_PORTC|GPIO_PIN5)
#define GPIO_ADC1_INP15           (GPIO_ANALOG|GPIO_PORTA|GPIO_PIN3)

#define PX4_ADC_GPIO  \
	/* PA0  */  GPIO_ADC1_INP16,  \
	/* PF12 */  GPIO_ADC1_INP6,   \
	/* PB0  */  GPIO_ADC1_INP9,   \
	/* PA4  */  GPIO_ADC1_INP18,  \
	/* PB1  */  GPIO_ADC1_INP5,   \
	/* PC1  */  GPIO_ADC1_INP11,  \
	/* PC0  */  GPIO_ADC1_INP10,  \
	/* PC2  */  GPIO_ADC1_INP12,  \
	/* PC3  */  GPIO_ADC1_INP13,  \
	/* PC4  */  GPIO_ADC1_INP4,   \
	/* PC5  */  GPIO_ADC1_INP8,   \
	/* PA3  */  GPIO_ADC1_INP15

/* Define Channel numbers must match above GPIO pin IN(n)*/
#define BATT_VOLTAGE_SENS      /* PA0  */  16
#define BATT_CURRENT_SENS      /* PF12 */  6
#define BATT2_VOLTAGE_SENS     /* PB0  */  9
#define BATT2_CURRENT_SENS     /* PA4  */  18
#define VDD_5V_SENS            /* PB1  */  5
#define SCALED_V3V3            /* PC1  */  11
#define RSSI_IN                /* PC0  */  10
#define ADC1_6V6               /* PC2  */  12
#define ADC1_3V3               /* PC3  */  13
#define HW_VER_SENSE           /* PC4  */  4
#define HW_REV_SENSE           /* PC5  */  8
#define FMU_SERVORAIL_VCC      /* PA3  */  15

/* Define Channel numbers must match above GPIO pins */
#define ADC_BATTERY1_VOLTAGE_CHANNEL        BATT_VOLTAGE_SENS  /* PA0:  BATT_VOLTAGE_SENS */
#define ADC_BATTERY1_CURRENT_CHANNEL        BATT_CURRENT_SENS  /* PF12: BATT_CURRENT_SENS */
#define ADC_BATTERY2_VOLTAGE_CHANNEL        BATT2_VOLTAGE_SENS /* PB0:  FMU_AUX_POWER_ADC1 */
#define ADC_BATTERY2_CURRENT_CHANNEL        BATT2_CURRENT_SENS /* PA4:  FMU_AUX_ADC2 */
#define ADC_SCALED_V5_CHANNEL               VDD_5V_SENS        /* PB1:  VDD_5V_SENS */
#define ADC_HW_VER_SENSE_CHANNEL            HW_VER_SENSE       /* PC4:  HW_VER_SENSE */
#define ADC_HW_REV_SENSE_CHANNEL            HW_REV_SENSE       /* PC5:  HW_REV_SENSE */

#define ADC_CHANNELS \
	((1 << BATT_VOLTAGE_SENS) | \
	 (1 << BATT_CURRENT_SENS) | \
	 (1 << BATT2_VOLTAGE_SENS) | \
	 (1 << BATT2_CURRENT_SENS) | \
	 (1 << VDD_5V_SENS) | \
	 (1 << SCALED_V3V3) | \
	 (1 << RSSI_IN) | \
	 (1 << ADC1_6V6) | \
	 (1 << ADC1_3V3) | \
	 (1 << HW_VER_SENSE) | \
	 (1 << HW_REV_SENSE) | \
	 (1 << FMU_SERVORAIL_VCC))

/* HW has to large of R termination on ADC todo:change when HW value is chosen */
#define BOARD_ADC_OPEN_CIRCUIT_V     (5.6f)

/* UAVCAN */
#define UAVCAN_NUM_IFACES_RUNTIME  1

/* PWM */
#define DIRECT_PWM_OUTPUT_CHANNELS  8
#define BOARD_NUM_IO_TIMERS 6

/* Power supply control and monitoring GPIOs */
#define BOARD_NUMBER_BRICKS             2
#define GPIO_nVDD_BRICK1_VALID          /* PG1  */ (GPIO_INPUT|GPIO_PULLUP|GPIO_PORTG|GPIO_PIN1) // VDD_BRICK_VALID
#define GPIO_nVDD_BRICK2_VALID          /* PG2  */ (GPIO_INPUT|GPIO_PULLUP|GPIO_PORTG|GPIO_PIN2) // VDD_BACKUP_VALID
#define GPIO_nVDD_USB_VALID             /* PG3  */ (GPIO_INPUT|GPIO_PULLUP|GPIO_PORTG|GPIO_PIN3) // VBUS_VALID
#define GPIO_VDD_3V3_SENSORS_EN         /* PG12 */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTG|GPIO_PIN12) // VDD_3V3_SENSORS_EN
#define GPIO_VDD_3V3_SENSORS2_EN        /* PD4  */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTD|GPIO_PIN4) // VDD_3V3_SENSORS2_EN
#define GPIO_VDD_3V3_SENSORS3_EN        /* PD3  */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTD|GPIO_PIN3) // VDD_3V3_SENSORS3_EN
#define GPIO_nVDD_5V_PERIPH_EN          /* PG4  */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTG|GPIO_PIN4) // VDD_5V_PERIPH_EN
#define GPIO_nVDD_5V_PERIPH_OC          /* PE15 */ (GPIO_INPUT|GPIO_PULLUP|GPIO_PORTE|GPIO_PIN15) // VDD_5V_PERIPH_OC
#define GPIO_nVDD_5V_HIPOWER_EN         /* PG10 */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTG|GPIO_PIN10)
#define GPIO_nVDD_5V_HIPOWER_OC         /* PF13 */ (GPIO_INPUT|GPIO_PULLUP|GPIO_PORTF|GPIO_PIN13) // VDD_5V_HIPOWER_OC
#define GPIO_VDD_3V3_SPEKTRUM_POWER_EN  /* PH2  */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTH|GPIO_PIN2)
#define GPIO_SD_CARD_EN                 /* PC13 */ (GPIO_INPUT|GPIO_PORTC|GPIO_PIN13)
#define GPIO_EXT_WDOG                   /* PG5  */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_OUTPUT_CLEAR|GPIO_PORTG|GPIO_PIN5)
#define GPIO_SCHA63T_RESET              /* PE7  */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_OUTPUT_SET|GPIO_PORTE|GPIO_PIN7)
#define GPIO_HW_VER_REV_DRIVE           /* PG0  */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_OUTPUT_CLEAR|GPIO_PORTG|GPIO_PIN0)

#define GPIO_CAN1_SILENT                /* PG13 */ (GPIO_OUTPUT|GPIO_OUTPUT_CLEAR|GPIO_PORTG|GPIO_PIN13)
#define GPIO_CAN2_SILENT                /* PG8  */ (GPIO_OUTPUT|GPIO_OUTPUT_CLEAR|GPIO_PORTG|GPIO_PIN8)

#define GPIO_BUFFER_OE_EN               /* PD11 */ (GPIO_OUTPUT|GPIO_OUTPUT_CLEAR|GPIO_PORTD|GPIO_PIN11)
#define GPIO_BUFFER_OE2_EN              /* PD5  */ (GPIO_OUTPUT|GPIO_OUTPUT_CLEAR|GPIO_PORTD|GPIO_PIN5)

#define GPIO_FMU_CAP1                   /* PE11 */ (GPIO_INPUT|GPIO_PULLUP|GPIO_PORTE|GPIO_PIN11)
#define GPIO_FMU_CAP2                   /* PB11 */ (GPIO_INPUT|GPIO_PULLUP|GPIO_PORTB|GPIO_PIN11)

#define GPIO_nARMED                     /* PB10 */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_OUTPUT_CLEAR|GPIO_PORTB|GPIO_PIN10)

#define GPIO_LED_SAFETY                 /* PD10 */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_OUTPUT_CLEAR|GPIO_PORTD|GPIO_PIN10)
#define GPIO_SAFETY_SWITCH_IN           /* PF5  */ (GPIO_INPUT|GPIO_PULLDOWN|GPIO_PORTF|GPIO_PIN5)
/* Enable the FMU to use the switch it if there is no px4io fixme:This should be BOARD_SAFTY_BUTTON() */
#define GPIO_BTN_SAFETY GPIO_SAFETY_SWITCH_IN /* Enable the FMU to control it if there is no px4io */

/* Tone alarm output */
#define TONE_ALARM_TIMER        14  /* timer 14 */
#define TONE_ALARM_CHANNEL      1  /* PF9 TIM14_CH1 */
#define GPIO_BUZZER_1  /* PF9 */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTF|GPIO_PIN9) // ALARM
#define GPIO_TONE_ALARM_IDLE    GPIO_BUZZER_1
#define GPIO_TONE_ALARM         GPIO_TIM14_CH1OUT_2

/* PWM input driver. timer3 channel 1 */
#define PWMIN_TIMER                       3
#define PWMIN_TIMER_CHANNEL    /* T3C1 */ 1
#define GPIO_PWM_IN            /* PC6  */ GPIO_TIM3_CH1IN_3

/* Define True logic Power Control in arch agnostic form */
#define VDD_5V_PERIPH_EN(on_true)          px4_arch_gpiowrite(GPIO_nVDD_5V_PERIPH_EN, !(on_true))
#define VDD_5V_HIPOWER_EN(on_true)         px4_arch_gpiowrite(GPIO_nVDD_5V_HIPOWER_EN, !(on_true))
#define VDD_3V3_SENSORS_EN(on_true)        px4_arch_gpiowrite(GPIO_VDD_3V3_SENSORS_EN, (on_true))
#define VDD_3V3_SENSORS2_EN(on_true)       px4_arch_gpiowrite(GPIO_VDD_3V3_SENSORS2_EN, (on_true))
#define VDD_3V3_SENSORS3_EN(on_true)       px4_arch_gpiowrite(GPIO_VDD_3V3_SENSORS3_EN, (on_true))
#define VDD_3V3_SPEKTRUM_POWER_EN(on_true) px4_arch_gpiowrite(GPIO_VDD_3V3_SPEKTRUM_POWER_EN, (on_true))
#define BUFFER_OE_EN(on_true)              px4_arch_gpiowrite(GPIO_BUFFER_OE_EN, (on_true))
#define BUFFER_OE2_EN(on_true)             px4_arch_gpiowrite(GPIO_BUFFER_OE2_EN, (on_true))
#define ARMED_EN(on_true)                  px4_arch_gpiowrite(GPIO_nARMED, (on_true))

/* USB
 *  OTG FS: PB3  OTG_FS_VBUS VBUS sensing
 */
#define GPIO_OTGFS_VBUS         /* PB3 */ (GPIO_INPUT|GPIO_PULLDOWN|GPIO_SPEED_100MHz|GPIO_PORTB|GPIO_PIN3)

/* High-resolution timer */
#define HRT_TIMER               8  /* use timer8 for the HRT */
#define HRT_TIMER_CHANNEL       3  /* use capture/compare channel 3 */

#define SDIO_SLOTNO             0  /* Only one slot */
#define SDIO_MINOR              0

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

#define GPIO_SDMMC2_CK GPIO_SDMMC2_CK_1
#define GPIO_SDMMC2_CMD GPIO_SDMMC2_CMD_1
#define GPIO_SDMMC2_D2 GPIO_SDMMC2_D2_1

/* By Providing BOARD_ADC_USB_CONNECTED (using the px4_arch abstraction)
 * this board support the ADC system_power interface, and therefore
 * provides the true logic GPIO BOARD_ADC_xxxx macros.
 */
#define BOARD_ADC_USB_CONNECTED (px4_arch_gpioread(GPIO_OTGFS_VBUS))
#define BOARD_ADC_USB_VALID     (!px4_arch_gpioread(GPIO_nVDD_USB_VALID))
#define BOARD_ADC_BRICK1_VALID  (!px4_arch_gpioread(GPIO_nVDD_BRICK1_VALID))
#define BOARD_ADC_BRICK2_VALID  (!px4_arch_gpioread(GPIO_nVDD_BRICK2_VALID))
#define BOARD_ADC_PERIPH_5V_OC  (!px4_arch_gpioread(GPIO_nVDD_5V_PERIPH_OC))
#define BOARD_ADC_HIPOWER_5V_OC (!px4_arch_gpioread(GPIO_nVDD_5V_HIPOWER_OC))

/* This board provides a DMA pool and APIs */
#define BOARD_DMA_ALLOC_POOL_SIZE 5120

/* This board provides the board_on_reset interface */
#define BOARD_HAS_ON_RESET 1

//Add a new setting instead of removing BOARD_HAS_HW_VERSIONING
#define BOARD_HAS_STATIC_MANIFEST 2

#define BOARD_ENABLE_CONSOLE_BUFFER

#define PX4_GPIO_INIT_LIST { \
		PX4_ADC_GPIO,                     \
		GPIO_CAN1_TX,                     \
		GPIO_CAN1_RX,                     \
		GPIO_CAN2_TX,                     \
		GPIO_CAN2_RX,                     \
		GPIO_nVDD_BRICK1_VALID,           \
		GPIO_nVDD_BRICK2_VALID,           \
		GPIO_nVDD_USB_VALID,              \
		GPIO_VDD_3V3_SENSORS_EN,          \
		GPIO_VDD_3V3_SENSORS2_EN,         \
		GPIO_VDD_3V3_SENSORS3_EN,         \
		GPIO_VDD_3V3_SPEKTRUM_POWER_EN,   \
		GPIO_nVDD_5V_PERIPH_EN,           \
		GPIO_nVDD_5V_PERIPH_OC,           \
		GPIO_nVDD_5V_HIPOWER_EN,          \
		GPIO_nVDD_5V_HIPOWER_OC,          \
		PX4_MAKE_GPIO_OUTPUT_CLEAR(GPIO_I2C1_SCL), \
		PX4_MAKE_GPIO_OUTPUT_CLEAR(GPIO_I2C1_SDA), \
		PX4_MAKE_GPIO_OUTPUT_CLEAR(GPIO_I2C2_SCL), \
		PX4_MAKE_GPIO_OUTPUT_CLEAR(GPIO_I2C2_SDA), \
		PX4_MAKE_GPIO_OUTPUT_CLEAR(GPIO_I2C3_SCL), \
		PX4_MAKE_GPIO_OUTPUT_CLEAR(GPIO_I2C3_SDA), \
		PX4_MAKE_GPIO_OUTPUT_CLEAR(GPIO_I2C4_SCL), \
		PX4_MAKE_GPIO_OUTPUT_CLEAR(GPIO_I2C4_SDA), \
		GPIO_TONE_ALARM_IDLE,             \
		GPIO_OTGFS_VBUS,                  \
		GPIO_EXT_WDOG,                    \
		GPIO_SCHA63T_RESET,               \
		GPIO_HW_VER_REV_DRIVE,            \
		GPIO_CAN1_SILENT,                 \
		GPIO_CAN2_SILENT,                 \
		GPIO_BUFFER_OE_EN,                \
		GPIO_BUFFER_OE2_EN,               \
		GPIO_FMU_CAP1,                    \
		GPIO_FMU_CAP2,                    \
		GPIO_nARMED,                      \
		GPIO_LED_SAFETY,                  \
		GPIO_SAFETY_SWITCH_IN,            \
	}

__BEGIN_DECLS
#ifndef __ASSEMBLY__

int stm32_sdio_initialize(void);
extern void stm32_spiinitialize(void);
extern void stm32_usbinitialize(void);
extern void board_peripheral_reset(int ms);

#include <px4_platform_common/board_common.h>
#endif /* __ASSEMBLY__ */
__END_DECLS
