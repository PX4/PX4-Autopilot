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

/* LEDs are driven with push open drain to support Anode to 5V or 3.3V */
#define GPIO_nLED_RED        /* PB1 */  (GPIO_OUTPUT|GPIO_OPENDRAIN|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTB|GPIO_PIN1)
#define GPIO_nLED_GREEN      /* PC6 */  (GPIO_OUTPUT|GPIO_OPENDRAIN|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTC|GPIO_PIN6)
#define GPIO_nLED_BLUE       /* PC7 */  (GPIO_OUTPUT|GPIO_OPENDRAIN|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTC|GPIO_PIN7)

#define BOARD_HAS_CONTROL_STATUS_LEDS      1
#define BOARD_OVERLOAD_LED     LED_RED
#define BOARD_ARMED_STATE_LED  LED_BLUE

#define FLASH_BASED_PARAMS

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
	/* PC0 */  ADC1_GPIO(10), \
	/* PC1 */  ADC1_GPIO(11), \
	/* PC2 */  ADC1_GPIO(12), \
	/* PC3 */  ADC1_GPIO(13), \
	/* PC4 */  ADC1_GPIO(14)

/* Define Channel numbers must match above GPIO pin IN(n)*/
#define ADC_BATTERY_VOLTAGE_CHANNEL         /* PA0 */  ADC1_CH(0)
#define ADC_BATTERY_CURRENT_CHANNEL 0
#define ADC_SCALED_V5_CHANNEL               /* PC0 */  ADC1_CH(10)
#define ADC_SCALED_VDD_3V3_SENSORS_CHANNEL  /* PC1 */  ADC1_CH(11)
#define ADC_HW_VER_SENSE_CHANNEL            /* PC2 */  ADC1_CH(12)
#define ADC_HW_REV_SENSE_CHANNEL            /* PC3 */  ADC1_CH(13)
#define ADC1_SPARE_1_CHANNEL                /* PC4 */  ADC1_CH(14) // POWER_AD

#define ADC_CHANNELS \
	((1 << ADC_BATTERY_VOLTAGE_CHANNEL)        | \
	 (1 << ADC_SCALED_V5_CHANNEL)              | \
	 (1 << ADC_SCALED_VDD_3V3_SENSORS_CHANNEL) | \
	 (1 << ADC_HW_VER_SENSE_CHANNEL)           | \
	 (1 << ADC_HW_REV_SENSE_CHANNEL)           | \
	 (1 << ADC1_SPARE_1_CHANNEL))

/* Define Battery 1 Voltage Divider and A per V */
#define BOARD_BATTERY1_V_DIV         (9.0f)     /* measured with the provided PM board */

/* HW has to large of R termination on ADC todo:change when HW value is chosen */
#define BOARD_ADC_OPEN_CIRCUIT_V     (5.6f)

/* HW Version and Revision drive signals Default to 1 to detect */
#define BOARD_HAS_HW_VERSIONING

#define GPIO_HW_REV_DRIVE    /* PH14  */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTH|GPIO_PIN14)
#define GPIO_HW_REV_SENSE    /* PC3   */ ADC1_GPIO(13)
#define GPIO_HW_VER_DRIVE    /* PG0   */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTG|GPIO_PIN0)
#define GPIO_HW_VER_SENSE    /* PC2   */ ADC1_GPIO(12)
#define HW_INFO_INIT         {'V','5','x', 'x',0}
#define HW_INFO_INIT_VER     2
#define HW_INFO_INIT_REV     3

#define BOARD_TAP_ESC_MODE 2 // select closed-loop control mode for the esc
// #define BOARD_USE_ESC_CURRENT_REPORT

// LED mapping
#define BOARD_FRONT_LED_MASK (1 << 0) | (1 << 3)
#define BOARD_REAR_LED_MASK  (1 << 1) | (1 << 2)

/* HEATER */
#define GPIO_HEATER_OUTPUT   /* PA7  T14CH1 */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTA|GPIO_PIN7)

#define BOARD_HAS_LED_PWM              1
#define BOARD_LED_PWM_DRIVE_ACTIVE_LOW 1

#define BOARD_UI_LED_PWM_DRIVE_ACTIVE_LOW 1

#define BOARD_ADC_BRICK_VALID           1
#define BOARD_NUMBER_BRICKS             1

#define GPIO_VDD_3V3_SD_CARD_EN         /* PG7  */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTG|GPIO_PIN7)
#define VDD_3V3_SD_CARD_EN(on_true)        px4_arch_gpiowrite(GPIO_VDD_3V3_SD_CARD_EN, (on_true))

/* USB OTG FS */
#define GPIO_OTGFS_VBUS         /* PA9 */ (GPIO_INPUT|GPIO_PULLDOWN|GPIO_SPEED_100MHz|GPIO_PORTA|GPIO_PIN9)

/* High-resolution timer */
#define HRT_TIMER               8  /* use timer8 for the HRT */
#define HRT_TIMER_CHANNEL       3  /* use capture/compare channel 3 */

/* RC Serial port */
#define RC_SERIAL_PORT                     "/dev/ttyS5"
#define RC_SERIAL_SINGLEWIRE

#define BOARD_HAS_POWER_CONTROL	1
/* power on/off */
#define MS_PWR_BUTTON_DOWN 750
#define KEY_AD_GPIO    (GPIO_INPUT|GPIO_PULLUP|GPIO_EXTI|GPIO_PORTC|GPIO_PIN4)
#define POWER_ON_GPIO  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTC|GPIO_PIN5)
#define POWER_OFF_GPIO (GPIO_INPUT|GPIO_PULLDOWN|GPIO_PORTC|GPIO_PIN5)
#define POWER_CHECK_GPIO (GPIO_INPUT|GPIO_PULLUP|GPIO_PORTF|GPIO_PIN0)

#define SDIO_SLOTNO                    0  /* Only one slot */
#define SDIO_MINOR                     0

/* By Providing BOARD_ADC_USB_CONNECTED (using the px4_arch abstraction)
 * this board support the ADC system_power interface, and therefore
 * provides the true logic GPIO BOARD_ADC_xxxx macros.
 */
#define BOARD_ADC_USB_CONNECTED (px4_arch_gpioread(GPIO_OTGFS_VBUS))
#define BOARD_ADC_USB_VALID     BOARD_ADC_USB_CONNECTED

/* FMUv5 never powers odd the Servo rail */
#define BOARD_ADC_SERVO_VALID     (1)

#define BOARD_ADC_BRICK1_VALID  (!px4_arch_gpioread(GPIO_nVDD_BRICK1_VALID))
#define BOARD_ADC_BRICK2_VALID  (0)

/* This board provides a DMA pool and APIs */
#define BOARD_DMA_ALLOC_POOL_SIZE 5120

/* This board provides the board_on_reset interface */
#define BOARD_HAS_ON_RESET 1
#define BOARD_HAS_POWER_CONTROL 1

#define GPIO_CAM_PWR_ON_H      /* PB0 */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTB|GPIO_PIN0)
#define GPIO_CAM_PWR_ON_L      /* PB0 */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTB|GPIO_PIN0)

#define PX4_GPIO_INIT_LIST { \
		PX4_ADC_GPIO,                     \
		GPIO_HW_REV_DRIVE,                \
		GPIO_HW_VER_DRIVE,                \
		GPIO_HEATER_OUTPUT,               \
		GPIO_VDD_3V3_SD_CARD_EN,          \
		GPIO_OTGFS_VBUS                   \
	}

#define BOARD_ENABLE_CONSOLE_BUFFER

#define BOARD_NUM_IO_TIMERS 1

__BEGIN_DECLS

#ifndef __ASSEMBLY__

int stm32_sdio_initialize(void);

extern void stm32_spiinitialize(void);

extern void stm32_usbinitialize(void);

extern void board_peripheral_reset(int ms);

/************************************************************************************
 * Name: board_pwr_init()
 *
 * Description:
 *   Called to configure power control for the tap-v2 board.
 *
 * Input Parameters:
 *   stage- 0 for boot, 1 for board init
 *
 ************************************************************************************/

void board_pwr_init(int stage);

#include <px4_platform_common/board_common.h>

#endif /* __ASSEMBLY__ */

__END_DECLS
