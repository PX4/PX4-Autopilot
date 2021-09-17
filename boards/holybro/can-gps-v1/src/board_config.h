/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
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
 * PX4FMUv4 internal definitions
 */

#pragma once

/****************************************************************************************************
 * Included Files
 ****************************************************************************************************/

#include <px4_platform_common/px4_config.h>
#include <nuttx/compiler.h>
#include <stdint.h>

// todo:NCP5623  datasheet says 0x38 driver says 0x39 - needs testing

/* GPIO *************************************************************************** */
/*
 * ADC channels
 *
 * These are the channel numbers of the ADCs of the microcontroller that
 * can be used by the Px4 Firmware in the adc driver
 */


#define ADC1_CH(n)                  (n)
#define ADC1_GPIO(n)                GPIO_ADC1_IN##n

/* Define GPIO pins used as ADC N.B. Channel numbers must match below */

#define PX4_ADC_GPIO  \
	/* PA0 */  ADC1_GPIO(0),  \
	/* PA1 */  ADC1_GPIO(1)

/* Define Channel numbers must match above GPIO pin IN(n)*/

#define ADC_HW_REV_SENSE_CHANNEL            /* PA0 */ ADC1_CH(0)
#define ADC_HW_VER_SENSE_CHANNEL            /* PA1 */ ADC1_CH(1)

#define ADC_CHANNELS \
	((1 << ADC_HW_REV_SENSE_CHANNEL)           | \
	 (1 << ADC_HW_VER_SENSE_CHANNEL))

/* HW Version and Revision drive signals Default to 1 to detect */

#define BOARD_HAS_HW_VERSIONING

#define GPIO_HW_VER_REV_DRIVE /* PB1 */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTB|GPIO_PIN1)
#define GPIO_HW_REV_SENSE     /* PA0 */ ADC1_GPIO(0)
#define GPIO_HW_VER_SENSE     /* PA1 */ ADC1_GPIO(1)
#define HW_INFO_INIT         {'C','A','N','G','P','S','x', 'x',0}
#define HW_INFO_INIT_VER     6
#define HW_INFO_INIT_REV     7

#define FLASH_BASED_PARAMS

/* CAN Silence
 *
 * Silent mode control
 */
#define GPIO_MCU_CAN1_SILENT  /* PB2 */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTB|GPIO_PIN2)
#define GPIO_MCU_CAN2_SILENT  /* PA4 */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTA|GPIO_PIN4)

// SPI pinning for ICM20649 is in spi.ccp

// GPIO_MCU_SPI1_DRDY                   /* PB0  */
// GPIO_MCU_SPI1_NCS_ACC                /* PA8  */

#define GPIO_SENSOR_3V3_EN              /* PC15 */  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTC|GPIO_PIN15)

#define VDD_3V3_SENSORS_EN(on_true)       px4_arch_gpiowrite(GPIO_SENSOR_3V3_EN, (on_true))

#define GPIO_MCU_I2C1_SCL_RESET         /* PB6  */  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTB|GPIO_PIN6)
#define GPIO_MCU_I2C1_SDA_RESET         /* PB7  */  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTB|GPIO_PIN7)

#define GPIO_MCU_I2C2_SCL_RESET         /* PB10 */  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTB|GPIO_PIN10)
#define GPIO_MCU_I2C2_SDA_RESET         /* PB3  */  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTB|GPIO_PIN3)


#define GPIO_NOPT_WAIT_FOR_GETNODEINFO  /* PC14 */ (GPIO_INPUT|GPIO_PULLUP|GPIO_PORTC|GPIO_PIN14)

#define GPIO_TIM1_CH1N                  /* PA7  */  GPIO_TIM1_CH1N_1 /* NLED_RED  */
#define GPIO_TIM1_CH2N                  /* PB14 */  GPIO_TIM1_CH2N_2 /* NLED_BLUE */
#define GPIO_TIM1_CH3N                  /* PB15 */  GPIO_TIM1_CH3N_2 /* NLED_GREEN */

#define GPIO_TIM2_CH1                   /* PA15 */  GPIO_TIM2_CH1_3  /* GPS_PPS_IN */ // todo:needs Driver
#define GPIO_GPS_PPS_IN                 /* PA15 */  (GPIO_INPUT|GPIO_PULLUP|GPIO_PORTA|GPIO_PIN15)

/* High-resolution timer */
#define HRT_TIMER                       3  /* use timer 3 for the HRT */
#define HRT_TIMER_CHANNEL               4  /* use capture/compare channel 4 */

#define PX4_GPIO_INIT_LIST { \
		PX4_ADC_GPIO,                   \
		GPIO_HW_VER_REV_DRIVE,          \
		GPIO_SENSOR_3V3_EN,             \
		GPIO_MCU_I2C1_SCL_RESET,        \
		GPIO_MCU_I2C1_SDA_RESET,        \
		GPIO_MCU_I2C2_SCL_RESET,        \
		GPIO_MCU_I2C2_SDA_RESET,        \
		GPIO_CAN1_TX,                   \
		GPIO_CAN1_RX,                   \
		GPIO_CAN2_TX,                   \
		GPIO_CAN2_RX,                   \
		GPIO_MCU_CAN1_SILENT,           \
		GPIO_MCU_CAN2_SILENT,           \
		GPIO_NOPT_WAIT_FOR_GETNODEINFO, \
		GPIO_GPS_PPS_IN                 \
	}

__BEGIN_DECLS

#ifndef __ASSEMBLY__

/****************************************************************************************************
 * Public Functions
 ****************************************************************************************************/

/****************************************************************************************************
 * Name: stm32_spiinitialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the PX4FMU board.
 *
 ****************************************************************************************************/

extern void stm32_spiinitialize(void);

extern void stm32_usbinitialize(void);

#include <px4_platform_common/board_common.h>

#endif /* __ASSEMBLY__ */

__END_DECLS
