/****************************************************************************
 *
 *   Copyright (c) 2013-2016 PX4 Development Team. All rights reserved.
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
 * AeroCore2 internal definitions
 */

#pragma once

/****************************************************************************************************
 * Included Files
 ****************************************************************************************************/

#include <px4_config.h>
#include <nuttx/compiler.h>
#include <stdint.h>

/****************************************************************************************************
 * Definitions
 ****************************************************************************************************/

/* LEDs */
#define GPIO_LED0	(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTE|GPIO_PIN9)
#define GPIO_LED1	(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTE|GPIO_PIN10)

#define GPIO_LED_RED	GPIO_LED1
#define GPIO_LED_GREEN	GPIO_LED0
#define GPIO_LED_BLUE	0
#define GPIO_LED_SAFETY 0

#define BOARD_HAS_CONTROL_STATUS_LEDS	1

/* Power muxes */
/* LOW=battery HIGH=USB */
#define GPIO_POWER_MUX_SENSE	(GPIO_INPUT|GPIO_PULLUP|GPIO_PORTC|GPIO_PIN5)

/* External interrupts */
#define GPIO_EXTI_GYRO_DRDY	(GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI|GPIO_PORTD|GPIO_PIN2)
#define GPIO_EXTI_MAG_DRDY	(GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI|GPIO_PORTD|GPIO_PIN4)
#define GPIO_EXTI_ACCEL_DRDY	(GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI|GPIO_PORTD|GPIO_PIN3)

/* Data ready pins off */
#define GPIO_GYRO_DRDY_OFF	(GPIO_INPUT|GPIO_PULLDOWN|GPIO_SPEED_2MHz|GPIO_PORTD|GPIO_PIN2)
#define GPIO_MAG_DRDY_OFF	(GPIO_INPUT|GPIO_PULLDOWN|GPIO_SPEED_2MHz|GPIO_PORTD|GPIO_PIN4)
#define GPIO_ACCEL_DRDY_OFF	(GPIO_INPUT|GPIO_PULLDOWN|GPIO_SPEED_2MHz|GPIO_PORTD|GPIO_PIN3)

/* SPI3 off */
#define GPIO_SPI3_SCK_OFF	(GPIO_INPUT|GPIO_PULLDOWN|GPIO_PORTC|GPIO_PIN10)
#define GPIO_SPI3_MISO_OFF	(GPIO_INPUT|GPIO_PULLDOWN|GPIO_PORTC|GPIO_PIN11)
#define GPIO_SPI3_MOSI_OFF	(GPIO_INPUT|GPIO_PULLDOWN|GPIO_PORTC|GPIO_PIN12)

/* SPI3 chip selects off */
#define GPIO_SPI_CS_GYRO_OFF		(GPIO_INPUT|GPIO_PULLDOWN|GPIO_SPEED_2MHz|GPIO_PORTE|GPIO_PIN3)
#define GPIO_SPI_CS_ACCEL_MAG_OFF	(GPIO_INPUT|GPIO_PULLDOWN|GPIO_SPEED_2MHz|GPIO_PORTE|GPIO_PIN2)
#define GPIO_SPI_CS_BARO_OFF		(GPIO_INPUT|GPIO_PULLDOWN|GPIO_SPEED_2MHz|GPIO_PORTE|GPIO_PIN4)

/* SPI chip selects */
#define GPIO_SPI_CS_GYRO	(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTE|GPIO_PIN3)
#define GPIO_SPI_CS_ACCEL_MAG	(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTE|GPIO_PIN2)
#define GPIO_SPI_CS_BARO	(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTE|GPIO_PIN4)
#define GPIO_SPI_CS_FRAM	(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTE|GPIO_PIN11)

#define PX4_SPI_BUS_SENSORS	3
#define PX4_SPI_BUS_RAMTRON	4
#define PX4_SPI_BUS_EXT		1
#define PX4_SPI_BUS_BARO	PX4_SPI_BUS_SENSORS

/* Use these in place of the uint32_t enumeration to select a specific SPI device on SPI3 */
#define PX4_SPIDEV_GYRO       PX4_MK_SPI_SEL(PX4_SPI_BUS_SENSORS, 1)
#define PX4_SPIDEV_ACCEL_MAG  PX4_MK_SPI_SEL(PX4_SPI_BUS_SENSORS, 2)
#define PX4_SPIDEV_BARO       PX4_MK_SPI_SEL(PX4_SPI_BUS_SENSORS, 3)

/* I2C busses */
#define PX4_I2C_BUS_EXPANSION	1
#define PX4_I2C_BUS_ONBOARD	2

/*
 * ADC channels
 *
 * These are the channel numbers of the ADCs of the microcontroller that can be used by the Px4 Firmware in the adc driver
 */
#define ADC_CHANNELS (1 << 10) | (1 << 11) | (1 << 12) | (1 << 13)

/*
 * ADC defines to be used in sensors.cpp to read from a particular channel
*/
#define ADC_BATTERY_VOLTAGE_CHANNEL	10
#define ADC_BATTERY_CURRENT_CHANNEL	0

/* Define Battery 1 Voltage Divider
 */

#define BOARD_BATTERY1_V_DIV   (11.0f)

/* Tone alarm output */
#define TONE_ALARM_TIMER	10	/* timer 10 */
#define TONE_ALARM_CHANNEL	1	/* channel 1 */
#define GPIO_TONE_ALARM_IDLE	(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTB|GPIO_PIN8)
#define GPIO_TONE_ALARM		(GPIO_ALT|GPIO_AF3|GPIO_SPEED_2MHz|GPIO_PUSHPULL|GPIO_PORTB|GPIO_PIN8)

/* PWM
 *
 * Eight PWM outputs are configured.
 *
 * Pins:
 *
 * CH1 : PD12 : TIM4_CH1
 * CH2 : PD13 : TIM4_CH2
 * CH3 : PD14 : TIM4_CH3
 * CH4 : PD15 : TIM4_CH4
 * CH5 : PA0  : TIM5_CH1
 * CH6 : PA1  : TIM5_CH2
 * CH7 : PA2  : TIM5_CH3
 * CH8 : PA3  : TIM5_CH4
 */
#define GPIO_TIM4_CH1OUT	GPIO_TIM4_CH1OUT_2
#define GPIO_TIM4_CH2OUT	GPIO_TIM4_CH2OUT_2
#define GPIO_TIM4_CH3OUT	GPIO_TIM4_CH3OUT_2
#define GPIO_TIM4_CH4OUT	GPIO_TIM4_CH4OUT_2
#define GPIO_TIM5_CH1OUT	GPIO_TIM5_CH1OUT_1
#define GPIO_TIM5_CH2OUT	GPIO_TIM5_CH2OUT_1
#define GPIO_TIM5_CH3OUT	GPIO_TIM5_CH3OUT_1
#define GPIO_TIM5_CH4OUT	GPIO_TIM5_CH4OUT_1
#define DIRECT_PWM_OUTPUT_CHANNELS	8

/* PWM
 *
 * Eight PWM inputs are configured.
 *
 * Pins:
 *
 * CH8 : PE5  : TIM9_CH1
 * CH7 : PE6  : TIM9_CH2
 * CH6 : PC6  : TIM3_CH1
 * CH5 : PC7  : TIM3_CH2
 * CH4 : PC8  : TIM3_CH3
 * CH3 : PA8  : TIM1_CH1
 * CH2 : PA9  : TIM1_CH2
 * CH1 : PA10 : TIM1_CH3
 */
#define GPIO_TIM9_CH1IN		GPIO_TIM9_CH1IN_2
#define GPIO_TIM9_CH2IN		GPIO_TIM9_CH2IN_2
#define GPIO_TIM3_CH1IN		GPIO_TIM3_CH1IN_3
#define GPIO_TIM3_CH2IN		GPIO_TIM3_CH2IN_3
#define GPIO_TIM3_CH3IN		GPIO_TIM3_CH3IN_2
#define GPIO_TIM1_CH1IN		GPIO_TIM1_CH1IN_1
#define GPIO_TIM1_CH2IN		GPIO_TIM1_CH2IN_1
#define GPIO_TIM1_CH3IN		GPIO_TIM1_CH3IN_1
#define DIRECT_INPUT_TIMER_CHANNELS  8

/* spektrum satellite receiver input */
#define GPIO_SPEKTRUM_PWR_EN	(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTE|GPIO_PIN15)
#define SPEKTRUM_POWER(_on_true)	px4_arch_gpiowrite(GPIO_SPEKTRUM_PWR_EN, (_on_true))
#define RC_SERIAL_PORT		"/dev/ttyS4" /* No HW invert support */
#define GPIO_PPM_IN		0
#define GPIO_RC_OUT		0
#define SPEKTRUM_RX_AS_GPIO_OUTPUT()	px4_arch_configgpio(GPIO_RC_OUT)
#define SPEKTRUM_OUT(_one_true)	px4_arch_gpiowrite(GPIO_RC_OUT, (_one_true))
#define SPEKTRUM_RX_AS_UART()	/* Can be left as uart */

/* USB OTG FS
 *
 * PA9  OTG_FS_VBUS VBUS sensing
 */
#define GPIO_OTGFS_VBUS		(GPIO_INPUT|GPIO_FLOAT|GPIO_SPEED_100MHz|GPIO_OPENDRAIN|GPIO_PORTA|GPIO_PIN9)

/* High-resolution timer */
#define HRT_TIMER		8	/* use timer8 for the HRT */
#define HRT_TIMER_CHANNEL	1	/* use capture/compare channel */

#define BOARD_HAS_PWM	DIRECT_PWM_OUTPUT_CHANNELS

/* This board provides a DMA pool and APIs */
#define BOARD_DMA_ALLOC_POOL_SIZE 5120

/* The board isn't extremely constrained but
 * src/modules/navigator/navigation.h defines
 * NUM_MISSIONS_SUPPORTED to be UINT16_MAX - 1
 * which is too large for the AeroCore2's FRAM
 * so we define MEMORY_CONSTRAINED_SYSTEM so that
 * NUM_MISSIONS_SUPPORTED is only 50
 */
#define MEMORY_CONSTRAINED_SYSTEM

/* Dataman uses the mtd to save data
 */
#define BOARD_HAS_MTD_PARTITION_OVERRIDE	{"/fs/mtd_params", "/fs/mtd_dataman"}

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

/****************************************************************************************************
 * Name: stm32_spiinitialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the PX4FMU board.
 *
 ****************************************************************************************************/

extern void stm32_spiinitialize(void);
extern void board_spi_reset(int ms);
extern void stm32_usbinitialize(void);
extern void board_peripheral_reset(int ms);

#include <drivers/boards/common/board_common.h>

#endif /* __ASSEMBLY__ */

__END_DECLS
