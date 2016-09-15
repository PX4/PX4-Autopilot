/****************************************************************************
 *
 *   Copyright (c) 2013, 2014 PX4 Development Team. All rights reserved.
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
 * PX4-CRAZYFLIE internal definitions
 */

#pragma once

/****************************************************************************************************
 * Included Files
 ****************************************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>
#include <stdint.h>

__BEGIN_DECLS

/* these headers are not C++ safe */
#include <stm32.h>
#include <arch/board/board.h>

#define UDID_START		0x1FFF7A10

/****************************************************************************************************
 * Definitions
 ****************************************************************************************************/
/* Configuration ************************************************************************************/

/* Crazyflie GPIOs **********************************************************************************/
/* LEDs */


/* Radio TX indicator */
#define GPIO_LED_RED_L       (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|\
			      GPIO_OUTPUT_CLEAR|GPIO_PORTC|GPIO_PIN0)
/* Radio RX indicator */
#define GPIO_LED_GREEN_L       (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|\
				GPIO_OUTPUT_CLEAR|GPIO_PORTC|GPIO_PIN1)
#define GPIO_LED_GREEN_R       (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|\
				GPIO_OUTPUT_CLEAR|GPIO_PORTC|GPIO_PIN2)
#define GPIO_LED_RED_R       (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|\
			      GPIO_OUTPUT_CLEAR|GPIO_PORTC|GPIO_PIN3)

/* PX4: armed state indicator ; Stock FW: Blinking while charging */
#define GPIO_LED_BLUE_L		(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTD|GPIO_PIN2)

#define LED_TX 4
#define LED_RX 5

#define GPIO_FSYNC_MPU9250		(GPIO_OUTPUT|GPIO_PORTC|GPIO_PIN14) // Needs to be set low
#define GPIO_DRDY_MPU9250		(GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI|GPIO_PORTC|GPIO_PIN13)


#define GPIO_NRF_TXEN			(GPIO_INPUT|GPIO_PULLUP|GPIO_EXTI|GPIO_PORTA|GPIO_PIN4)


/*
 * I2C busses
 */
#define PX4_I2C_BUS_ONBOARD	3
#define PX4_I2C_BUS_EXPANSION	1

#define PX4_I2C_BUS_ONBOARD_HZ      400000
#define PX4_I2C_BUS_EXPANSION_HZ      400000

#define PX4_I2C_BUS_MTD	PX4_I2C_BUS_EXPANSION



/* Devices on the onboard bus.
 *
 * Note that these are unshifted addresses.
 */
#define PX4_I2C_OBDEV_MPU9250	0x69

/* USB OTG FS
 *
 * PA9  OTG_FS_VBUS VBUS sensing
 */
#define GPIO_OTGFS_VBUS		(GPIO_INPUT|GPIO_FLOAT|GPIO_SPEED_100MHz|GPIO_OPENDRAIN|GPIO_PORTA|GPIO_PIN9)

/*
 * ADC channels
 *
 * These are the channel numbers of the ADCs of the microcontroller that can be used by the Px4 Firmware in the adc driver
 */
#define ADC_CHANNELS 0

// ADC defines to be used in sensors.cpp to read from a particular channel
// Crazyflie 2 performs battery sensing via the NRF module
#define ADC_BATTERY_VOLTAGE_CHANNEL	((uint8_t)(-1))
#define ADC_BATTERY_CURRENT_CHANNEL	((uint8_t)(-1))
#define ADC_AIRSPEED_VOLTAGE_CHANNEL	((uint8_t)(-1))

/* PWM
*
* Four PWM motor outputs are configured.
*
* Pins:
*
* CH1 : PA1  : TIM2_CH2
* CH2 : PB11 : TIM2_CH4
* CH3 : PA15 : TIM2_CH1
* CH4 : PB9  : TIM4_CH4
*/

#define GPIO_TIM2_CH2OUT	GPIO_TIM2_CH2OUT_1
#define GPIO_TIM2_CH4OUT	GPIO_TIM2_CH4OUT_2
#define GPIO_TIM2_CH1OUT	GPIO_TIM2_CH1OUT_2
#define GPIO_TIM4_CH4OUT	GPIO_TIM4_CH4OUT_1
#define DIRECT_PWM_OUTPUT_CHANNELS	4

#define GPIO_TIM2_CH2IN		GPIO_TIM2_CH2IN_1
#define GPIO_TIM2_CH4IN		GPIO_TIM2_CH4IN_2
#define GPIO_TIM2_CH1IN		GPIO_TIM2_CH1IN_2
#define GPIO_TIM4_CH4IN		GPIO_TIM4_CH4IN_1



/* High-resolution timer */
#define HRT_TIMER		8	/* use timer8 for the HRT */
#define HRT_TIMER_CHANNEL	1	/* use capture/compare channel */



#define BOARD_HAS_PWM	DIRECT_PWM_OUTPUT_CHANNELS

#define BOARD_FMU_GPIO_TAB { {0, 0, 0}, }




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
void board_spi_reset(int ms);

extern void stm32_usbinitialize(void);

extern void board_peripheral_reset(int ms);

/****************************************************************************
 * Name: nsh_archinitialize
 *
 * Description:
 *   Perform architecture specific initialization for NSH.
 *
 *   CONFIG_NSH_ARCHINIT=y :
 *     Called from the NSH library
 *
 *   CONFIG_BOARD_INITIALIZE=y, CONFIG_NSH_LIBRARY=y, &&
 *   CONFIG_NSH_ARCHINIT=n :
 *     Called from board_initialize().
 *
 ****************************************************************************/

#ifdef CONFIG_NSH_LIBRARY
int nsh_archinitialize(void);
#endif




/****************************************************************************
 * Name: board_i2c_initialize
 *
 * Description:
 *   Called to set I2C bus frequncies.
 *
 ****************************************************************************/

int board_i2c_initialize(void);


#endif /* __ASSEMBLY__ */

__END_DECLS
