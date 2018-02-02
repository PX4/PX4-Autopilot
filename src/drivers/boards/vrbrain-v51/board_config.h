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
 * VRBRAINv52 internal definitions
 */

#pragma once

/****************************************************************************************************
 * Included Files
 ****************************************************************************************************/

#include <px4_config.h>
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










/* VRBRAIN GPIOs ***********************************************************************************/
/* LEDs */

#define GPIO_LED1		(GPIO_OUTPUT|GPIO_OPENDRAIN|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTC|GPIO_PIN5)
#define GPIO_LED2		(GPIO_OUTPUT|GPIO_OPENDRAIN|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTD|GPIO_PIN14)
#define GPIO_LED3		(GPIO_OUTPUT|GPIO_OPENDRAIN|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTD|GPIO_PIN15)

#define GPIO_LED_RED 	GPIO_LED3
#define GPIO_LED_GREEN 	GPIO_LED1
#define GPIO_LED_BLUE   GPIO_LED2

/*  Define the Chip Selects */

#define GPIO_SPI_CS_RAMTRON	    	(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTE|GPIO_PIN15)
#define GPIO_SPI_CS_MS5611  		(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTE|GPIO_PIN0)
#define GPIO_SPI_CS_IMU_MS5611 		(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTB|GPIO_PIN11)
#define GPIO_SPI_CS_EXP_MS5611 		(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTB|GPIO_PIN11)
#define GPIO_SPI_CS_MPU6000  		(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTE|GPIO_PIN10)
#define GPIO_SPI_CS_IMU_MPU6000		(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTC|GPIO_PIN2)
#define GPIO_SPI_CS_EXP_MPU6000		(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTA|GPIO_PIN8)
#define GPIO_SPI_CS_SDCARD	    	(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTA|GPIO_PIN4)
#define GPIO_SPI_CS_IMU_HMC5983		(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTB|GPIO_PIN10)
#define GPIO_SPI_CS_EXP_HMC5983		(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTB|GPIO_PIN10)

/*  Define the Ready interrupts */

#define GPIO_DRDY_MPU6000		(GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI|GPIO_PORTD|GPIO_PIN10)

/*
 *  Define the ability to shut off off the sensor signals
 *  by changing the signals to inputs
 */

#define _PIN_OFF(def) (((def) & (GPIO_PORT_MASK | GPIO_PIN_MASK)) | (GPIO_INPUT|GPIO_PULLDOWN|GPIO_SPEED_2MHz))

#define GPIO_SPI_CS_OFF_MS5611		_PIN_OFF(GPIO_SPI_CS_MS5611)
#define GPIO_SPI_CS_OFF_IMU_MS5611 	_PIN_OFF(GPIO_SPI_CS_IMU_MS5611)
#define GPIO_SPI_CS_OFF_EXP_MS5611	_PIN_OFF(GPIO_SPI_CS_EXP_MS5611)
#define GPIO_SPI_CS_OFF_MPU6000		_PIN_OFF(GPIO_SPI_CS_MPU6000)
#define GPIO_SPI_CS_OFF_IMU_MPU6000	_PIN_OFF(GPIO_SPI_CS_IMU_MPU6000)
#define GPIO_SPI_CS_OFF_EXP_MPU6000	_PIN_OFF(GPIO_SPI_CS_EXP_MPU6000)
#define GPIO_SPI_CS_OFF_IMU_HMC5983	_PIN_OFF(GPIO_SPI_CS_IMU_HMC5983)
#define GPIO_SPI_CS_OFF_EXP_HMC5983	_PIN_OFF(GPIO_SPI_CS_EXP_HMC5983)

#define GPIO_DRDY_OFF_MPU6000		_PIN_OFF(GPIO_DRDY_MPU6000)

/* SPI1 off */
#define GPIO_SPI1_SCK_OFF	_PIN_OFF(GPIO_SPI1_SCK)
#define GPIO_SPI1_MISO_OFF	_PIN_OFF(GPIO_SPI1_MISO)
#define GPIO_SPI1_MOSI_OFF	_PIN_OFF(GPIO_SPI1_MOSI)

/* SPI2 off */
#define GPIO_SPI2_SCK_OFF	_PIN_OFF(GPIO_SPI2_SCK)
#define GPIO_SPI2_MISO_OFF	_PIN_OFF(GPIO_SPI2_MISO)
#define GPIO_SPI2_MOSI_OFF	_PIN_OFF(GPIO_SPI2_MOSI)

#define PX4_SPI_BUS_BARO		1
#define PX4_SPI_BUS_SENSORS		2
#define PX4_SPI_BUS_RAMTRON		3
//#define PX4_SPI_BUS_EXT			PX4_SPI_BUS_SENSORS

/*
 * Use these in place of the spi_dev_e enumeration to select a specific SPI device on SPI1
 */
#define SPIDEV_MS5611		1
#define SPIDEV_EXP_MS5611	2
#define SPIDEV_EXP_MPU6000	3
#define SPIDEV_EXP_HMC5983	4
#define PX4_SPIDEV_BARO		SPIDEV_MS5611





/*
 * Use these in place of the spi_dev_e enumeration to select a specific SPI device on SPI2
 */
#define SPIDEV_MPU6000  	1
#define SPIDEV_IMU_MS5611	2
#define SPIDEV_IMU_MPU6000	3
#define SPIDEV_IMU_HMC5983	4
#define PX4_SPIDEV_EXT_MPU  SPIDEV_IMU_MPU6000
#define PX4_SPIDEV_MPU		SPIDEV_MPU6000
#define PX4_SPIDEV_HMC		5
#define PX4_SPIDEV_ICM		6
#define PX4_SPIDEV_LIS		7
#define PX4_SPIDEV_BMI		8
#define PX4_SPIDEV_BMA		9

/*
 * Use these in place of the spi_dev_e enumeration to select a specific SPI device on SPI3
 */

/*
 * I2C busses
 */
#define PX4_I2C_BUS_EXPANSION   1
#define PX4_I2C_BUS_ONBOARD     2

//#define I2C_BUS_HMC5883			PX4_I2C_BUS_ONBOARD
//#define I2C_BUS_EXT_HMC5883		PX4_I2C_BUS_EXPANSION

#define PX4_I2C_BUS_LED         PX4_I2C_BUS_ONBOARD

/*
 * Devices on the onboard bus.
 *
 * Note that these are unshifted addresses.
 */
#define PX4_I2C_OBDEV_LED        0x55
#define PX4_I2C_OBDEV_HMC5883    0x1E
#define PX4_I2C_OBDEV_LIS3MDL	 0x1E

/*
 * ADC channels
 *
 * These are the channel numbers of the ADCs of the microcontroller that can be used by the Px4 Firmware in the adc driver
 */
//#define ADC_CHANNELS (1 << 10) | (1 << 11) | (1 << 14)
#define ADC_CHANNELS (1 << 10) | (1 << 11)

// ADC defines to be used in sensors.cpp to read from a particular channel
#define ADC_BATTERY_VOLTAGE_CHANNEL		10
#define ADC_BATTERY_CURRENT_CHANNEL		11


/* User GPIOs
 *
 * GPIO0-5 are the PWM servo outputs.
 */
#define GPIO_GPIO0_INPUT	(GPIO_INPUT|GPIO_PULLUP|GPIO_PORTA|GPIO_PIN1)
#define GPIO_GPIO1_INPUT	(GPIO_INPUT|GPIO_PULLUP|GPIO_PORTA|GPIO_PIN2)
#define GPIO_GPIO2_INPUT	(GPIO_INPUT|GPIO_PULLUP|GPIO_PORTA|GPIO_PIN3)
#define GPIO_GPIO3_INPUT	(GPIO_INPUT|GPIO_PULLUP|GPIO_PORTB|GPIO_PIN5)
#define GPIO_GPIO4_INPUT	(GPIO_INPUT|GPIO_PULLUP|GPIO_PORTB|GPIO_PIN0)
#define GPIO_GPIO5_INPUT	(GPIO_INPUT|GPIO_PULLUP|GPIO_PORTB|GPIO_PIN1)
#define GPIO_GPIO6_INPUT	(GPIO_INPUT|GPIO_PULLUP|GPIO_PORTD|GPIO_PIN13)
#define GPIO_GPIO7_INPUT	(GPIO_INPUT|GPIO_PULLUP|GPIO_PORTD|GPIO_PIN12)
#define GPIO_GPIO8_INPUT	(GPIO_INPUT|GPIO_PULLUP|GPIO_PORTE|GPIO_PIN9)
#define GPIO_GPIO9_INPUT	(GPIO_INPUT|GPIO_PULLUP|GPIO_PORTE|GPIO_PIN11)
#define GPIO_GPI10_INPUT	(GPIO_INPUT|GPIO_PULLUP|GPIO_PORTE|GPIO_PIN13)
#define GPIO_GPI11_INPUT	(GPIO_INPUT|GPIO_PULLUP|GPIO_PORTE|GPIO_PIN14)
#define GPIO_GPI12_INPUT    (GPIO_INPUT|GPIO_PULLUP|GPIO_PORTE|GPIO_PIN3)
#define GPIO_GPI13_INPUT    (GPIO_INPUT|GPIO_PULLUP|GPIO_PORTE|GPIO_PIN2)
#define GPIO_GPI14_INPUT    (GPIO_INPUT|GPIO_PULLUP|GPIO_PORTE|GPIO_PIN4)

#define GPIO_GPIO0_OUTPUT	(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTA|GPIO_PIN1)
#define GPIO_GPIO1_OUTPUT	(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTA|GPIO_PIN2)
#define GPIO_GPIO2_OUTPUT	(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTA|GPIO_PIN3)
#define GPIO_GPIO3_OUTPUT	(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTB|GPIO_PIN5)
#define GPIO_GPIO4_OUTPUT	(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTB|GPIO_PIN0)
#define GPIO_GPIO5_OUTPUT	(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTB|GPIO_PIN1)
#define GPIO_GPIO6_OUTPUT	(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTD|GPIO_PIN13)
#define GPIO_GPIO7_OUTPUT	(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTD|GPIO_PIN12)
#define GPIO_GPIO8_OUTPUT	(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTE|GPIO_PIN9)
#define GPIO_GPIO9_OUTPUT	(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTE|GPIO_PIN11)
#define GPIO_GPI10_OUTPUT	(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTE|GPIO_PIN13)
#define GPIO_GPI11_OUTPUT	(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTE|GPIO_PIN14)
#define GPIO_GPI12_OUTPUT   (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTE|GPIO_PIN3)
#define GPIO_GPI13_OUTPUT   (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTE|GPIO_PIN2)
#define GPIO_GPI14_OUTPUT   (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTE|GPIO_PIN4)

/* Power supply control and monitoring GPIOs */



/* Tone alarm output */
#define TONE_ALARM_TIMER		5	/* timer 2 */
#define TONE_ALARM_CHANNEL		1	/* channel 1 */
#define GPIO_TONE_ALARM_IDLE	(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTA|GPIO_PIN0)
#define GPIO_TONE_ALARM			(GPIO_ALT|GPIO_AF2|GPIO_SPEED_2MHz|GPIO_PUSHPULL|GPIO_PORTA|GPIO_PIN0)

/* PWM
 *
 * Six PWM outputs are configured.
 *
 * Pins:
 *
 * CH1  : PA1  : TIM2_CH2
 * CH2  : PA2  : TIM2_CH3
 * CH3  : PA3  : TIM2_CH4
 * CH4  : PB5  : TIM3_CH2
 * CH5  : PB0  : TIM3_CH3
 * CH6  : PB1  : TIM3_CH4
 * CH7  : PD13 : TIM4_CH2
 * CH8  : PD12 : TIM4_CH1
 * CH9  : PE9  : TIM1_CH1
 * CH10 : PE11 : TIM1_CH2
 * CH11 : PE13 : TIM1_CH3
 * CH12 : PE14 : TIM1_CH4
 */
#define GPIO_TIM2_CH2OUT	(GPIO_ALT|GPIO_AF1|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PUSHPULL|GPIO_PORTA|GPIO_PIN1)
#define GPIO_TIM2_CH3OUT	(GPIO_ALT|GPIO_AF1|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PUSHPULL|GPIO_PORTA|GPIO_PIN2)
#define GPIO_TIM2_CH4OUT	(GPIO_ALT|GPIO_AF1|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PUSHPULL|GPIO_PORTA|GPIO_PIN3)
#define GPIO_TIM3_CH2OUT	(GPIO_ALT|GPIO_AF2|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PUSHPULL|GPIO_PORTB|GPIO_PIN5)
#define GPIO_TIM3_CH3OUT	(GPIO_ALT|GPIO_AF2|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PUSHPULL|GPIO_PORTB|GPIO_PIN0)
#define GPIO_TIM3_CH4OUT	(GPIO_ALT|GPIO_AF2|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PUSHPULL|GPIO_PORTB|GPIO_PIN1)
#define GPIO_TIM4_CH2OUT	(GPIO_ALT|GPIO_AF2|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PUSHPULL|GPIO_PORTD|GPIO_PIN13)
#define GPIO_TIM4_CH1OUT	(GPIO_ALT|GPIO_AF2|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PUSHPULL|GPIO_PORTD|GPIO_PIN12)
#define GPIO_TIM1_CH1OUT	(GPIO_ALT|GPIO_AF1|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PUSHPULL|GPIO_PORTE|GPIO_PIN9)
#define GPIO_TIM1_CH2OUT	(GPIO_ALT|GPIO_AF1|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PUSHPULL|GPIO_PORTE|GPIO_PIN11)
#define GPIO_TIM1_CH3OUT	(GPIO_ALT|GPIO_AF1|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PUSHPULL|GPIO_PORTE|GPIO_PIN13)
#define GPIO_TIM1_CH4OUT	(GPIO_ALT|GPIO_AF1|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PUSHPULL|GPIO_PORTE|GPIO_PIN14)
#define DIRECT_PWM_OUTPUT_CHANNELS	12

#define GPIO_TIM2_CH2IN		GPIO_TIM2_CH2IN_1
#define GPIO_TIM2_CH3IN		GPIO_TIM2_CH3IN_1
#define GPIO_TIM2_CH4IN		GPIO_TIM2_CH4IN_1
#define GPIO_TIM3_CH2IN		GPIO_TIM3_CH2IN_2
#define GPIO_TIM3_CH3IN		GPIO_TIM3_CH3IN_1
#define GPIO_TIM3_CH4IN		GPIO_TIM3_CH4IN_1
#define GPIO_TIM4_CH2IN		GPIO_TIM4_CH2IN_2
#define GPIO_TIM4_CH1IN		GPIO_TIM4_CH1IN_2
#define GPIO_TIM1_CH1IN		GPIO_TIM1_CH1IN_2
#define GPIO_TIM1_CH2IN		GPIO_TIM1_CH2IN_2
#define GPIO_TIM1_CH3IN		GPIO_TIM1_CH3IN_2
#define GPIO_TIM1_CH4IN		GPIO_TIM1_CH4IN_2
#define DIRECT_INPUT_TIMER_CHANNELS  12

/* USB OTG FS
 *
 * PA9  OTG_FS_VBUS VBUS sensing
 */
#define GPIO_OTGFS_VBUS		(GPIO_INPUT|GPIO_FLOAT|GPIO_SPEED_100MHz|GPIO_OPENDRAIN|GPIO_PORTA|GPIO_PIN9)

/* High-resolution timer */
#define HRT_TIMER			8	/* use timer8 for the HRT */
#define HRT_TIMER_CHANNEL	3	/* use capture/compare channel */

#define HRT_PPM_CHANNEL		4	/* use capture/compare channel 2 */
#define GPIO_PPM_IN			(GPIO_ALT|GPIO_AF3|GPIO_PULLUP|GPIO_PORTC|GPIO_PIN9)

#define RC_SERIAL_PORT		"/dev/ttyS3"

/* PWM input driver. Use FMU AUX5 pins attached to timer4 channel 2 */
#define PWMIN_TIMER			9
#define PWMIN_TIMER_CHANNEL	1
#define GPIO_PWM_IN			GPIO_TIM9_CH1IN_2


#define GPIO_LED_SAFETY			-1
//#define GPIO_BTN_SAFETY			(GPIO_INPUT|GPIO_PULLUP|GPIO_PORTC|GPIO_PIN8)


/* for R07, this signal is active low */




/* for R12, this signal is active high */
#define GPIO_SBUS_INV			(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTD|GPIO_PIN7)
#define INVERT_RC_INPUT(_s)		stm32_gpiowrite(GPIO_SBUS_INV, 1-_s);







/* Power switch controls ******************************************************/





// FMUv4 has a separate GPIO for serial RC output
#define GPIO_RC_OUT			(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTC|GPIO_PIN6)
#define SPEKTRUM_RX_AS_GPIO()		stm32_configgpio(GPIO_RC_OUT)
#define SPEKTRUM_RX_HIGH(_s)		stm32_gpiowrite(GPIO_RC_OUT, (_s))

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
 *   Called to configure SPI chip select GPIO pins for the VRBRAIN board.
 *
 ****************************************************************************************************/

extern void stm32_spiinitialize(void);

extern void stm32_usbinitialize(void);

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

#endif /* __ASSEMBLY__ */

__END_DECLS
