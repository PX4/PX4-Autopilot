/****************************************************************************
 *
 *   Copyright (c) 2015-2016 Dronesmith Technologies. All rights reserved.
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
 * Luci-v1 internal definitions
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

/* FIXME Luci Revision E2 Error Patches (Remove this after revision E2) */
#define LUCI_NO_MMCSD /* Rev E2's SD Card is unusable, and trying to init mmcsd will cause the FMU to crash. */

/* Sensor rotation for LSM Gyro */
#define SENSOR_BOARD_ROTATION_DEFAULT		(2) /* SENSOR_BOARD_ROTATION_180_DEG */

/****************************************************************************************************
 * Definitions
 ****************************************************************************************************/

/* PX4FMU GPIOs ***********************************************************************************/

/* LEDs */
#define GPIO_LED1		(GPIO_OUTPUT|GPIO_OPENDRAIN|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTE|GPIO_PIN12)
#define GPIO_LED_RED 	GPIO_LED1

/*  Define the Chip Selects */
#define GPIO_SPI_CS_LSM9DS0_GYRO			(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTC|GPIO_PIN1)
#define GPIO_SPI_CS_LSM9DS0_ACCEL_MAG (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTC|GPIO_PIN2)
#define GPIO_SPI_CS_MPU9250						(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTE|GPIO_PIN3)
#define GPIO_SPI_CS_MS5611						(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTB|GPIO_PIN5)

#define GPIO_SPI_CS_FRAM							(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTD|GPIO_PIN10)

#define GPIO_SPI_CS_EXT0							(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTB|GPIO_PIN0)
#define GPIO_SPI_CS_EXT1							(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTB|GPIO_PIN1)

/*  Define the Ready interrupts */
#define GPIO_DRDY_LSM9DS0_GYRO				(GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI|GPIO_PORTC|GPIO_PIN13)
#define GPIO_DRDY_LSM9DS0_XM1					(GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI|GPIO_PORTC|GPIO_PIN15)
#define GPIO_DRDY_LSM9DS0_XM2 				(GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI|GPIO_PORTC|GPIO_PIN0)
#define GPIO_DRDY_MPU9250							(GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI|GPIO_PORTE|GPIO_PIN4)

/*
 *  Define the ability to shut off off the sensor signals
 *  by changing the signals to inputs
 */

#define _PIN_OFF(def) (((def) & (GPIO_PORT_MASK | GPIO_PIN_MASK)) | (GPIO_INPUT|GPIO_PULLDOWN|GPIO_SPEED_2MHz))

#define GPIO_SPI_CS_OFF_LSM9DS0_GYRO			_PIN_OFF(GPIO_SPI_CS_LSM9DS0_GYRO)
#define GPIO_SPI_CS_OFF_LSM9DS0_ACCEL_MAG	_PIN_OFF(GPIO_SPI_CS_LSM9DS0_ACCEL_MAG)
#define GPIO_SPI_CS_OFF_MPU9250						_PIN_OFF(GPIO_SPI_CS_MPU9250)
#define GPIO_SPI_CS_OFF_MS5611						_PIN_OFF(GPIO_SPI_CS_MS5611)

#define GPIO_SPI_CS_OFF_EXT0							_PIN_OFF(GPIO_SPI_CS_EXT0)
#define GPIO_SPI_CS_OFF_EXT1							_PIN_OFF(GPIO_SPI_CS_EXT1)

#define GPIO_DRDY_OFF_LSM9DS0_GYRO				_PIN_OFF(GPIO_DRDY_LSM9DS0_GYRO)
#define GPIO_DRDY_OFF_LSM9DS0_XM1					_PIN_OFF(GPIO_DRDY_LSM9DS0_XM1)
#define GPIO_DRDY_OFF_LSM9DS0_XM2 				_PIN_OFF(GPIO_DRDY_LSM9DS0_XM2)
#define GPIO_DRDY_OFF_MPU9250							_PIN_OFF(GPIO_DRDY_MPU9250)

/* SPI1 Off */
#define GPIO_SPI1_SCK_OFF									_PIN_OFF(GPIO_SPI1_SCK)
#define GPIO_SPI1_MISO_OFF								_PIN_OFF(GPIO_SPI1_MISO)
#define GPIO_SPI1_MOSI_OFF								_PIN_OFF(GPIO_SPI1_MOSI)

#define PX4_SPI_BUS_SENSORS	4
#define PX4_SPI_BUS_RAMTRON	2
#define PX4_SPI_BUS_EXT			1
#define PX4_SPI_BUS_BARO		PX4_SPI_BUS_SENSORS

/* External bus */
#define PX4_SPIDEV_EXT0		1
#define PX4_SPIDEV_EXT1		2

/* With only 2 external chip selects, not sure how useful these will be... */
#define PX4_SPIDEV_EXT_MPU		PX4_SPIDEV_EXT0
#define PX4_SPIDEV_EXT_BARO		PX4_SPIDEV_EXT1
#define PX4_SPIDEV_EXT_ACCEL_MAG	PX4_SPIDEV_EXT0
#define PX4_SPIDEV_EXT_GYRO		PX4_SPIDEV_EXT1

#define PX4_SPIDEV_EXT_BMI		PX4_SPIDEV_EXT0

/* Use these in place of the spi_dev_e enumeration to select a specific SPI device on SPI4 */
#define PX4_SPIDEV_GYRO			1
#define PX4_SPIDEV_ACCEL_MAG		2
#define PX4_SPIDEV_BARO			3
#define PX4_SPIDEV_MPU			4

/* I2C busses */
#define PX4_I2C_BUS_EXPANSION	1
#define PX4_I2C_BUS_ONBOARD		2
#define PX4_I2C_BUS_LED				PX4_I2C_BUS_ONBOARD

/*
 * Devices on the external bus.
 *
 * Note that these are unshifted addresses.
 */
#define PX4_I2C_OBDEV_LED	0x62
#define PX4_I2C_OBDEV_HMC5883	0x1e
#define PX4_I2C_OBDEV_LIS3MDL	0x1e

/*
 * ADC channels
 *
 * Luci uses channels 14 and 15 for the external HDI connector.
 */
#define ADC_CHANNELS (1 << 2) | (1 << 3) | (1 << 4) | (1 << 14) | (1 << 15)

// ADC defines to be used in sensors.cpp to read from a particular channel
#define ADC_BATTERY_VOLTAGE_CHANNEL		2
#define ADC_BATTERY_CURRENT_CHANNEL		3
#define ADC_5V_RAIL_SENSE							4
#define ADC_EXT_CHANNEL1							14
#define ADC_EXT_CHANNEL2							15

/* User GPIOs
 *
 * GPIO0-5 are the PWM servo outputs.
 */
#define GPIO_GPIO0_INPUT	(GPIO_INPUT|GPIO_PULLUP|GPIO_PORTE|GPIO_PIN14)
#define GPIO_GPIO1_INPUT	(GPIO_INPUT|GPIO_PULLUP|GPIO_PORTE|GPIO_PIN13)
#define GPIO_GPIO2_INPUT	(GPIO_INPUT|GPIO_PULLUP|GPIO_PORTE|GPIO_PIN11)
#define GPIO_GPIO3_INPUT	(GPIO_INPUT|GPIO_PULLUP|GPIO_PORTE|GPIO_PIN9)
#define GPIO_GPIO4_INPUT	(GPIO_INPUT|GPIO_PULLUP|GPIO_PORTD|GPIO_PIN13)
#define GPIO_GPIO5_INPUT	(GPIO_INPUT|GPIO_PULLUP|GPIO_PORTD|GPIO_PIN14)

#define GPIO_GPIO0_OUTPUT	(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTE|GPIO_PIN14)
#define GPIO_GPIO1_OUTPUT	(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTE|GPIO_PIN13)
#define GPIO_GPIO2_OUTPUT	(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTE|GPIO_PIN11)
#define GPIO_GPIO3_OUTPUT	(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTE|GPIO_PIN9)
#define GPIO_GPIO4_OUTPUT	(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTD|GPIO_PIN13)
#define GPIO_GPIO5_OUTPUT	(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTD|GPIO_PIN14)

/* Power supply control and monitoring GPIOs */
#define GPIO_VDD_5V_PERIPH_EN	(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTB|GPIO_PIN7)
#define GPIO_VDD_5V_TELEM_OC	(GPIO_INPUT|GPIO_PULLUP|GPIO_PORTD|GPIO_PIN7)
#define GPIO_VDD_5V_PERIPH_OC	(GPIO_INPUT|GPIO_PULLUP|GPIO_PORTB|GPIO_PIN4)

/* FIXME 3V3 rail never added in hardware, will be fixed on next revision */
#define GPIO_VDD_3V3_SENSORS_EN	(0) /*(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTC|GPIO_PIN3)*/


/* Tone alarm output */
#define TONE_ALARM_TIMER		2	/* timer 2 */
#define TONE_ALARM_CHANNEL		1	/* channel 1 */
#define GPIO_TONE_ALARM_IDLE	(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTA|GPIO_PIN15)
#define GPIO_TONE_ALARM			(GPIO_ALT|GPIO_AF1|GPIO_SPEED_2MHz|GPIO_PUSHPULL|GPIO_PORTA|GPIO_PIN15)

/* PWM
 *
 * Six PWM outputs are configured.
 *
 * Pins:
 *
 * CH1 : PE14 : TIM1_CH4
 * CH2 : PE13 : TIM1_CH3
 * CH3 : PE11 : TIM1_CH2
 * CH4 : PE9  : TIM1_CH1
 * CH5 : PD13 : TIM4_CH2
 * CH6 : PD14 : TIM4_CH3
 */
#define GPIO_TIM1_CH1OUT	(GPIO_ALT|GPIO_AF1|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PUSHPULL|GPIO_PORTE|GPIO_PIN9)
#define GPIO_TIM1_CH2OUT	(GPIO_ALT|GPIO_AF1|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PUSHPULL|GPIO_PORTE|GPIO_PIN11)
#define GPIO_TIM1_CH3OUT	(GPIO_ALT|GPIO_AF1|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PUSHPULL|GPIO_PORTE|GPIO_PIN13)
#define GPIO_TIM1_CH4OUT	(GPIO_ALT|GPIO_AF1|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PUSHPULL|GPIO_PORTE|GPIO_PIN14)
#define GPIO_TIM4_CH2OUT	(GPIO_ALT|GPIO_AF2|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PUSHPULL|GPIO_PORTD|GPIO_PIN13)
#define GPIO_TIM4_CH3OUT	(GPIO_ALT|GPIO_AF2|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PUSHPULL|GPIO_PORTD|GPIO_PIN14)
#define DIRECT_PWM_OUTPUT_CHANNELS	6

#define GPIO_TIM1_CH1IN		GPIO_TIM1_CH1IN_2
#define GPIO_TIM1_CH2IN		GPIO_TIM1_CH2IN_2
#define GPIO_TIM1_CH3IN		GPIO_TIM1_CH3IN_2
#define GPIO_TIM1_CH4IN		GPIO_TIM1_CH4IN_2
#define GPIO_TIM4_CH2IN		GPIO_TIM4_CH2IN_2
#define GPIO_TIM4_CH3IN		GPIO_TIM4_CH3IN_2
#define DIRECT_INPUT_TIMER_CHANNELS  6

/* USB OTG FS
 *
 * PA9  OTG_FS_VBUS VBUS sensing
 */
#define GPIO_OTGFS_VBUS		(GPIO_INPUT|GPIO_FLOAT|GPIO_SPEED_100MHz|GPIO_OPENDRAIN|GPIO_PORTA|GPIO_PIN9)

/* High-resolution timer */
#define HRT_TIMER		8	/* use timer8 for the HRT */
#define HRT_TIMER_CHANNEL	1	/* use capture/compare channel */

#define HRT_PPM_CHANNEL		3	/* use capture/compare channel 2 */
#define GPIO_PPM_IN			(GPIO_ALT|GPIO_AF2|GPIO_PULLUP|GPIO_PORTA|GPIO_PIN10)

#define RC_SERIAL_PORT		"/dev/ttyS5"

/* PWM input driver. Use FMU AUX5 pins attached to timer4 channel 2 */
#define PWMIN_TIMER			4
#define PWMIN_TIMER_CHANNEL	2
#define GPIO_PWM_IN			GPIO_TIM4_CH2IN_2

/* FIXME - Currently not implemented on Luci, but we'd like to have this on future revisions. */
#define GPIO_SBUS_INV		(0)	/*(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTC|GPIO_PIN13)*/
#define INVERT_RC_INPUT(_s)		px4_arch_gpiowrite(GPIO_SBUS_INV, _s);

#define GPIO_SPEKTRUM_PWR_EN		(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTD|GPIO_PIN15)

/* Power switch controls ******************************************************/

#define POWER_SPEKTRUM(_s)			px4_arch_gpiowrite(GPIO_SPEKTRUM_PWR_EN, (1-_s))
#define GPIO_UART7_RX_SPEKTRUM		(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTE|GPIO_PIN7)
#define SPEKTRUM_RX_AS_UART()		px4_arch_configgpio(GPIO_UART7_RX)

// NOTE - On PX4 v4, this is a different GPIO, on Luci, they are same.
#define GPIO_RC_OUT							(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTE|GPIO_PIN7)
#define SPEKTRUM_RX_AS_GPIO()		px4_arch_configgpio(GPIO_RC_OUT)
#define SPEKTRUM_RX_HIGH(_s)		px4_arch_gpiowrite(GPIO_RC_OUT, (_s))


#define	BOARD_NAME "LUCI_V1"

/* By Providing BOARD_ADC_USB_CONNECTED this board support the ADC
 * system_power interface, and herefore provides the true logic
 * GPIO BOARD_ADC_xxxx macros.
 */
#define BOARD_ADC_USB_CONNECTED (px4_arch_gpioread(GPIO_OTGFS_VBUS))
#define BOARD_ADC_PERIPH_5V_OC  (!px4_arch_gpioread(GPIO_VDD_5V_PERIPH_OC))
#define BOARD_ADC_5V_TELEM_OC 	(!px4_arch_gpioread(GPIO_VDD_5V_TELEM_OC))

/* Not supported on this board, but needed for adc driver */
#define BOARD_ADC_BRICK_VALID (1)
#define BOARD_ADC_SERVO_VALID (1)
#define BOARD_ADC_HIPOWER_5V_OC (0)

#define BOARD_HAS_PWM	DIRECT_PWM_OUTPUT_CHANNELS

#define BOARD_FMU_GPIO_TAB { \
		{GPIO_GPIO0_INPUT,       GPIO_GPIO0_OUTPUT,       0}, \
		{GPIO_GPIO1_INPUT,       GPIO_GPIO1_OUTPUT,       0}, \
		{GPIO_GPIO2_INPUT,       GPIO_GPIO2_OUTPUT,       0}, \
		{GPIO_GPIO3_INPUT,       GPIO_GPIO3_OUTPUT,       0}, \
		{GPIO_GPIO4_INPUT,       GPIO_GPIO4_OUTPUT,       0}, \
		{GPIO_GPIO5_INPUT,       GPIO_GPIO5_OUTPUT,       0}, \
		{0,                      GPIO_VDD_5V_PERIPH_EN,   0}, \
    {0,                      GPIO_VDD_3V3_SENSORS_EN, 0}, \
		{GPIO_VDD_5V_TELEM_OC,   0, 											0}, \
		{GPIO_VDD_5V_PERIPH_OC,  0,                       0}, }

/* This board provides a DMA pool and APIs */

#define BOARD_DMA_ALLOC_POOL_SIZE 5120

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

#include "../common/board_common.h"

#endif /* __ASSEMBLY__ */

__END_DECLS
