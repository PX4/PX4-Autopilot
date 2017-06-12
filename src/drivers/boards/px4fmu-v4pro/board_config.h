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
 * PX4FMUv2 internal definitions
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

/* PX4IO connection configuration */
//#define BOARD_USES_PX4IO_VERSION       2
#define PX4IO_SERIAL_DEVICE     "/dev/ttyS4"
#define PX4IO_SERIAL_TX_GPIO    GPIO_USART6_TX
#define PX4IO_SERIAL_RX_GPIO    GPIO_USART6_RX
#define PX4IO_SERIAL_BASE       STM32_USART6_BASE	/* hardwired on the board */
#define PX4IO_SERIAL_VECTOR     STM32_IRQ_USART6
#define PX4IO_SERIAL_TX_DMAMAP  DMAMAP_USART6_TX_2
#define PX4IO_SERIAL_RX_DMAMAP  DMAMAP_USART6_RX_2
#define PX4IO_SERIAL_CLOCK      STM32_PCLK2_FREQUENCY
#define PX4IO_SERIAL_BITRATE    1500000			/* 1.5Mbps -> max rate for IO */

/* Configuration ************************************************************************************/
//{GPIO_RSSI_IN,           0,                       0}, - pio Analog used as PWM
//{0,                      GPIO_LED_SAFETY,         0},	pio replacement
//{GPIO_SAFETY_SWITCH_IN,  0,                       0},   pio replacement
//{0,                      GPIO_PERIPH_3V3_EN,      0},	Owned by the 8266 driver
//{0,                      GPIO_SBUS_INV,           0},	https://github.com/PX4/Firmware/blob/master/src/modules/px4iofirmware/sbus.c
//{GPIO_8266_GPIO0,        0,                       0},   Owned by the 8266 driver
//{0,                      GPIO_SPEKTRUM_PWR_EN,     0},	Owned Spektum driver input to auto pilot
//{0,                      GPIO_8266_PD,            0},	Owned by the 8266 driver
//{0,                      GPIO_8266_RST,           0},	Owned by the 8266 driver

/* PX4FMU GPIOs ***********************************************************************************/
/* LEDs */

#define GPIO_LED1		(GPIO_OUTPUT|GPIO_OPENDRAIN|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTB|GPIO_PIN11)
#define GPIO_LED2		(GPIO_OUTPUT|GPIO_OPENDRAIN|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTB|GPIO_PIN1)
#define GPIO_LED3		(GPIO_OUTPUT|GPIO_OPENDRAIN|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTB|GPIO_PIN3)

#define GPIO_LED_RED 	GPIO_LED1
#define GPIO_LED_GREEN 	GPIO_LED2
#define GPIO_LED_BLUE   GPIO_LED3

/*  Define the Chip Selects */

/* SPI Bus 1 Internal Sensors */

#define GPIO_SPI_CS_MPU9250		(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTC|GPIO_PIN2)
#define GPIO_SPI_CS_LIS3MDL		(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTE|GPIO_PIN15)
#define GPIO_SPI_CS_MS5611		(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTD|GPIO_PIN7)
#define GPIO_SPI_CS_ICM_2060X		(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTC|GPIO_PIN15)
#define GPIO_SPI_CS_TEMPCAL_EEPROM	(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTH|GPIO_PIN5)

/* SPI Bus 2 Memory */

#define GPIO_SPI_CS_FRAM        (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTD|GPIO_PIN10)

/* Aux Chip selects (not pined out on RC0 HW) */

#define GPIO_AUX_CS0            (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTF|GPIO_PIN10)
#define GPIO_AUX_CS1            (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTF|GPIO_PIN11)
#define GPIO_AUX_CS2            (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTF|GPIO_PIN12)
#define GPIO_AUX_CS3            (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTF|GPIO_PIN13)
#define GPIO_AUX_CS4            (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTF|GPIO_PIN14)
#define GPIO_AUX_CS5            (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTF|GPIO_PIN15)


/* External Chip selects on Connectors ahead PAD3 */

#define GPIO_SPI5_CS            (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTF|GPIO_PIN6)
#define GPIO_SPI6_CS            (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTG|GPIO_PIN11)

/*  Define the Ready interrupts */

#define GPIO_DRDY_MPU9250       (GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI|GPIO_PORTD|GPIO_PIN15)
#define GPIO_DRDY_LIS3MDL       (GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI|GPIO_PORTE|GPIO_PIN12)
#define GPIO_DRDY_ICM_2060X   (GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI|GPIO_PORTC|GPIO_PIN14)


/*
 *  Define the ability to shut off off the sensor signals
 *  by changing the signals to inputs
 */

#define _PIN_OFF(def) (((def) & (GPIO_PORT_MASK | GPIO_PIN_MASK)) | (GPIO_INPUT|GPIO_PULLDOWN|GPIO_SPEED_2MHz))

#define GPIO_SPI_CS_OFF_MPU9250		_PIN_OFF(GPIO_SPI_CS_MPU9250)
#define GPIO_SPI_CS_OFF_LIS3MDL		_PIN_OFF(GPIO_SPI_CS_LIS3MDL)
#define GPIO_SPI_CS_OFF_MS5611		_PIN_OFF(GPIO_SPI_CS_MS5611)
#define GPIO_SPI_CS_OFF_ICM_2060X	_PIN_OFF(GPIO_SPI_CS_ICM_2060X)
#define GPIO_SPI_CS_OFF_TEMPCAL_EEPROM	_PIN_OFF(GPIO_SPI_CS_OFF_TEMPCAL_EEPROM)

#define GPIO_DRDY_OFF_MPU9250       _PIN_OFF(GPIO_DRDY_MPU9250)
#define GPIO_DRDY_OFF_LIS3MDL       _PIN_OFF(GPIO_DRDY_LIS3MDL)
#define GPIO_DRDY_OFF_ICM_2060X   _PIN_OFF(GPIO_DRDY_ICM_2060X)


/* SPI1 off */

#define GPIO_SPI1_SCK_OFF    _PIN_OFF(GPIO_SPI1_SCK)
#define GPIO_SPI1_MISO_OFF   _PIN_OFF(GPIO_SPI1_MISO)
#define GPIO_SPI1_MOSI_OFF   _PIN_OFF(GPIO_SPI1_MOSI)

#define PX4_SPI_BUS_SENSORS    1
#define PX4_SPI_BUS_RAMTRON    2
#define PX4_SPI_BUS_BARO       PX4_SPI_BUS_SENSORS
#define PX4_SPI_EXT0           5
#define PX4_SPI_EXT1           6

/* Use these in place of the spi_dev_e enumeration to select a specific SPI device on SPI1 */

#define PX4_SPIDEV_GYRO         1
#define PX4_SPIDEV_ACCEL_MAG    2
#define PX4_SPIDEV_BARO         3
#define PX4_SPIDEV_MPU          4
#define PX4_SPIDEV_HMC          5
#define PX4_SPIDEV_ICM          6
#define PX4_SPIDEV_LIS          7
#define PX4_SPIDEV_BMI          8
#define PX4_SPIDEV_BMA          9
#define PX4_SPIDEV_EXT0         10
#define PX4_SPIDEV_EXT1         11
#define PX4_SPIDEV_EEPROM	    12
#define PX4_SPIDEV_ICM_20608    13
#define PX4_SPIDEV_ICM_20602	14

/* I2C busses */
#define PX4_I2C_BUS_ONBOARD     1
#define PX4_I2C_BUS_EXPANSION   2
#define PX4_I2C_BUS_LED         PX4_I2C_BUS_ONBOARD

/* Devices on the external bus.
 *
 * Note that these are unshifted addresses.
 */
#define PX4_I2C_OBDEV_LED       0x55
#define PX4_I2C_OBDEV_HMC5883   0x1e
#define PX4_I2C_OBDEV_LIS3MDL   0x1e

/*
 * ADC channels
 *
 * These are the channel numbers of the ADCs of the microcontroller that can be used by the Px4 Firmware in the adc driver
 */
#define ADC_CHANNELS (1 << 2) | (1 << 3) | (1 << 4) | (1 << 10) | (1 << 11) | (1 << 12) | (1 << 13) | (1 << 14)

// ADC defines to be used in sensors.cpp to read from a particular channel
#define ADC_BATTERY_VOLTAGE_CHANNEL	2
#define ADC_BATTERY_CURRENT_CHANNEL	3
#define ADC_5V_RAIL_SENSE		4
#define ADC_BATTERY2_VOLTAGE_CHANNEL	11
#define ADC_BATTERY2_CURRENT_CHANNEL	13
//#define ADC_RC_RSSI_CHANNEL		11

// /* Define Battery 1 Voltage Divider and A per V
 // */

// #define BOARD_BATTERY1_V_DIV (6.490196078f)
// #define BOARD_BATTERY1_A_PER_V (26.4f)

// /* Define Battery 2 Voltage Divider and A per V
 // */

// #define BOARD_BATTERY2_V_DIV (6.490196078f)
// #define BOARD_BATTERY2_A_PER_V (26.4f)

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
#define GPIO_VDD_BRICK_VALID	(GPIO_INPUT|GPIO_PULLUP|GPIO_PORTB|GPIO_PIN5)
#define GPIO_VDD_BRICK2_VALID	(GPIO_INPUT|GPIO_PULLUP|GPIO_PORTG|GPIO_PIN5)
#define GPIO_VDD_3V3_SENSORS_EN	(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTE|GPIO_PIN3)
#define GPIO_VDD_3V3_PERIPH_EN	(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTC|GPIO_PIN5)
#define GPIO_VDD_5V_PERIPH_EN	(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTG|GPIO_PIN10)
#define GPIO_VDD_5V_HIPOWER_EN	(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTF|GPIO_PIN4)
#define GPIO_VDD_5V_PERIPH_OC	(GPIO_INPUT|GPIO_FLOAT|GPIO_PORTG|GPIO_PIN4)
#define GPIO_VDD_5V_HIPOWER_OC	(GPIO_INPUT|GPIO_FLOAT|GPIO_PORTF|GPIO_PIN3)
#define GPIO_VBUS_VALID		(GPIO_INPUT|GPIO_PULLUP|GPIO_PORTC|GPIO_PIN0)

/* Tone alarm output */
#define TONE_ALARM_TIMER	2	/* timer 2 */
#define TONE_ALARM_CHANNEL	1	/* channel 1 */
#define GPIO_TONE_ALARM_IDLE	(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTA|GPIO_PIN15)
#define GPIO_TONE_ALARM		(GPIO_ALT|GPIO_AF1|GPIO_SPEED_2MHz|GPIO_PUSHPULL|GPIO_PORTA|GPIO_PIN15)

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
#define HRT_TIMER           3 /* use timer 3 for the HRT */
#define HRT_TIMER_CHANNEL   4 /* use capture/compare channel 4 */

// #define HRT_PPM_CHANNEL     3	/* use capture/compare channel 3 */
// #define GPIO_PPM_IN         (GPIO_ALT|GPIO_AF2|GPIO_PULLUP|GPIO_PORTB|GPIO_PIN0)

/* PWM input driver. Use FMU AUX5 pins attached to timer4 channel 2 */
#define PWMIN_TIMER         4
#define PWMIN_TIMER_CHANNEL 2
#define GPIO_PWM_IN			GPIO_TIM4_CH2IN_2

// #define GPIO_BTN_SAFETY_FMU		(GPIO_INPUT|GPIO_FLOAT|GPIO_PORTC|GPIO_PIN4)
// #define GPIO_SBUS_INV			(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTC|GPIO_PIN13)
// #define INVERT_RC_INPUT(_invert_true)  px4_arch_gpiowrite(GPIO_SBUS_INV, _invert_true)

// #define GPIO_8266_GPIO0			(GPIO_INPUT|GPIO_PULLUP|GPIO_PORTE|GPIO_PIN2)
// #define GPIO_SPEKTRUM_PWR_EN		(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTE|GPIO_PIN4)
// #define GPIO_8266_PD			(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTE|GPIO_PIN5)
// #define GPIO_8266_RST			(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTE|GPIO_PIN6)

// /* No Power switch controls or binding control *********************************************
 // * V4 Pro does not have control to bind SPEKTRUM - there is only 5V VCC on
 // * the connector interface and Spektrum requires VDD 3v3 to be controllable
 // */

// #define	BOARD_NAME "PX4FMU_V4PRO"

// /* By Providing BOARD_ADC_USB_CONNECTED (using the px4_arch abstraction)
 // * this board support the ADC system_power interface, and therefore
 // * provides the true logic GPIO BOARD_ADC_xxxx macros.
 // */
// #define BOARD_ADC_USB_CONNECTED (px4_arch_gpioread(GPIO_OTGFS_VBUS))
// #define BOARD_ADC_BRICK_VALID   (px4_arch_gpioread(GPIO_VDD_BRICK_VALID))
// #define BOARD_ADC_BRICK2_VALID  (px4_arch_gpioread(GPIO_VDD_BRICK2_VALID))
// #define BOARD_ADC_SERVO_VALID   (1)
// #define BOARD_ADC_PERIPH_5V_OC  (!px4_arch_gpioread(GPIO_VDD_5V_PERIPH_OC))
// #define BOARD_ADC_HIPOWER_5V_OC (!px4_arch_gpioread(GPIO_VDD_5V_HIPOWER_OC))

// #define BOARD_HAS_PWM	DIRECT_PWM_OUTPUT_CHANNELS


 // * GPIO numbers.
 // *
 // * There are no alternate functions on this board.
 // */
// #define GPIO_SERVO_1           (1<<0)  /**< servo 1 output */
// #define GPIO_SERVO_2           (1<<1)  /**< servo 2 output */
// #define GPIO_SERVO_3           (1<<2)  /**< servo 3 output */
// #define GPIO_SERVO_4           (1<<3)  /**< servo 4 output */
// #define GPIO_SERVO_5           (1<<4)  /**< servo 5 output */
// #define GPIO_SERVO_6           (1<<5)  /**< servo 6 output */

// #define GPIO_3V3_SENSORS_EN    (1<<6)  /**< PE3  - GPIO_VDD_3V3_SENSORS_EN */
// #define GPIO_3V3_PERIPH_EN     (1<<7)  /**< PC5  - GPIO_VDD_3V3_PERIPH_EN  */

// #define GPIO_BRICK_VALID       (1<<8)  /**< PB5  - !GPIO_VDD_BRICK_VALID */
// #define GPIO_BRICK2_VALID      (1<<9)  /**< PG5  - !GPIO_VDD_BRICK2_VALID */
// #define GPIO_USB_VBUS_VALID    (1<<10) /**< PC0  - !GPIO_VBUS_VALID */

// #define GPIO_5V_HIPOWER_OC     (1<<11) /**< PF3  - !GPIO_VDD_5V_RC_OC */
// #define GPIO_5V_PERIPH_OC      (1<<12) /**< PE10 - !GPIO_VDD_5V_PERIPH_OC */



// /* This board provides a DMA pool and APIs */

// #define BOARD_DMA_ALLOC_POOL_SIZE 5120

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
//void board_spi_reset(int ms);

extern void stm32_usbinitialize(void);

//extern void board_peripheral_reset(int ms);


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

//#include "../common/board_common.h"

#endif /* __ASSEMBLY__ */

__END_DECLS
