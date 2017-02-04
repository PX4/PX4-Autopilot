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
 * PX4NUCLEOF767ZI-v1 internal definitions
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
#include <chip.h>
#include <stm32_gpio.h>
#include <arch/board/board.h>

/****************************************************************************************************
 * Definitions
 ****************************************************************************************************/
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
/*                                Port[CON-PIN] FMUv5 Delta */
#define GPIO_LED1              /* PB14[CN12-28] DRDY2   */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTB|GPIO_PIN14)
#define GPIO_LED2              /* PB0[CN11-34]  RSSI    */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTB|GPIO_PIN0)
#define GPIO_LED3              /* PB7[CN11-21]  GPS1_TX */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTB|GPIO_PIN7)

#define GPIO_LED_RED 	GPIO_LED1
#define GPIO_LED_GREEN 	GPIO_LED2
#define GPIO_LED_BLUE   GPIO_LED3

/*  Define the Chip Selects */

#define GPIO_SPI_CS_MPU9250     /* PF2[CN11-52]  */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTF|GPIO_PIN2)
#define GPIO_SPI_CS_HMC5983     /* PF3[CN12-58]  */	(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTF|GPIO_PIN3)
#define GPIO_SPI_CS_LIS3MDL     /* PF4[CN12-38]  */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTF|GPIO_PIN4)

#define GPIO_SPI_CS_FRAM        /* PF5[CN12-36] */(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTF|GPIO_PIN5)

#define GPIO_SPI_CS_MS5611      /* PF10[CN12-42] */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTF|GPIO_PIN10)

#define GPIO_SPI_CS_ICM_20608_G /* PF13[CN12-57] */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTF|GPIO_PIN13)


/*  Define the Ready interrupts */

#define GPIO_DRDY_MPU9250       /* PB4[CN12-27]  */ (GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI|GPIO_PORTB|GPIO_PIN4)
#define GPIO_DRDY_HMC5983       /* PB15[CN12-26] */ (GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI|GPIO_PORTB|GPIO_PIN15)
#define GPIO_DRDY_ICM_20608_G   /* PC5[CN12-6]   */ (GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI|GPIO_PORTC|GPIO_PIN5)

/*
 *  Define the ability to shut off off the sensor signals
 *  by changing the signals to inputs
 */

#define _PIN_OFF(def) (((def) & (GPIO_PORT_MASK | GPIO_PIN_MASK)) | (GPIO_INPUT|GPIO_PULLDOWN|GPIO_SPEED_2MHz))

#define GPIO_SPI_CS_OFF_MPU9250		_PIN_OFF(GPIO_SPI_CS_MPU9250)
#define GPIO_SPI_CS_OFF_HMC5983		_PIN_OFF(GPIO_SPI_CS_HMC5983)
#define GPIO_SPI_CS_OFF_LIS3MDL		_PIN_OFF(GPIO_SPI_CS_LIS3MDL)
#define GPIO_SPI_CS_OFF_MS5611		_PIN_OFF(GPIO_SPI_CS_MS5611)
#define GPIO_SPI_CS_OFF_ICM_20608_G _PIN_OFF(GPIO_SPI_CS_ICM_20608_G)

#define GPIO_DRDY_OFF_MPU9250		_PIN_OFF(GPIO_DRDY_MPU9250)
#define GPIO_DRDY_OFF_HMC5983		_PIN_OFF(GPIO_DRDY_HMC5983)
#define GPIO_DRDY_OFF_ICM_20608_G	_PIN_OFF(GPIO_DRDY_ICM_20608_G)


/* SPI1 off */
#define GPIO_SPI1_SCK_OFF	_PIN_OFF(GPIO_SPI1_SCK)
#define GPIO_SPI1_MISO_OFF	_PIN_OFF(GPIO_SPI1_MISO)
#define GPIO_SPI1_MOSI_OFF	_PIN_OFF(GPIO_SPI1_MOSI)

/* SENSORS are on SPI1
 * FRAM is on bus SPI4
 * MS5611 is on bus SPI6
 */
#define PX4_SPI_BUS_SENSORS	1
#define PX4_SPI_BUS_RAMTRON	4
#define PX4_SPI_BUS_BARO    5
#define PX4_SPI_BUS_ICM     6

#define PX4_SPIDEV_GYRO			 PX4_MK_SPI_SEL(PX4_SPI_BUS_SENSORS,0)
#define PX4_SPIDEV_ACCEL_MAG	 PX4_MK_SPI_SEL(PX4_SPI_BUS_SENSORS,1)
#define PX4_SPIDEV_MPU			 PX4_MK_SPI_SEL(PX4_SPI_BUS_SENSORS,2)
#define PX4_SPIDEV_HMC			 PX4_MK_SPI_SEL(PX4_SPI_BUS_SENSORS,3)
#define PX4_SPIDEV_LIS           PX4_MK_SPI_SEL(PX4_SPI_BUS_SENSORS,4)
#define PX4_SPIDEV_BMI           PX4_MK_SPI_SEL(PX4_SPI_BUS_SENSORS,5)
#define PX4_SPIDEV_BMA           PX4_MK_SPI_SEL(PX4_SPI_BUS_SENSORS,6)

#define PX4_SENSOR_BUS_CS_GPIO   {0, 0, GPIO_SPI_CS_MPU9250, GPIO_SPI_CS_HMC5983, GPIO_SPI_CS_LIS3MDL, 0, 0}
#define PX4_SENSORS_BUS_FIRST_CS PX4_SPIDEV_GYRO
#define PX4_SENSORS_BUS_LAST_CS  PX4_SPIDEV_BMA

#define PX4_SPIDEV_FRAM          PX4_MK_SPI_SEL(PX4_SPI_BUS_RAMTRON,0)
#define PX4_RAMTRON_BUS_CS_GPIO  {GPIO_SPI_CS_FRAM}
#define PX4_RAMTRON_BUS_FIRST_CS PX4_SPIDEV_FRAM
#define PX4_RAMTRON_BUS_LAST_CS  PX4_SPIDEV_FRAM

#define PX4_SPIDEV_BARO          PX4_MK_SPI_SEL(PX4_SPI_BUS_BARO,0)
#define PX4_BARO_BUS_CS_GPIO     {GPIO_SPI_CS_MS5611}
#define PX4_BARO_BUS_FIRST_CS    PX4_SPIDEV_BARO
#define PX4_BARO_BUS_LAST_CS     PX4_SPIDEV_BARO

#define PX4_SPIDEV_ICM           PX4_MK_SPI_SEL(PX4_SPI_BUS_ICM,0)
#define PX4_ICM_BUS_CS_GPIO      {GPIO_SPI_CS_ICM_20608_G}
#define PX4_ICM_BUS_FIRST_CS     PX4_SPIDEV_ICM
#define PX4_ICM_BUS_LAST_CS      PX4_SPIDEV_ICM

/* I2C busses */
#define PX4_I2C_BUS_EXPANSION	4
#define PX4_I2C_BUS_LED			PX4_I2C_BUS_EXPANSION

/* Devices on the external bus.
 *
 * Note that these are unshifted addresses.
 */
#define PX4_I2C_OBDEV_LED	    0x55
#define PX4_I2C_OBDEV_HMC5883	0x1e
#define PX4_I2C_OBDEV_LIS3MDL	0x1e

/*
 * ADC channels
 *
 * These are the channel numbers of the ADCs of the microcontroller that
 * can be used by the Px4 Firmware in the adc driver
 */
#define ADC_CHANNELS (1 << 0) | (1 << 1) | (1 << 2) | (1 << 3) | (1 << 4) | \
	(1 << 8) | \
	(1 << 10) | (1 << 11) | (1 << 12) | (1 << 13) | (1 << 14)

// ADC defines to be used in sensors.cpp to read from a particular channel
#define ADC_BATTERY_VOLTAGE_CHANNEL		0
#define ADC_BATTERY_CURRENT_CHANNEL		1
#define ADC_5V_RAIL_SENSE				10
#define ADC_RC_RSSI_CHANNEL				14

/* User GPIOs
 *
 * GPIO0-5 are the PWM servo outputs.
 */
#define GPIO_GPIO0_INPUT        /* PE14[CN12-51] */ (GPIO_INPUT|GPIO_PULLUP|GPIO_PORTE|GPIO_PIN14)
#define GPIO_GPIO1_INPUT        /* PA10[CN12-33] */ (GPIO_INPUT|GPIO_PULLUP|GPIO_PORTA|GPIO_PIN10)
#define GPIO_GPIO2_INPUT        /* PE11[CN12-56] */ (GPIO_INPUT|GPIO_PULLUP|GPIO_PORTE|GPIO_PIN11)
#define GPIO_GPIO3_INPUT        /* PE9[CN12-52]  */ (GPIO_INPUT|GPIO_PULLUP|GPIO_PORTE|GPIO_PIN9)
#define GPIO_GPIO4_INPUT        /* PD13[CN12-41] */ (GPIO_INPUT|GPIO_PULLUP|GPIO_PORTD|GPIO_PIN13)
#define GPIO_GPIO5_INPUT        /* PD14[CN12-46] */ (GPIO_INPUT|GPIO_PULLUP|GPIO_PORTD|GPIO_PIN14)

#define GPIO_GPIO0_OUTPUT       /* PE14[CN12-51] */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTE|GPIO_PIN14)
#define GPIO_GPIO1_OUTPUT       /* PE13[CN12-55] */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTE|GPIO_PIN13)
#define GPIO_GPIO2_OUTPUT       /* PE11[CN12-56] */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTE|GPIO_PIN11)
#define GPIO_GPIO3_OUTPUT       /* PE9[CN12-52]  */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTE|GPIO_PIN9)
#define GPIO_GPIO4_OUTPUT       /* PD13[CN12-41] */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTD|GPIO_PIN13)
#define GPIO_GPIO5_OUTPUT       /* PD14[CN12-46] */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTD|GPIO_PIN14)

/* Power supply control and monitoring GPIOs */
#define GPIO_VDD_BRICK_VALID    /* PB10[CN12-25] */ (GPIO_INPUT|GPIO_PULLUP|GPIO_PORTB|GPIO_PIN10)
#define GPIO_VDD_3V3_SENSORS_EN /* PE3[CN11-47] */	(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTE|GPIO_PIN3)

/* Tone alarm output */
#define TONE_ALARM_TIMER		2	/* timer 2 */
#define TONE_ALARM_CHANNEL      /* PA5[CN12-11] TIM2_CH1 */ 1	/* channel 1 */
#define GPIO_TONE_ALARM_IDLE    /* PA5[CN12-11] */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTA|GPIO_PIN5)
#define GPIO_TONE_ALARM         /* PA5[CN12-11] */ (GPIO_ALT|GPIO_AF1|GPIO_SPEED_2MHz|GPIO_PUSHPULL|GPIO_PORTA|GPIO_PIN5)

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
#define GPIO_TIM1_CH1OUT        /* PE14[CN12-51] */ (GPIO_ALT|GPIO_AF1|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PUSHPULL|GPIO_PORTE|GPIO_PIN9)
#define GPIO_TIM1_CH2OUT        /* PE13[CN12-55] */ (GPIO_ALT|GPIO_AF1|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PUSHPULL|GPIO_PORTE|GPIO_PIN11)
#define GPIO_TIM1_CH3OUT        /* PE11[CN12-56] */ (GPIO_ALT|GPIO_AF1|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PUSHPULL|GPIO_PORTE|GPIO_PIN13)
#define GPIO_TIM1_CH4OUT        /* PE9[CN12-52]  */ (GPIO_ALT|GPIO_AF1|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PUSHPULL|GPIO_PORTE|GPIO_PIN14)
#define GPIO_TIM4_CH2OUT        /* PD13[CN12-41] */ (GPIO_ALT|GPIO_AF2|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PUSHPULL|GPIO_PORTD|GPIO_PIN13)
#define GPIO_TIM4_CH3OUT        /* PD14[CN12-46] */ (GPIO_ALT|GPIO_AF2|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PUSHPULL|GPIO_PORTD|GPIO_PIN14)
#define DIRECT_PWM_OUTPUT_CHANNELS	6

#define GPIO_TIM1_CH1IN         /* PE14[CN12-51] */ GPIO_TIM1_CH1IN_2
#define GPIO_TIM1_CH2IN         /* PE13[CN12-55] */ GPIO_TIM1_CH2IN_2
#define GPIO_TIM1_CH3IN         /* PE11[CN12-56] */ GPIO_TIM1_CH3IN_2
#define GPIO_TIM1_CH4IN         /* PE9[CN12-52]  */ GPIO_TIM1_CH4IN_2
#define GPIO_TIM4_CH2IN         /* PD13[CN12-41] */ GPIO_TIM4_CH2IN_2
#define GPIO_TIM4_CH3IN         /* PD14[CN12-46] */ GPIO_TIM4_CH3IN_2
#define DIRECT_INPUT_TIMER_CHANNELS  6

/* USB OTG FS
 *
 * PA9  OTG_FS_VBUS VBUS sensing
 */
#define GPIO_OTGFS_VBUS         /* PA9[CN12-21] */ (GPIO_INPUT|GPIO_FLOAT|GPIO_SPEED_100MHz|GPIO_OPENDRAIN|GPIO_PORTA|GPIO_PIN9)

/* High-resolution timer */
#define HRT_TIMER		    8	/* use timer8 for the HRT */
#define HRT_TIMER_CHANNEL   3	/* use capture/compare channel 3 */

#define HRT_PPM_CHANNEL         /* PA7[CN12-15] */  1	/* use capture/compare channel 1 */
#define GPIO_PPM_IN             /* PB0[CN11-34] */ GPIO_TIM3_CH3IN_1

#define RC_SERIAL_PORT		"/dev/ttyS4"

/* PWM input driver. Use FMU AUX5 pins attached to timer4 channel 2 */
#define PWMIN_TIMER			4
#define PWMIN_TIMER_CHANNEL     /* PD13[CN12-41] */ 2
#define GPIO_PWM_IN             /* PD13[CN12-41] */ GPIO_TIM4_CH2IN_2

#define GPIO_RSSI_IN            /* PC4[CN12-34]  */	(GPIO_INPUT|GPIO_PULLUP|GPIO_PORTC|GPIO_PIN4)
#define GPIO_LED_SAFETY         /* PE12[CN12-49] */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTE|GPIO_PIN12)
#define GPIO_BTN_SAFETY         /* PE10[CN12-47] */ (GPIO_INPUT|GPIO_PULLUP|GPIO_PORTE|GPIO_PIN10)
#define GPIO_PERIPH_3V3_EN      /* PG4[CN12-69]  */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTG|GPIO_PIN4)

#define GPIO_SBUS_INV		    /* PD10[CN12-65] */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTD|GPIO_PIN10)
#define INVERT_RC_INPUT(_s)		px4_arch_gpiowrite(GPIO_SBUS_INV, _s);

#define GPIO_8266_GPIO0         /* PD15[CN12-48] */ (GPIO_INPUT|GPIO_PULLUP|GPIO_PORTD|GPIO_PIN15)
#define GPIO_SPEKTRUM_PWR_EN    /* PE4[CN11-48]  */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTE|GPIO_PIN4)
#define GPIO_8266_PD            /* PE7[CN12-44]  */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTE|GPIO_PIN7)
#define GPIO_8266_RST           /* PG10[CN11-66] */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTG|GPIO_PIN10)

#define GPIO_VDD_5V_PERIPH_OC   /* PE15[CN12-53] */ (GPIO_INPUT|GPIO_PULLDOWN|GPIO_PORTE|GPIO_PIN15)

/* Power switch controls ******************************************************/

#define POWER_SPEKTRUM(_s)      px4_arch_gpiowrite(GPIO_SPEKTRUM_PWR_EN, (1-_s))
#define SPEKTRUM_RX_AS_UART()   px4_arch_configgpio(GPIO_USART1_RX)

// FMUv4 has a separate GPIO for serial RC output
#define GPIO_RC_OUT			    /* PE5[CN11-50] */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTE|GPIO_PIN5)
#define SPEKTRUM_RX_AS_GPIO()   px4_arch_configgpio(GPIO_RC_OUT)
#define SPEKTRUM_RX_HIGH(_s)    px4_arch_gpiowrite(GPIO_RC_OUT, (_s))

#define SDIO_SLOTNO             0  /* Only one slot */
#define SDIO_MINOR              0

/* SD card bringup does not work if performed on the IDLE thread because it
 * will cause waiting.  Use either:
 *
 *  CONFIG_LIB_BOARDCTL=y, OR
 *  CONFIG_BOARD_INITIALIZE=y && CONFIG_BOARD_INITTHREAD=y
 */

#if defined(CONFIG_BOARD_INITIALIZE) && !defined(CONFIG_LIB_BOARDCTL) && \
   !defined(CONFIG_BOARD_INITTHREAD)
#  warning SDIO initialization cannot be perfomed on the IDLE thread
#endif

#define	BOARD_NAME "PX4NUCLEOF767ZI_V1"

/* By Providing BOARD_ADC_USB_CONNECTED (using the px4_arch abstraction)
 * this board support the ADC system_power interface, and therefore
 * provides the true logic GPIO BOARD_ADC_xxxx macros.
 */
#define BOARD_ADC_USB_CONNECTED (px4_arch_gpioread(GPIO_OTGFS_VBUS))
#define BOARD_ADC_BRICK_VALID   (px4_arch_gpioread(GPIO_VDD_BRICK_VALID))
#define BOARD_ADC_SERVO_VALID   (1)
#define BOARD_ADC_PERIPH_5V_OC  (px4_arch_gpioread(GPIO_VDD_5V_PERIPH_OC))
#define BOARD_ADC_HIPOWER_5V_OC (0)

#define BOARD_HAS_PWM	DIRECT_PWM_OUTPUT_CHANNELS

#define BOARD_FMU_GPIO_TAB { \
		{GPIO_GPIO0_INPUT,       GPIO_GPIO0_OUTPUT,       0}, \
		{GPIO_GPIO1_INPUT,       GPIO_GPIO1_OUTPUT,       0}, \
		{GPIO_GPIO2_INPUT,       GPIO_GPIO2_OUTPUT,       0}, \
		{GPIO_GPIO3_INPUT,       GPIO_GPIO3_OUTPUT,       0}, \
		{GPIO_GPIO4_INPUT,       GPIO_GPIO4_OUTPUT,       0}, \
		{GPIO_GPIO5_INPUT,       GPIO_GPIO5_OUTPUT,       0}, \
		{0,                      GPIO_VDD_3V3_SENSORS_EN, 0}, \
		{GPIO_VDD_BRICK_VALID,   0,                       0}, }

/*
 * GPIO numbers.
 *
 * There are no alternate functions on this board.
 */
#define GPIO_SERVO_1          (1<<0)  /**< servo 1 output */
#define GPIO_SERVO_2          (1<<1)  /**< servo 2 output */
#define GPIO_SERVO_3          (1<<2)  /**< servo 3 output */
#define GPIO_SERVO_4          (1<<3)  /**< servo 4 output */
#define GPIO_SERVO_5          (1<<4)  /**< servo 5 output */
#define GPIO_SERVO_6          (1<<5)  /**< servo 6 output */

#define GPIO_3V3_SENSORS_EN   (1<<6)  /**< PE3  - VDD_3V3_SENSORS_EN */
#define GPIO_BRICK_VALID      (1<<7)  /**< PB10 - !VDD_BRICK_VALID */

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

/************************************************************************************
 * Name: stm32_spi_bus_initialize
 *
 * Description:
 *   Called to configure SPI Buses.
 *
 ************************************************************************************/

extern int stm32_spi_bus_initialize(void);

/****************************************************************************************************
 * Name: board_spi_reset board_peripheral_reset
 *
 * Description:
 *   Called to reset SPI and the perferal bus
 *
 ****************************************************************************************************/

void board_spi_reset(int ms);
extern void board_peripheral_reset(int ms);

/****************************************************************************************************
 * Name: stm32_usbinitialize
 *
 * Description:
 *   Called to configure USB IO.
 *
 ****************************************************************************************************/

extern void stm32_usbinitialize(void);

#include "../common/board_common.h"

#endif /* __ASSEMBLY__ */

__END_DECLS
