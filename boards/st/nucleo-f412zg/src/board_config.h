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
 * PX4FMUv4 internal definitions
 */

#pragma once

/****************************************************************************************************
 * Included Files
 ****************************************************************************************************/

#include <px4_platform_common/px4_config.h>
#include <nuttx/compiler.h>
#include <stdint.h>

/****************************************************************************************************
 * Definitions
 ****************************************************************************************************/
/* Configuration ************************************************************************************/

#define FLASH_BASED_PARAMS

/* PX4FMU GPIOs ***********************************************************************************/
/* LEDs */

#define GPIO_LED_GREEN               GPIO_LD1
#define GPIO_LED_RED                 GPIO_LD2
#define GPIO_LED_BLUE                GPIO_LD3

#define BOARD_HAS_CONTROL_STATUS_LEDS 1
#define BOARD_OVERLOAD_LED     LED_RED
#define BOARD_ARMED_LED        LED_BLUE
#define BOARD_ARMED_STATE_LED  LED_GREEN

/**
 *  Define the Chip Selects for SPI1
 *  CS           Devices                                 DRDY
 *  ---- ----------------------------------------------- -----
 *  PC2  MPU9250                                         PD15
 *  PC15 ICM, ICM_20602, ICM_20608                       PC14
 *  PE15 HMC5983                                         PE12
 *  ---- ----------------------------------------------- -----
 */
#define BOARD_SPI_BUS_MAX_BUS_ITEMS 1

#define GPIO_SPI1_CS_PORTC_PIN2      (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTC|GPIO_PIN2)
#define GPIO_SPI1_CS_PORTC_PIN15     (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTC|GPIO_PIN15)
#define GPIO_SPI1_CS_PORTE_PIN15     (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTE|GPIO_PIN15)

/* Define the Data Ready interrupts On SPI 1. */
#define GPIO_DRDY_PORTD_PIN15        (GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI|GPIO_PORTD|GPIO_PIN15)
#define GPIO_DRDY_PORTC_PIN14        (GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI|GPIO_PORTC|GPIO_PIN14)
#define GPIO_DRDY_PORTE_PIN12        (GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI|GPIO_PORTE|GPIO_PIN12)


/* Define the Chip Selects for SPI2. */
#define GPIO_SPI2_CS_MS5611          (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTD|GPIO_PIN7)
#define GPIO_SPI2_CS_FRAM            (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTD|GPIO_PIN10)


/**
 * Define the ability to shut off off the sensor signals
 * by changing the signals to inputs.
 */
#define _PIN_OFF(def) (((def) & (GPIO_PORT_MASK | GPIO_PIN_MASK)) | (GPIO_INPUT|GPIO_PULLDOWN|GPIO_SPEED_2MHz))

/* SPI 1 bus off. */
#define GPIO_SPI1_SCK_OFF            _PIN_OFF(GPIO_SPI1_SCK)
#define GPIO_SPI1_MISO_OFF           _PIN_OFF(GPIO_SPI1_MISO)
#define GPIO_SPI1_MOSI_OFF           _PIN_OFF(GPIO_SPI1_MOSI)

/* SPI 1 CS's  off. */
#define GPIO_SPI1_CS_OFF_PORTC_PIN2  _PIN_OFF(GPIO_SPI1_CS_PORTC_PIN2)
#define GPIO_SPI1_CS_OFF_PORTC_PIN15 _PIN_OFF(GPIO_SPI1_CS_PORTC_PIN15)
#define GPIO_SPI1_CS_OFF_PORTE_PIN15 _PIN_OFF(GPIO_SPI1_CS_PORTE_PIN15)

/* SPI 1 DRDY's off. */
#define GPIO_DRDY_OFF_PORTD_PIN15    _PIN_OFF(GPIO_DRDY_PORTD_PIN15)
#define GPIO_DRDY_OFF_PORTC_PIN14    _PIN_OFF(GPIO_DRDY_PORTC_PIN14)
#define GPIO_DRDY_OFF_PORTE_PIN12    _PIN_OFF(GPIO_DRDY_PORTE_PIN12)

/**
 * N.B we do not have control over the SPI 2 buss powered devices
 * so the the ms5611 is not resetable.
 *
 */

#define PX4_SPI_BUS_SENSORS          1
#define PX4_SPI_BUS_RAMTRON          2
#define PX4_SPI_BUS_BARO             PX4_SPI_BUS_RAMTRON

#ifdef CONFIG_STM32_SPI4
#  define PX4_SPI_BUS_EXTERNAL       4
/* The mask passes to init the SPI bus pins
 * N.B This works ONLY with buss numbers that are powers of 2
 * Adding SPI3 would break this!
 */
#  define   SPI_BUS_INIT_MASK_EXT     PX4_SPI_BUS_EXTERNAL
#endif /* CONFIG_STM32_SPI4 */

#include <drivers/drv_sensor.h>

/* Use these in place of the uint32_t enumeration to select a specific SPI device on SPI1 */
#define PX4_SPIDEV_MPU               PX4_MK_SPI_SEL(0, DRV_IMU_DEVTYPE_MPU6000)
#define PX4_SPIDEV_HMC               PX4_MK_SPI_SEL(0, DRV_MAG_DEVTYPE_HMC5883)
#define PX4_SPIDEV_LIS               PX4_MK_SPI_SEL(0, DRV_MAG_DEVTYPE_LIS3MDL)
#define PX4_SPIDEV_ICM_20608         PX4_MK_SPI_SEL(0, DRV_IMU_DEVTYPE_ICM20608)
#define PX4_SPIDEV_ICM_20602         PX4_MK_SPI_SEL(0, DRV_IMU_DEVTYPE_ICM20602)
#define PX4_SPIDEV_MPU2              PX4_MK_SPI_SEL(0, DRV_IMU_DEVTYPE_MPU9250)

/**
 * Onboard MS5611 and FRAM are both on bus SPI2.
 * spi_dev_e:SPIDEV_FLASH has the value 2 and is used in the NuttX ramtron driver.
 * PX4_MK_SPI_SEL  differentiate by adding in PX4_SPI_DEVICE_ID.
 */
#define PX4_SPIDEV_BARO             PX4_MK_SPI_SEL(0, DRV_BARO_DEVTYPE_MS5611)

#ifdef CONFIG_STM32_SPI4
#  define PX4_SPIDEV_EXTERNAL       PX4_MK_SPI_SEL(0, 0)
#endif /* CONFIG_STM32_SPI4 */

/* I2C busses. */
#define PX4_I2C_BUS_EXPANSION        1
#define PX4_I2C_BUS_LED              PX4_I2C_BUS_EXPANSION

/**
 * ADC channels:
 * These are the channel numbers of the ADCs of the microcontroller that can be used by the Px4 Firmware in the adc driver.
 */
#define ADC_CHANNELS (1 << 2) | (1 << 3) | (1 << 4) | (1 << 10) | (1 << 11) | (1 << 12) | (1 << 13) | (1 << 14)

/* ADC defines to be used in sensors.cpp to read from a particular channel. */
#define ADC_BATTERY_VOLTAGE_CHANNEL  2
#define ADC_BATTERY_CURRENT_CHANNEL  3
#define ADC_5V_RAIL_SENSE            4
#define ADC_RC_RSSI_CHANNEL          11


/* Power supply control and monitoring GPIOs. */
#define GPIO_VDD_BRICK_VALID         (GPIO_INPUT|GPIO_PULLUP|GPIO_PORTB|GPIO_PIN5)
#define GPIO_VDD_USB_VALID           (GPIO_INPUT|GPIO_PULLUP|GPIO_PORTC|GPIO_PIN0)



/**
 * PWM:
 *
 * Six PWM outputs are configured.
 */
#define DIRECT_PWM_OUTPUT_CHANNELS   6
#define DIRECT_INPUT_TIMER_CHANNELS  6

/**
 * USB OTG FS:
 * PA9  OTG_FS_VBUS VBUS sensing.
 */
#define GPIO_OTGFS_VBUS              (GPIO_INPUT|GPIO_FLOAT|GPIO_SPEED_100MHz|GPIO_OPENDRAIN|GPIO_PORTA|GPIO_PIN9)

/* High-resolution timer */
#define HRT_TIMER                    3  /* use timer 3 for the HRT */
#define HRT_TIMER_CHANNEL            4  /* use capture/compare channel 4 */


#define GPIO_PERIPH_3V3_EN           (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTC|GPIO_PIN5)


/* This board provides a DMA pool and APIs. */
#define BOARD_DMA_ALLOC_POOL_SIZE (5120 + 512 + 1024)	// 5120 fat + 512 + 1024 spi

#define BOARD_HAS_ON_RESET 1


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

extern void stm32_usbinitialize(void);

extern void board_peripheral_reset(int ms);

#include <px4_platform_common/board_common.h>

#endif /* __ASSEMBLY__ */

__END_DECLS
