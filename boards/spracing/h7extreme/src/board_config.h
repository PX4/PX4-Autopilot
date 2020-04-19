/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
 *   Authors: Igor Misic <igy1000mb@gmail.com>
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
 * sp racing h7 extreme internal definitions
 */

#pragma once

/****************************************************************************************************
 * Included Files
 ****************************************************************************************************/

#include <px4_platform_common/px4_config.h>
#include <nuttx/compiler.h>
#include <stdint.h>

#include <stm32_gpio.h>

/****************************************************************************************************
 * Definitions
 ****************************************************************************************************/

#define FLASH_BASED_PARAMS
#define FLASH_BASED_PARAMS_EXT

/* PX4IO connection configuration */

#define BOARD_USES_PX4IO_VERSION       2
#define PX4IO_SERIAL_DEVICE            "/dev/ttyS6"
#define PX4IO_SERIAL_TX_GPIO           GPIO_UART8_TX
#define PX4IO_SERIAL_RX_GPIO           GPIO_UART8_RX
#define PX4IO_SERIAL_BASE              STM32_UART8_BASE
#define PX4IO_SERIAL_VECTOR            STM32_IRQ_UART8
#define PX4IO_SERIAL_TX_DMAMAP         DMAMAP_UART8_TX
#define PX4IO_SERIAL_RX_DMAMAP         DMAMAP_UART8_RX
#define PX4IO_SERIAL_RCC_REG           STM32_RCC_APB1LENR
#define PX4IO_SERIAL_RCC_EN            RCC_APB1LENR_UART8EN
#define PX4IO_SERIAL_CLOCK             STM32_PCLK1_FREQUENCY
#define PX4IO_SERIAL_BITRATE           1500000               /* 1.5Mbps -> max rate for IO */

#define BOARD_HAS_USB_VALID            1 // LTC Has No USB valid

/* LEDs are driven with push open drain to support Anode to 5V or 3.3V */

#define GPIO_nLED_RED        /* PE3 */  (GPIO_OUTPUT|GPIO_OPENDRAIN|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTE|GPIO_PIN3)

#define BOARD_HAS_CONTROL_STATUS_LEDS      1
#define BOARD_ARMED_STATE_LED  LED_RED
#define BOARD_OVERLOAD_LED     LED_RED

/*  Define the Chip Selects, Data Ready and Control signals per SPI bus */

/* SPI2. */
#if 0 //Some problem here. Gyro return 0x1B instead 0x12 for whoami
#define PX4_SPI_BUS_SENSORS		2
#define PX4_SPIDEV_ICM_20602			PX4_MK_SPI_SEL(PX4_SPI_BUS_SENSORS, 0)
#endif
#define GPIO_SPI2_CS_GYRO_ICM20602_2	(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTB|GPIO_PIN12)
#define PX4_SENSORS1_BUS_CS_GPIO		{GPIO_SPI2_CS_GYRO_ICM20602_2}
#define GPIO_SPI2_DRDY1_ICM20602		(GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI|GPIO_PORTE|GPIO_PIN15)


/* SPI3. */
#define PX4_SPI_BUS_SENSORS1	3
#define PX4_SPIDEV_ICM_20602			PX4_MK_SPI_SEL(PX4_SPI_BUS_SENSORS1, 0)
#define GPIO_SPI3_CS_GYRO_ICM20602_1	(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTA|GPIO_PIN15)
#define PX4_SENSORS2_BUS_CS_GPIO		{GPIO_SPI3_CS_GYRO_ICM20602_1}
#define GPIO_SPI3_DRDY1_ICM20602		(GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI|GPIO_PORTD|GPIO_PIN4)

/* SPI4. */
#define PX4_SPI_BUS_OSD			4
#define GPIO_SPI4_CS_OSD_MAX7456		(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTE|GPIO_PIN11)
#define PX4_OSD_BUS_CS_GPIO				{GPIO_SPI4_CS_OSD_MAX7456}
#define PX4_SPIDEV_OSD			1
/*
 *  Define the ability to shut off off the sensor signals
 *  by changing the signals to inputs
 */

#define _PIN_OFF(def) (((def) & (GPIO_PORT_MASK | GPIO_PIN_MASK)) | (GPIO_INPUT|GPIO_PULLDOWN|GPIO_SPEED_2MHz))



/* I2C busses */

#define PX4_I2C_BUS_ONBOARD         1
#define PX4_I2C_BUS_EXPANSION       2
#define PX4_I2C_BUS_LED             PX4_I2C_BUS_EXPANSION

#define BOARD_NUMBER_I2C_BUSES      2
#define BOARD_I2C_BUS_CLOCK_INIT    {100000, 100000}

#define PX4_I2C_OBDEV_BMP388		0x76

/*
 * ADC channels
 *
 * These are the channel numbers of the ADCs of the microcontroller that
 * can be used by the Px4 Firmware in the adc driver
 */

/* ADC defines to be used in sensors.cpp to read from a particular channel */

#define ADC1_CH(n)                  (n)

/* Define GPIO pins used as ADC N.B. Channel numbers must match below */
#define PX4_ADC_GPIO  \
	/* PC4 */  GPIO_ADC12_INP4,  \
	/* PC1 */  GPIO_ADC123_INP11, \
	/* PC0 */  GPIO_ADC123_INP10


/* Define Channel numbers must match above GPIO pin IN(n)*/
#define ADC_RSSI_IN_CHANNEL                 /* PC4 */  ADC1_CH(4)
#define ADC_BATTERY_VOLTAGE_CHANNEL         /* PC1 */  ADC1_CH(11)
#define ADC_BATTERY_CURRENT_CHANNEL         /* PC0 */  ADC1_CH(10)

#define ADC_CHANNELS \
	((1 << ADC_BATTERY_VOLTAGE_CHANNEL)       | \
	 (1 << ADC_BATTERY_CURRENT_CHANNEL)       | \
	 (1 << ADC_RSSI_IN_CHANNEL))

/* Define Battery 1 Voltage Divider and A per V
 */
#define BOARD_BATTERY1_V_DIV         (10.9f)
#define BOARD_BATTERY1_A_PER_V       (17.f)

/* PWM
 */
#define DIRECT_PWM_OUTPUT_CHANNELS  10
#define DIRECT_INPUT_TIMER_CHANNELS  6
#define GPIO_nPOWER_IN_C                /* PG3  */ (GPIO_INPUT|GPIO_PULLUP|GPIO_PORTG|GPIO_PIN3)
#define GPIO_nVDD_USB_VALID             GPIO_nPOWER_IN_C /* USB     Is Chosen */


/* Tone alarm output */
#define GPIO_TONE_ALARM_IDLE    /* PE5 */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTE|GPIO_PIN5)
#define GPIO_TONE_ALARM_GPIO    /* PE5 */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTE|GPIO_PIN5)

/* USB OTG FS
 *
 * PA8  OTG_FS_VBUS VBUS sensing
 */
#define GPIO_OTGFS_VBUS         /* PA9 */ (GPIO_INPUT|GPIO_PULLDOWN|GPIO_SPEED_100MHz|GPIO_PORTA|GPIO_PIN9)

/* High-resolution timer */
#define HRT_TIMER               8  /* use timer8 for the HRT */
#define HRT_TIMER_CHANNEL       3  /* use capture/compare channel 3 */

/* RC Serial port */

#define RC_SERIAL_PORT                     "/dev/ttyS4"

#define GPIO_RSSI_IN                       /* PC5  */ (GPIO_INPUT|GPIO_PULLUP|GPIO_PORTC|GPIO_PIN5)

#define SDIO_SLOTNO                    0  /* Only one slot */
#define SDIO_MINOR                     0


/* By Providing BOARD_ADC_USB_CONNECTED (using the px4_arch abstraction)
 * this board support the ADC system_power interface, and therefore
 * provides the true logic GPIO BOARD_ADC_xxxx macros.
 */
#define BOARD_ADC_USB_CONNECTED (px4_arch_gpioread(GPIO_OTGFS_VBUS))
#define BOARD_ADC_USB_VALID     (!px4_arch_gpioread(GPIO_nVDD_USB_VALID))

#define BOARD_ADC_SERVO_VALID     (1)
#define BOARD_HAS_PWM  DIRECT_PWM_OUTPUT_CHANNELS

/* This board provides a DMA pool and APIs */
#define BOARD_DMA_ALLOC_POOL_SIZE 5120

/* This board provides the board_on_reset interface */
#define BOARD_HAS_ON_RESET 1

#define PX4_GPIO_INIT_LIST { \
		PX4_ADC_GPIO,                     \
		GPIO_nPOWER_IN_C,                 \
		GPIO_TONE_ALARM_IDLE,             \
		GPIO_RSSI_IN,                \
	}

#define BOARD_ENABLE_CONSOLE_BUFFER

#define BOARD_NUM_IO_TIMERS 3

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
 *   Called to configure SPI chip select GPIO pins for the board.
 *
 ****************************************************************************************************/

extern void stm32_spiinitialize(void);

void board_spi_reset(int ms);

extern void stm32_usbinitialize(void);

extern void board_peripheral_reset(int ms);

#include <px4_platform_common/board_common.h>

/****************************************************************************************************
 * Progmem external flash functions:
 *
 * This board has custom progmem functions. It is used to save data to external flash.
 *
 ****************************************************************************************************/
void flash_w25q128_init(void);

#include <sys/types.h>
__ramfunc__ ssize_t up_progmem_ext_getpage(size_t addr);
__ramfunc__ ssize_t up_progmem_ext_eraseblock(size_t block);
__ramfunc__ ssize_t up_progmem_ext_write(size_t addr, FAR const void *buf, size_t count);

#endif /* __ASSEMBLY__ */

__END_DECLS
