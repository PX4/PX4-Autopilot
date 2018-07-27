/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
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
 * av_x-v1 internal definitions
 */

#pragma once

/****************************************************************************************************
 * Included Files
 ****************************************************************************************************/

#include <px4_config.h>
#include <nuttx/compiler.h>
#include <stdint.h>

#include <stm32_gpio.h>

/* Configuration ************************************************************************************/

#define BOARD_HAS_NBAT_V              1 // Only one Vbat to ADC
#define BOARD_HAS_NBAT_I              0 // No Ibat ADC

/* SENSORS are on SPI1, 5, 6
 * MEMORY is on bus SPI2
 * MS5611 is on bus SPI4
 */

#define PX4_SPI_BUS_SENSOR1  1
#define PX4_SPI_BUS_SENSOR2  2
#define PX4_SPI_BUS_SENSOR4  4
#define PX4_SPI_BUS_SENSOR5  5

/*  Define the Chip Selects, Data Ready and Control signals per SPI bus */

/* SPI 1 CS */
#define GPIO_SPI1_CS1_ADIS16477    /* PG10 */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTG|GPIO_PIN10)
#define GPIO_SPI1_RESET_ADIS16477  /* PB15 */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTB|GPIO_PIN15)

/* SPI 2 CS */
#define GPIO_SPI2_CS1_ADIS16497    /* PI0 */  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTI|GPIO_PIN10)

/* SPI 4 CS */
#define GPIO_SPI4_CS1_LPS22HB      /* PE4 */  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTE|GPIO_PIN4)

/* SPI 5 CS */
#define GPIO_SPI5_CS1_LSM303A_M    /* PH5 */  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTH|GPIO_PIN5)
#define GPIO_SPI5_CS1_LSM303A_X    /* PB0 */  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTB|GPIO_PIN0)

/*  Define the SPI1 Data Ready interrupts */
#define GPIO_SPI1_DRDY1_ADIS16477  /* PJ0 */   (GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI|GPIO_PORTJ|GPIO_PIN0)

/*  Define the SPI2 Data Ready interrupts */
#define GPIO_SPI2_DRDY1_ADIS16497  /* PJ5 */   (GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI|GPIO_PORTJ|GPIO_PIN5)

/*  Define the SPI4 Data Ready interrupts */
#define GPIO_SPI4_DRDY1_LPS22HB    /* PK1 */   (GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI|GPIO_PORTK|GPIO_PIN1)

/*  Define the SPI5 Data Ready interrupts */
#define GPIO_SPI5_DRDY1_LSM303A_M  /* PK7 */   (GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI|GPIO_PORTK|GPIO_PIN7)
#define GPIO_SPI5_DRDY2_LSM303A_X  /* PD12 */  (GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI|GPIO_PORTD|GPIO_PIN12)

/* SPI1 off */
#define GPIO_SPI1_SCK_OFF	_PIN_OFF(GPIO_SPI1_SCK)
#define GPIO_SPI1_MISO_OFF	_PIN_OFF(GPIO_SPI1_MISO)
#define GPIO_SPI1_MOSI_OFF	_PIN_OFF(GPIO_SPI1_MOSI)

/* SPI2 off */
#define GPIO_SPI2_SCK_OFF	_PIN_OFF(GPIO_SPI2_SCK)
#define GPIO_SPI2_MISO_OFF	_PIN_OFF(GPIO_SPI2_MISO)
#define GPIO_SPI2_MOSI_OFF	_PIN_OFF(GPIO_SPI2_MOSI)

/* SPI4 off */
#define GPIO_SPI4_SCK_OFF	_PIN_OFF(GPIO_SPI4_SCK)
#define GPIO_SPI4_MISO_OFF	_PIN_OFF(GPIO_SPI4_MISO)
#define GPIO_SPI4_MOSI_OFF	_PIN_OFF(GPIO_SPI4_MOSI)

/* SPI5 off */
#define GPIO_SPI5_SCK_OFF	_PIN_OFF(GPIO_SPI5_SCK)
#define GPIO_SPI5_MISO_OFF	_PIN_OFF(GPIO_SPI5_MISO)
#define GPIO_SPI5_MOSI_OFF	_PIN_OFF(GPIO_SPI5_MOSI)


#define GPIO_DRDY_OFF_SPI1_DRDY1_ADIS16477    _PIN_OFF(GPIO_SPI1_DRDY1_ADIS16477)
#define GPIO_DRDY_OFF_SPI2_DRDY1_ADIS16497    _PIN_OFF(GPIO_SPI2_DRDY1_ADIS16497)
#define GPIO_DRDY_OFF_SPI4_DRDY1_LPS22HB      _PIN_OFF(GPIO_SPI4_DRDY1_LPS22HB)
#define GPIO_DRDY_OFF_SPI5_DRDY1_LSM303A_M    _PIN_OFF(GPIO_SPI5_DRDY1_LSM303A_M)
#define GPIO_DRDY_OFF_SPI5_DRDY2_LSM303A_X    _PIN_OFF(GPIO_SPI5_DRDY1_LSM303A_X)

/*
 *  Define the ability to shut off off the sensor signals
 *  by changing the signals to inputs
 */
#define _PIN_OFF(def) (((def) & (GPIO_PORT_MASK | GPIO_PIN_MASK)) | (GPIO_INPUT|GPIO_PULLDOWN|GPIO_SPEED_2MHz))


/* SPI1 */
#define PX4_SPIDEV_ADIS16477       PX4_MK_SPI_SEL(PX4_SPI_BUS_SENSOR1,0)
#define PX4_SENSOR1_BUS_CS_GPIO    {GPIO_SPI1_CS1_ADIS16477}
#define PX4_SENSOR1_BUS_FIRST_CS   PX4_SPIDEV_ADIS16477
#define PX4_SENSOR1_BUS_LAST_CS    PX4_SPIDEV_ADIS16477

/* SPI2 */
#define PX4_SPIDEV_ADIS16497       PX4_MK_SPI_SEL(PX4_SPI_BUS_SENSOR2,0)
#define PX4_SENSOR2_BUS_CS_GPIO    {GPIO_SPI2_CS1_ADIS16497}
#define PX4_SENSOR2_BUS_FIRST_CS   PX4_SPIDEV_ADIS16497
#define PX4_SENSOR2_BUS_LAST_CS    PX4_SPIDEV_ADIS16497

/* SPI4 */
#define PX4_SPIDEV_LPS22HB         PX4_MK_SPI_SEL(PX4_SPI_BUS_SENSOR4,0)
#define PX4_SENSOR4_BUS_CS_GPIO    {GPIO_SPI4_CS1_LPS22HB}
#define PX4_SENSOR4_BUS_FIRST_CS   PX4_SPIDEV_LPS22HB
#define PX4_SENSOR4_BUS_LAST_CS    PX4_SPIDEV_LPS22HB

/* SPI5 */
#define PX4_SPIDEV_LSM303A_M       PX4_MK_SPI_SEL(PX4_SPI_BUS_SENSOR5,0)
#define PX4_SPIDEV_LSM303A_X       PX4_MK_SPI_SEL(PX4_SPI_BUS_SENSOR5,1)
#define PX4_SENSOR5_BUS_CS_GPIO    {GPIO_SPI5_CS1_LSM303A_M, GPIO_SPI5_CS1_LSM303A_X}
#define PX4_SENSOR5_BUS_FIRST_CS   PX4_SPIDEV_LSM303A_M
#define PX4_SENSOR5_BUS_LAST_CS    PX4_SPIDEV_LSM303A_X

/* I2C busses */
#define PX4_I2C_BUS_EXPANSION	2
#define PX4_I2C_BUS_EXPANSION1	4
#define PX4_I2C_BUS_ONBOARD	3

#define BOARD_NUMBER_I2C_BUSES  4
#define BOARD_I2C_BUS_CLOCK_INIT {100000, 100000, 100000, 100000}

/*
 * ADC channels
 *
 * These are the channel numbers of the ADCs of the microcontroller that
 * can be used by the Px4 Firmware in the adc driver
 */

/* ADC defines to be used in sensors.cpp to read from a particular channel */

#define ADC1_CH(n)                      (n)
#define ADC1_GPIO(n)                    GPIO_ADC1_IN##n

/* Define GPIO pins used as ADC N.B. Channel numbers must match below */

#define PX4_ADC_GPIO  \
	/* PA0 */  ADC1_GPIO(0),  \
	/* PA1 */  ADC1_GPIO(1),  \
	/* PA2 */  ADC1_GPIO(2),  \
	/* PA3 */  ADC1_GPIO(3),  \
	/* PA4 */  ADC1_GPIO(4),  \
	/* PB8 */  ADC1_GPIO(8),  \
	/* PC0 */  ADC1_GPIO(10), \
	/* PC1 */  ADC1_GPIO(11), \
	/* PC2 */  ADC1_GPIO(12), \
	/* PC3 */  ADC1_GPIO(13), \
	/* PC4 */  ADC1_GPIO(14)

/* Define Channel numbers must match above GPIO pin IN(n)*/

#define ADC_BATTERY1_VOLTAGE_CHANNEL        /* PA0 */  ADC1_CH(0)
#define ADC_BATTERY1_CURRENT_CHANNEL        /* PA1 */  ADC1_CH(1)
#define ADC1_SPARE_2_CHANNEL                /* PA4 */  ADC1_CH(4)
#define ADC_RSSI_IN_CHANNEL                 /* PB0 */  ADC1_CH(8)
#define ADC_SCALED_V5_CHANNEL               /* PC0 */  ADC1_CH(10)
#define ADC_SCALED_VDD_3V3_SENSORS_CHANNEL  /* PC1 */  ADC1_CH(11)
#define ADC_HW_VER_SENSE_CHANNEL            /* PC2 */  ADC1_CH(12)
#define ADC_HW_REV_SENSE_CHANNEL            /* PC3 */  ADC1_CH(13)
#define ADC1_SPARE_1_CHANNEL                /* PC4 */  ADC1_CH(14)

#if BOARD_HAS_NBAT_V == 2 && BOARD_HAS_NBAT_I == 2
#define ADC_CHANNELS \
	((1 << ADC_BATTERY1_VOLTAGE_CHANNEL)       | \
	 (1 << ADC_BATTERY1_CURRENT_CHANNEL)       | \
	 (1 << ADC_BATTERY2_VOLTAGE_CHANNEL)       | \
	 (1 << ADC_BATTERY2_CURRENT_CHANNEL)       | \
	 (1 << ADC1_SPARE_2_CHANNEL)               | \
	 (1 << ADC_RSSI_IN_CHANNEL)                | \
	 (1 << ADC_SCALED_V5_CHANNEL)              | \
	 (1 << ADC_SCALED_VDD_3V3_SENSORS_CHANNEL) | \
	 (1 << ADC_HW_VER_SENSE_CHANNEL)           | \
	 (1 << ADC_HW_REV_SENSE_CHANNEL)           | \
	 (1 << ADC1_SPARE_1_CHANNEL))
#elif BOARD_HAS_NBAT_V == 1 && BOARD_HAS_NBAT_I == 1
#define ADC_CHANNELS \
	((1 << ADC_BATTERY1_VOLTAGE_CHANNEL)       | \
	 (1 << ADC_BATTERY1_CURRENT_CHANNEL)       | \
	 (1 << ADC1_SPARE_2_CHANNEL)               | \
	 (1 << ADC_RSSI_IN_CHANNEL)                | \
	 (1 << ADC_SCALED_V5_CHANNEL)              | \
	 (1 << ADC_SCALED_VDD_3V3_SENSORS_CHANNEL) | \
	 (1 << ADC_HW_VER_SENSE_CHANNEL)           | \
	 (1 << ADC_HW_REV_SENSE_CHANNEL)           | \
	 (1 << ADC1_SPARE_1_CHANNEL))
#elif BOARD_HAS_NBAT_V == 1 && BOARD_HAS_NBAT_I == 0
#define ADC_CHANNELS \
	((1 << ADC_BATTERY1_VOLTAGE_CHANNEL)       | \
	 (1 << ADC1_SPARE_2_CHANNEL)               | \
	 (1 << ADC_RSSI_IN_CHANNEL)                | \
	 (1 << ADC_SCALED_V5_CHANNEL)              | \
	 (1 << ADC_SCALED_VDD_3V3_SENSORS_CHANNEL) | \
	 (1 << ADC_HW_VER_SENSE_CHANNEL)           | \
	 (1 << ADC_HW_REV_SENSE_CHANNEL)           | \
	 (1 << ADC1_SPARE_1_CHANNEL))
#endif

/* Define Battery 1 Voltage Divider and A per V
 */

#define BOARD_BATTERY1_V_DIV         (10.133333333f)
#define BOARD_BATTERY1_A_PER_V       (36.367515152f)

/* HW has to large of R termination on ADC todo:change when HW value is chosen */

#define BOARD_ADC_OPEN_CIRCUIT_V               (5.6f)

/* PWM
 *
 * 8  PWM outputs are configured.
 *
 * Pins:
 *
 * FMU_CH1 : PA8  : TIM1_CH1
 * FMU_CH2 : PA9  : TIM1_CH2
 * FMU_CH3 : PA10 : TIM1_CH3
 * FMU_CH4 : PA11 : TIM1_CH4
 * FMU_CH5 : PA3  : TIM2_CH4
 * FMU_CH6 : PI6  : TIM8_CH2
 * FMU_CH7 : PF7  : TIM11_CH1
 * FMU_CH8 : PF6  : TIM10_CH1
 *
 */
#define GPIO_TIM1_CH1OUT      /* PA8   T1C1   FMU1 */ GPIO_TIM1_CH1OUT_1
#define GPIO_TIM1_CH2OUT      /* PA9   T1C2   FMU2 */ GPIO_TIM1_CH2OUT_1
#define GPIO_TIM1_CH3OUT      /* PA10  T1C3   FMU3 */ GPIO_TIM1_CH3OUT_1
#define GPIO_TIM1_CH4OUT      /* PA11  T1C4   FMU4 */ GPIO_TIM1_CH4OUT_1
#define GPIO_TIM2_CH4OUT      /* PA3   T2C4   FMU5 */ GPIO_TIM2_CH4OUT_1
#define GPIO_TIM8_CH2OUT      /* PI6   T8C2   FMU6 */ GPIO_TIM8_CH2IN_2
#define GPIO_TIM11_CH1OUT     /* PF7   T11C1  FMU7 */ GPIO_TIM11_CH1OUT_2
#define GPIO_TIM10_CH1OUT     /* PF6   T10C1  FMU8 */ GPIO_TIM10_CH1OUT_2

#define DIRECT_PWM_OUTPUT_CHANNELS  8

#define GPIO_TIM1_CH1IN      /* PA8   T1C1   FMU1 */ GPIO_TIM1_CH1IN_1
#define GPIO_TIM1_CH2IN      /* PA9   T1C2   FMU2 */ GPIO_TIM1_CH2IN_1
#define GPIO_TIM1_CH3IN      /* PA10  T1C3   FMU3 */ GPIO_TIM1_CH3IN_1
#define GPIO_TIM1_CH4IN      /* PA11  T1C4   FMU4 */ GPIO_TIM1_CH4IN_1
#define GPIO_TIM2_CH4IN      /* PA3   T2C4   FMU5 */ GPIO_TIM2_CH4IN_1
#define GPIO_TIM8_CH2IN      /* PI6   T8C2   FMU6 */ GPIO_TIM8_CH2IN_2
//#define GPIO_TIM11_CH1IN     /* PF7   T11C1  FMU7 */ GPIO_TIM11_CH1IN_2
//#define GPIO_TIM10_CH1IN     /* PF6   T10C1  FMU8 */ GPIO_TIM10_CH1IN_2

#define DIRECT_INPUT_TIMER_CHANNELS  8

/* User GPIOs
 *
 * GPIO0-5 are the PWM servo outputs.
 */

#define _MK_GPIO_INPUT(def) (((def) & (GPIO_PORT_MASK | GPIO_PIN_MASK)) | (GPIO_INPUT|GPIO_PULLUP))

#define GPIO_GPIO0_INPUT        /* PA8  T1C1   FMU1 */ _MK_GPIO_INPUT(GPIO_TIM1_CH1IN)
#define GPIO_GPIO1_INPUT        /* PA9  T1C2   FMU2 */ _MK_GPIO_INPUT(GPIO_TIM1_CH2IN)
#define GPIO_GPIO2_INPUT        /* PA10 T1C3   FMU3 */ _MK_GPIO_INPUT(GPIO_TIM1_CH3IN)
#define GPIO_GPIO3_INPUT        /* PA11 T1C4   FMU4 */ _MK_GPIO_INPUT(GPIO_TIM1_CH4IN)
#define GPIO_GPIO4_INPUT        /* PA3  T2C4   FMU5 */ _MK_GPIO_INPUT(GPIO_TIM2_CH4IN_1)
#define GPIO_GPIO5_INPUT        /* PI6  T8C2   FMU6 */ _MK_GPIO_INPUT(GPIO_TIM8_CH2IN_2)
#define GPIO_GPIO6_INPUT        /* PF7  T11C1  FMU7 */ _MK_GPIO_INPUT(GPIO_TIM11_CH1IN_2)
#define GPIO_GPIO7_INPUT        /* PF6  T10C1  FMU8 */ _MK_GPIO_INPUT(GPIO_TIM10_CH1IN_2)

#define _MK_GPIO_OUTPUT(def) (((def) & (GPIO_PORT_MASK | GPIO_PIN_MASK)) | (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR))

#define GPIO_GPIO0_OUTPUT        /* PA8  T1C1   FMU1 */ _MK_GPIO_OUTPUT(GPIO_TIM1_CH1OUT)
#define GPIO_GPIO1_OUTPUT        /* PA9  T1C2   FMU2 */ _MK_GPIO_OUTPUT(GPIO_TIM1_CH2OUT)
#define GPIO_GPIO2_OUTPUT        /* PA10 T1C3   FMU3 */ _MK_GPIO_OUTPUT(GPIO_TIM1_CH3OUT)
#define GPIO_GPIO3_OUTPUT        /* PA11 T1C4   FMU4 */ _MK_GPIO_OUTPUT(GPIO_TIM1_CH4OUT)
#define GPIO_GPIO4_OUTPUT        /* PA3  T2C4   FMU5 */ _MK_GPIO_OUTPUT(GPIO_TIM2_CH4OUT_1)
#define GPIO_GPIO5_OUTPUT        /* PI6  T8C2   FMU6 */ _MK_GPIO_OUTPUT(GPIO_TIM8_CH2OUT_2)
#define GPIO_GPIO6_OUTPUT        /* PF7  T11C1  FMU7 */ _MK_GPIO_OUTPUT(GPIO_TIM11_CH1OUT_2)
#define GPIO_GPIO7_OUTPUT        /* PF6  T10C1  FMU8 */ _MK_GPIO_OUTPUT(GPIO_TIM10_CH1OUT_2)

/* High-resolution timer */
#define HRT_TIMER		     5  /* use timer5 for the HRT */
#define HRT_TIMER_CHANNEL    1  /* use capture/compare channel 3 */

#define RC_UXART_BASE                      STM32_UART5_BASE
#define RC_SERIAL_PORT                     "/dev/ttyS4"
#define BOARD_HAS_SINGLE_WIRE              1 /* HW is capable of Single Wire */
#define BOARD_HAS_SINGLE_WIRE_ON_TX        1 /* HW default is wired as Single Wire On TX pin */
#define BOARD_HAS_RX_TX_SWAP               1 /* HW Can swap TX and RX */
#define RC_SERIAL_PORT_IS_SWAPED           0 /* Board wired with RC's TX is on cpu RX */

#define GPS_DEFAULT_UART_PORT "/dev/ttyS5" /* UART7 */

/* Power switch controls ******************************************************/

#define SDIO_SLOTNO                    0  /* Only one slot */
#define SDIO_MINOR                     0

/* SD card bringup does not work if performed on the IDLE thread because it
 * will cause waiting.  Use either:
 *
 *  CONFIG_LIB_BOARDCTL=y, OR
 *  CONFIG_BOARD_INITIALIZE=y && CONFIG_BOARD_INITTHREAD=y
 */

#if defined(CONFIG_BOARD_INITIALIZE) && !defined(CONFIG_LIB_BOARDCTL) && !defined(CONFIG_BOARD_INITTHREAD)
#  warning SDIO initialization cannot be perfomed on the IDLE thread
#endif

#define	BOARD_NAME "AV_X_V1"

/* AV-X_V1 never powers off the Servo rail */

#define BOARD_ADC_SERVO_VALID     (1)

#define ADC_BATTERY_VOLTAGE_CHANNEL  0
#define ADC_BATTERY_CURRENT_CHANNEL  1 // TODO: review

#define BOARD_ADC_PERIPH_5V_OC  (!px4_arch_gpioread(GPIO_nVDD_5V_PERIPH_OC))
#define BOARD_ADC_HIPOWER_5V_OC (!px4_arch_gpioread(GPIO_nVDD_5V_HIPOWER_OC))

#define BOARD_HAS_PWM	DIRECT_PWM_OUTPUT_CHANNELS

#define BOARD_FMU_GPIO_TAB { \
		{GPIO_GPIO0_INPUT,       GPIO_GPIO0_OUTPUT,              0}, \
		{GPIO_GPIO1_INPUT,       GPIO_GPIO1_OUTPUT,              0}, \
		{GPIO_GPIO2_INPUT,       GPIO_GPIO2_OUTPUT,              0}, \
		{GPIO_GPIO3_INPUT,       GPIO_GPIO3_OUTPUT,              0}, \
		{GPIO_GPIO4_INPUT,       GPIO_GPIO4_OUTPUT,              0}, \
		{GPIO_GPIO5_INPUT,       GPIO_GPIO5_OUTPUT,              0}, \
		{GPIO_GPIO6_INPUT,       GPIO_GPIO6_OUTPUT,              0}, \
		{GPIO_GPIO7_INPUT,       GPIO_GPIO7_OUTPUT,              0}, \
	}

/*
 * GPIO numbers.
 *
 * There are no alternate functions on this board.
 */
#define GPIO_SERVO_1                (1<<0)   /**< servo 1 output */
#define GPIO_SERVO_2                (1<<1)   /**< servo 2 output */
#define GPIO_SERVO_3                (1<<2)   /**< servo 3 output */
#define GPIO_SERVO_4                (1<<3)   /**< servo 4 output */
#define GPIO_SERVO_5                (1<<4)   /**< servo 5 output */
#define GPIO_SERVO_6                (1<<5)   /**< servo 6 output */
#define GPIO_SERVO_7                (1<<6)   /**< servo 7 output */
#define GPIO_SERVO_8                (1<<7)   /**< servo 8 output */

/* This board provides a DMA pool and APIs */

#define BOARD_DMA_ALLOC_POOL_SIZE 5120

/* This board provides the board_on_reset interface */

#define BOARD_HAS_ON_RESET 1

/* The list of GPIO that will be initialized */

#define PX4_GPIO_PWM_INIT_LIST { \
		GPIO_GPIO7_INPUT, \
		GPIO_GPIO6_INPUT, \
		GPIO_GPIO5_INPUT, \
		GPIO_GPIO4_INPUT, \
		GPIO_GPIO3_INPUT, \
		GPIO_GPIO2_INPUT, \
		GPIO_GPIO1_INPUT, \
		GPIO_GPIO0_INPUT, \
	}

#define PX4_GPIO_INIT_LIST { \
		PX4_ADC_GPIO,                     \
	}

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

void board_spi_reset(int ms);
#define board_peripheral_reset(ms)

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
