/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
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
 * KakuteF7 internal definitions
 */

#pragma once

/****************************************************************************************************
 * Included Files
 ****************************************************************************************************/

#include <px4_config.h>
#include <nuttx/compiler.h>
#include <stdint.h>

#include <stm32_gpio.h>

/****************************************************************************************************
 * Definitions
 ****************************************************************************************************/

/* GPIOs ***********************************************************************************/

/* LEDs are driven with push open drain to support Anode to 5V or 3.3V */

#define GPIO_nLED_BLUE       /* PA2 */  (GPIO_OUTPUT|GPIO_OPENDRAIN|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTA|GPIO_PIN2)

#define BOARD_HAS_CONTROL_STATUS_LEDS      1
#define BOARD_ARMED_STATE_LED  LED_BLUE

#define  FLASH_BASED_PARAMS

/* SPI busses
 */
#define PX4_SPI_BUS_SD_CARD   1
#define PX4_SPI_BUS_OSD   	2
#define PX4_SPI_BUS_SENSORS   4

/*  Define the Chip Selects, Data Ready and Control signals per SPI bus */

#define GPIO_SPI1_CS1_SD_CARD    /* PA4 */  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTA|GPIO_PIN4)

#define PX4_SD_CARD_BUS_CS_GPIO      {GPIO_SPI1_CS1_SD_CARD}

#define GPIO_SPI2_CS1_OSD    /* PB12 */  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTB|GPIO_PIN12)
#define PX4_SPIDEV_OSD              1

#define PX4_OSD_BUS_CS_GPIO      {GPIO_SPI2_CS1_OSD}

/* SPI 4 CS (SPI4_SS) */

#define GPIO_SPI4_CS1_ICM20689    /* PE4 */  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTE|GPIO_PIN4)


/*  Define the SPI4 Data Ready interrupts */

#define GPIO_SPI4_DRDY1_ICM20689    /* PE1 */   (GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI|GPIO_PORTE|GPIO_PIN1)


#define PX4_SPIDEV_MPU              PX4_MK_SPI_SEL(PX4_SPI_BUS_SENSORS,0)
#define PX4_SPIDEV_ICM_20689        PX4_MK_SPI_SEL(PX4_SPI_BUS_SENSORS,0)
#define PX4_SENSOR_BUS_CS_GPIO      {GPIO_SPI4_CS1_ICM20689}

/* I2C busses */

#define PX4_I2C_BUS_EXPANSION       1
#define PX4_I2C_BUS_LED             PX4_I2C_BUS_EXPANSION

#define BOARD_NUMBER_I2C_BUSES      1
#define BOARD_I2C_BUS_CLOCK_INIT    {100000}

#define PX4_I2C_OBDEV_BMP280        0x76

/*
 * ADC channels
 *
 * These are the channel numbers of the ADCs of the microcontroller that
 * can be used by the Px4 Firmware in the adc driver
 */

/* ADC defines to be used in sensors.cpp to read from a particular channel */
#define ADC1_CH(n)                  (n)
#define ADC1_GPIO(n)                GPIO_ADC1_IN##n

/* Define GPIO pins used as ADC N.B. Channel numbers must match below */
#define PX4_ADC_GPIO  \
	/* PC2 */  ADC1_GPIO(12),  \
	/* PC3 */  ADC1_GPIO(13),  \
	/* PC5 */  ADC1_GPIO(15)

/* Define Channel numbers must match above GPIO pin IN(n)*/
#define ADC_RSSI_IN_CHANNEL                 /* PC5 */  ADC1_CH(15)
#define ADC_BATTERY_VOLTAGE_CHANNEL         /* PC3 */  ADC1_CH(13)
#define ADC_BATTERY_CURRENT_CHANNEL         /* PC2 */  ADC1_CH(12)

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
#define GPIO_TIM5_CH4OUT      /* PA3   T5C4   PWM6 */ GPIO_TIM5_CH4OUT_1
#define GPIO_TIM8_CH4OUT      /* PC9   T8C4   PWM5 */ GPIO_TIM8_CH4OUT_1
#define GPIO_TIM1_CH2OUT      /* PE11  T1C2   PWM4 */ GPIO_TIM1_CH2OUT_2
#define GPIO_TIM1_CH1OUT      /* PE9   T1C1   PWM3 */ GPIO_TIM1_CH1OUT_2
#define GPIO_TIM3_CH4OUT      /* PB1   T3C4   PWM2 */ GPIO_TIM3_CH4OUT_1
#define GPIO_TIM3_CH3OUT      /* PB0   T3C3   PWM1 */ GPIO_TIM3_CH3OUT_1

#define DIRECT_PWM_OUTPUT_CHANNELS  6

#define GPIO_TIM5_CH4IN       /* PA3   T5C4   PWM6 */ GPIO_TIM5_CH4IN_1
#define GPIO_TIM8_CH4IN       /* PC9   T8C4   PWM5 */ GPIO_TIM8_CH4IN_1
#define GPIO_TIM1_CH2IN       /* PE11  T1C2   PWM4 */ GPIO_TIM1_CH2IN_2
#define GPIO_TIM1_CH1IN       /* PE9   T1C1   PWM3 */ GPIO_TIM1_CH1IN_2
#define GPIO_TIM3_CH4IN       /* PB1   T3C4   PWM2 */ GPIO_TIM3_CH4IN_1
#define GPIO_TIM3_CH3IN       /* PB0   T3C3   PWM1 */ GPIO_TIM3_CH3IN_1

#define DIRECT_INPUT_TIMER_CHANNELS  6

/* User GPIOs
 */

#define _MK_GPIO_INPUT(def) (((def) & (GPIO_PORT_MASK | GPIO_PIN_MASK)) | (GPIO_INPUT|GPIO_PULLDOWN))

#define GPIO_GPIO5_INPUT        /* PWM6 */ _MK_GPIO_INPUT(GPIO_TIM5_CH4IN)
#define GPIO_GPIO4_INPUT        /* PWM5 */ _MK_GPIO_INPUT(GPIO_TIM8_CH4IN)
#define GPIO_GPIO3_INPUT        /* PWM4 */ _MK_GPIO_INPUT(GPIO_TIM1_CH2IN)
#define GPIO_GPIO2_INPUT        /* PWM3 */ _MK_GPIO_INPUT(GPIO_TIM1_CH1IN)
#define GPIO_GPIO1_INPUT        /* PWM2 */ _MK_GPIO_INPUT(GPIO_TIM3_CH4IN)
#define GPIO_GPIO0_INPUT        /* PWM1 */ _MK_GPIO_INPUT(GPIO_TIM3_CH3IN)

/* Tone alarm output */
#define GPIO_TONE_ALARM_IDLE    /* PD15 */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTD|GPIO_PIN15)
#define GPIO_TONE_ALARM_GPIO    /* PD15 */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTD|GPIO_PIN15)

/* USB OTG FS
 *
 * PA8  OTG_FS_VBUS VBUS sensing
 */
#define GPIO_OTGFS_VBUS         /* PA8 */ (GPIO_INPUT|GPIO_PULLDOWN|GPIO_SPEED_100MHz|GPIO_PORTA|GPIO_PIN8)

/* High-resolution timer */
#define HRT_TIMER               2  /* use timer 2 for the HRT */
#define HRT_TIMER_CHANNEL       1  /* use capture/compare channel 1 */

/* RC Serial port */

#define RC_SERIAL_PORT                     "/dev/ttyS4"

#define GPIO_RSSI_IN                       /* PC5  */ (GPIO_INPUT|GPIO_PULLUP|GPIO_PORTC|GPIO_PIN5)

#define BOARD_HAS_PWM  DIRECT_PWM_OUTPUT_CHANNELS

/* This board provides a DMA pool and APIs */
#define BOARD_DMA_ALLOC_POOL_SIZE 5120

/* This board provides the board_on_reset interface */
#define BOARD_HAS_ON_RESET 1

/* The list of GPIO that will be initialized */

#define PX4_GPIO_PWM_INIT_LIST { \
		GPIO_GPIO5_INPUT, \
		GPIO_GPIO4_INPUT, \
		GPIO_GPIO3_INPUT, \
		GPIO_GPIO2_INPUT, \
		GPIO_GPIO1_INPUT, \
		GPIO_GPIO0_INPUT, \
	}

#define PX4_GPIO_INIT_LIST { \
		PX4_ADC_GPIO,                     \
		GPIO_TONE_ALARM_IDLE,             \
		GPIO_RSSI_IN,                \
	}

#define BOARD_ENABLE_CONSOLE_BUFFER

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
 *   Called to configure SPI chip select GPIO pins for the board.
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

#include <drivers/boards/common/board_common.h>

#endif /* __ASSEMBLY__ */

__END_DECLS
