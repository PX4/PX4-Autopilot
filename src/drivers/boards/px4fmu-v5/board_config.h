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
 * PX4FMU-v5 internal definitions
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

/* PX4FMU GPIOs ***********************************************************************************/
/* LEDs */
#define GPIO_LED1              /* PB1 */  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTB|GPIO_PIN1)
#define GPIO_LED2              /* PC6 */  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTC|GPIO_PIN6)
#define GPIO_LED3              /* PC7 */  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTC|GPIO_PIN7)

#define GPIO_LED_RED 	GPIO_LED1
#define GPIO_LED_GREEN 	GPIO_LED2
#define GPIO_LED_BLUE   GPIO_LED3

/*  Define the Chip Selects */

#define GPIO_SPI_CS_ICM20689    /* PF2 */  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTF|GPIO_PIN2)
#define GPIO_SPI_CS_ICM20602    /* PF3 */  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTF|GPIO_PIN3)
#define GPIO_SPI_CS_BMI055_GYR  /* PF4 */  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTF|GPIO_PIN4)
#define GPIO_SPI_CS_MEMORY      /* PF5 */  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTF|GPIO_PIN5)
#define GPIO_SPI_CS_MS5611      /* PF10 */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTF|GPIO_PIN10)
#define GPIO_SPI_CS_BMI055_ACC  /* PG10 */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTG|GPIO_PIN10)

#define SPI6_EXTERNAL1_CS1     /* PI4  */  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTI|GPIO_PIN4)
#define SPI6_EXTERNAL1_CS2     /* PI10 */  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTI|GPIO_PIN10)
#define SPI6_EXTERNAL1_CS3     /* PI11 */  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTI|GPIO_PIN11)
#define SPI6_EXTERNAL2_CS1     /* PI6  */  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTI|GPIO_PIN6)
#define SPI6_EXTERNAL2_CS2     /* PI7  */  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTI|GPIO_PIN7)
#define SPI6_EXTERNAL2_CS3     /* PI8  */  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTI|GPIO_PIN8)

/*  Define the Ready interrupts */ /* NOT FMUv5 test HW ONLY*/

#define GPIO_ICM20689_DRDY1    /* PB4 */   (GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI|GPIO_PORTB|GPIO_PIN4)
#define GPIO_BMI055_DRDY2_GYRO /* PB14 */  (GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI|GPIO_PORTB|GPIO_PIN14)
#define GPIO_BMI055_DRDY3_ACC  /* PB15 */  (GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI|GPIO_PORTB|GPIO_PIN15)
#define GPIO_ICM20602_DRDY4    /* PC5  */  (GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI|GPIO_PORTC|GPIO_PIN5)
#define GPIO_BMI055_DRDY5_GYRO /* PC13 */  (GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI|GPIO_PORTC|GPIO_PIN13)
#define GPIO_BMI055_DRDY6_ACC  /* PD10 */  (GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI|GPIO_PORTD|GPIO_PIN10)

/*
 *  Define the ability to shut off off the sensor signals
 *  by changing the signals to inputs
 */

#define _PIN_OFF(def) (((def) & (GPIO_PORT_MASK | GPIO_PIN_MASK)) | (GPIO_INPUT|GPIO_PULLDOWN|GPIO_SPEED_2MHz))


#define GPIO_DRDY_OFF_ICM20689_DRDY1    _PIN_OFF(GPIO_ICM20689_DRDY1)
#define GPIO_DRDY_OFF_BMI055_DRDY2_GYRO _PIN_OFF(GPIO_BMI055_DRDY2_GYRO)
#define GPIO_DRDY_OFF_BMI055_DRDY3_ACC  _PIN_OFF(GPIO_BMI055_DRDY3_ACC)
#define GPIO_DRDY_OFF_ICM20602_DRDY4    _PIN_OFF(GPIO_ICM20602_DRDY4)
#define GPIO_DRDY_OFF_BMI055_DRDY5_GYRO _PIN_OFF(GPIO_BMI055_DRDY5_GYRO)
#define GPIO_DRDY_OFF_BMI055_DRDY6_ACC  _PIN_OFF(GPIO_BMI055_DRDY6_ACC)


/* SPI1 off */
#define GPIO_SPI1_SCK_OFF	_PIN_OFF(GPIO_SPI1_SCK)
#define GPIO_SPI1_MISO_OFF	_PIN_OFF(GPIO_SPI1_MISO)
#define GPIO_SPI1_MOSI_OFF	_PIN_OFF(GPIO_SPI1_MOSI)

/* SENSORS are on SPI1
 * MEMORY is on bus SPI2
 * MS5611 is on bus SPI4
 */
#define PX4_SPI_BUS_SENSORS  1
#define PX4_SPI_BUS_MEMORY   2
#define PX4_SPI_BUS_BARO     4
#define PX4_SPI_BUS_EXTERNAL 6

/* v BEGIN Legacy SPI defines TODO: fix this with enumeration */
#define PX4_SPI_BUS_RAMTRON  PX4_SPI_BUS_MEMORY
#define PX4_SPIDEV_BMA 0
#define PX4_SPIDEV_BMI 0
/* ^ END Legacy SPI defines TODO: fix this with enumeration */


#define PX4_SPIDEV_ICM_20689      PX4_MK_SPI_SEL(PX4_SPI_BUS_SENSORS,0)
#define PX4_SPIDEV_ICM_20602      PX4_MK_SPI_SEL(PX4_SPI_BUS_SENSORS,1)
#define PX4_SPIDEV_BMI055_GYRO    PX4_MK_SPI_SEL(PX4_SPI_BUS_SENSORS,2)
#define PX4_SPIDEV_BMI055_ACCEL   PX4_MK_SPI_SEL(PX4_SPI_BUS_SENSORS,3)

#define PX4_SENSOR_BUS_CS_GPIO    {GPIO_SPI_CS_ICM20689, GPIO_SPI_CS_ICM20602, GPIO_SPI_CS_BMI055_GYR, GPIO_SPI_CS_BMI055_ACC}
#define PX4_SENSORS_BUS_FIRST_CS  PX4_SPIDEV_ICM_20689
#define PX4_SENSORS_BUS_LAST_CS   PX4_SPIDEV_BMI055_ACCEL

#define PX4_SPIDEV_MEMORY         PX4_MK_SPI_SEL(PX4_SPI_BUS_MEMORY,0)
#define PX4_MEMORY_BUS_CS_GPIO    {GPIO_SPI_CS_MEMORY}
#define PX4_MEMORY_BUS_FIRST_CS   PX4_SPIDEV_MEMORY
#define PX4_MEMORY_BUS_LAST_CS    PX4_SPIDEV_MEMORY

#define PX4_SPIDEV_BARO           PX4_MK_SPI_SEL(PX4_SPI_BUS_BARO,0)
#define PX4_BARO_BUS_CS_GPIO      {GPIO_SPI_CS_MS5611}
#define PX4_BARO_BUS_FIRST_CS     PX4_SPIDEV_BARO
#define PX4_BARO_BUS_LAST_CS      PX4_SPIDEV_BARO

#define PX4_SPIDEV_EXTERNAL1      PX4_MK_SPI_SEL(PX4_SPI_BUS_EXTERNAL,0)
#define PX4_SPIDEV_EXTERNAL2      PX4_MK_SPI_SEL(PX4_SPI_BUS_EXTERNAL,1)
#define PX4_SPIDEV_EXTERNAL3      PX4_MK_SPI_SEL(PX4_SPI_BUS_EXTERNAL,2)
#define PX4_SPIDEV_EXTERNAL4      PX4_MK_SPI_SEL(PX4_SPI_BUS_EXTERNAL,3)
#define PX4_SPIDEV_EXTERNAL5      PX4_MK_SPI_SEL(PX4_SPI_BUS_EXTERNAL,4)
#define PX4_SPIDEV_EXTERNAL6      PX4_MK_SPI_SEL(PX4_SPI_BUS_EXTERNAL,5)
#define PX4_EXTERNAL_BUS_CS_GPIO  {SPI6_EXTERNAL1_CS1, SPI6_EXTERNAL1_CS2, \
		SPI6_EXTERNAL1_CS3, \
		SPI6_EXTERNAL2_CS1, SPI6_EXTERNAL2_CS2, \
		SPI6_EXTERNAL2_CS3}
#define PX4_EXTERNAL_BUS_FIRST_CS  PX4_SPIDEV_EXTERNAL1
#define PX4_EXTERNAL_BUS_LAST_CS   PX4_SPIDEV_EXTERNAL6


/* I2C busses */
#define PX4_I2C_BUS_EXPANSION	1
#define PX4_I2C_BUS_EXPANSION1	2
#define PX4_I2C_BUS_EXPANSION2	4
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

#define ADC_BATTERY_VOLTAGE_CHANNEL     0
#define ADC_BATTERY_CURRENT_CHANNEL     1
#define ADC_BATTERY1_VOLTAGE_CHANNEL    2
#define ADC_BATTERY1_CURRENT_CHANNEL    3
#define ADC_5V_RAIL_SENSE               4 /* NOT to FMUv5 spec on test HW ONLY - delete this on when running on FMUv5 spec-ed HW */
#define ADC_RC_RSSI_CHANNEL             8
#define ADC_SCALED_V5                   10
#define ADC_SCALED_V3V3                 11
#define ADC_INT_3                       12
#define ADC_INT_4                       13
#define ADC_INT_5                       14

/* CAN Silence
 *
 * Silent mode control
 */

#define GPIO_CAN1_SILENCE  /* PH2  */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTH|GPIO_PIN2)
#define GPIO_CAN2_SILENCE  /* PH3  */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTH|GPIO_PIN3)
#define GPIO_CAN3_SILENCE  /* PH4  */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTH|GPIO_PIN4)

/* HEATER
 * PWM in future
 */
#define GPIO_HEATER        /* PA7  */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTA|GPIO_PIN7)

/* PWM
 *
 * 4  PWM outputs are configured.
 *
 * Pins:
 *
 * FMU_CH1 : PE14 : TIM1_CH4
 * FMU_CH2 : PA10 : TIM1_CH3
 * FMU_CH3 : PE11 : TIM1_CH2
 * FMU_CH4 : PE9  : TIM1_CH1
 * FMU_CH5 : PD13 : TIM4_CH2
 * FMU_CH6 : PD14 : TIM4_CH3
 *
 */
#define GPIO_TIM4_CH3OUT       /* PD14  T4C3  FMU6 */ GPIO_TIM4_CH3OUT_2
#define GPIO_TIM4_CH2OUT       /* PD13  T4C2  FMU5 */ GPIO_TIM4_CH2OUT_2
#define GPIO_TIM1_CH1OUT       /* PE9   T1C1  FMU4 */ GPIO_TIM1_CH1OUT_2
#define GPIO_TIM1_CH2OUT       /* PE11  T1C2  FMU3 */ GPIO_TIM1_CH2OUT_2
#define GPIO_TIM1_CH3OUT       /* PA10  T1C3  FMU2 */ GPIO_TIM1_CH3OUT_1
#define GPIO_TIM1_CH4OUT       /* PE14  T1C4  FMU1 */ GPIO_TIM1_CH4OUT_2
#define DIRECT_PWM_OUTPUT_CHANNELS  6


#define GPIO_TIM4_CH3IN       /* PD14  T4C3  FMU6 */ GPIO_TIM4_CH3IN_2
#define GPIO_TIM4_CH2IN       /* PD13  T4C2  FMU5 */ GPIO_TIM4_CH2IN_2
#define GPIO_TIM1_CH1IN       /* PE9   T1C1  FMU4 */ GPIO_TIM1_CH1IN_2
#define GPIO_TIM1_CH2IN       /* PE11  T1C2  FMU3 */ GPIO_TIM1_CH2IN_2
#define GPIO_TIM1_CH3IN       /* PA10  T1C3  FMU2 */ GPIO_TIM1_CH3IN_1
#define GPIO_TIM1_CH4IN       /* PE14  T1C4  FMU1 */ GPIO_TIM1_CH4IN_2

#define DIRECT_INPUT_TIMER_CHANNELS  6

#define BOARD_HAS_LED_PWM
#define BOARD_LED_PWM_DRIVE_ACTIVE_LOW 1

#define LED_TIM3_CH1OUT  /* PC6   T3C1  GREEN */ GPIO_TIM3_CH1OUT_3
#define LED_TIM3_CH2OUT  /* PC7   T3C2  BLUE  */ GPIO_TIM3_CH2OUT_3
#define LED_TIM3_CH4OUT  /* PB1   T3C4  RED   */ GPIO_TIM3_CH4OUT_1

#define LED_TIM5_CH1OUT  /* PH10  T5C1  RED   */ GPIO_TIM5_CH1OUT_2
#define LED_TIM5_CH2OUT  /* PH11  T5C2  GREEN */ GPIO_TIM5_CH2OUT_2
#define LED_TIM5_CH3OUT  /* PH12  T5C3  BLUE  */ GPIO_TIM5_CH3OUT_2


/* User GPIOs
 *
 * GPIO0-5 are the PWM servo outputs.
 */

#define _MK_GPIO_INPUT(def) (((def) & (GPIO_PORT_MASK | GPIO_PIN_MASK)) | (GPIO_INPUT|GPIO_PULLUP))

#define GPIO_GPIO5_INPUT        /* PD14  T4C3  FMU6 */ _MK_GPIO_INPUT(GPIO_TIM4_CH3IN)
#define GPIO_GPIO4_INPUT        /* PD13  T4C2  FMU5 */ _MK_GPIO_INPUT(GPIO_TIM4_CH2IN)
#define GPIO_GPIO3_INPUT        /* PE9   T1C1  FMU4 */ _MK_GPIO_INPUT(GPIO_TIM1_CH1IN)
#define GPIO_GPIO2_INPUT        /* PE11  T1C2  FMU3 */ _MK_GPIO_INPUT(GPIO_TIM1_CH2IN)
#define GPIO_GPIO1_INPUT        /* PA10  T1C3  FMU2 */ _MK_GPIO_INPUT(GPIO_TIM1_CH3IN)
#define GPIO_GPIO0_INPUT        /* PE14  T1C4  FMU1 */ _MK_GPIO_INPUT(GPIO_TIM1_CH4IN)

#define _MK_GPIO_OUTPUT(def) (((def) & (GPIO_PORT_MASK | GPIO_PIN_MASK)) | (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR))

#define GPIO_GPIO5_OUTPUT        /* PD14  T4C3  FMU6 */ _MK_GPIO_OUTPUT(GPIO_TIM4_CH3OUT)
#define GPIO_GPIO4_OUTPUT        /* PD13  T4C2  FMU5 */ _MK_GPIO_OUTPUT(GPIO_TIM4_CH2OUT)
#define GPIO_GPIO3_OUTPUT        /* PE9   T1C1  FMU4 */ _MK_GPIO_OUTPUT(GPIO_TIM1_CH1OUT)
#define GPIO_GPIO2_OUTPUT        /* PE11  T1C2  FMU3 */ _MK_GPIO_OUTPUT(GPIO_TIM1_CH2OUT)
#define GPIO_GPIO1_OUTPUT        /* PA10  T1C3  FMU2 */ _MK_GPIO_OUTPUT(GPIO_TIM1_CH3OUT)
#define GPIO_GPIO0_OUTPUT        /* PE14  T1C4  FMU1 */ _MK_GPIO_OUTPUT(GPIO_TIM1_CH4OUT)


/* Power supply control and monitoring GPIOs */

#define GPIO_POWER_IN_A           /* PG1  */ (GPIO_INPUT|GPIO_PULLUP|GPIO_PORTG|GPIO_PIN1)
#define GPIO_POWER_IN_B           /* PG2  */ (GPIO_INPUT|GPIO_PULLUP|GPIO_PORTG|GPIO_PIN2)
#define GPIO_POWER_IN_C           /* PG3  */ (GPIO_INPUT|GPIO_PULLUP|GPIO_PORTG|GPIO_PIN3)

#define GPIO_VDD_BRICK_VALID      /* PB10 */ (GPIO_INPUT|GPIO_PULLUP|GPIO_PORTB|GPIO_PIN10)
#define GPIO_VDD_3V3_SENSORS_EN   /* PE3  */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTE|GPIO_PIN3)
#define GPIO_SPEKTRUM_POWER_EN    /* PE4  */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTE|GPIO_PIN4)
#define GPIO_PERIPH_5V_EN         /* PG4  */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTG|GPIO_PIN4)
#define GPIO_VDD_5V_RC_EN         /* PG5  */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTG|GPIO_PIN5)
#define GPIO_VDD_5V_WIFI_EN       /* PG6  */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTG|GPIO_PIN5)
#define GPIO_VDD_3V3V_SD_CARD_EN  /* PG7  */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTG|GPIO_PIN7)

/* Active Low SD Card Power enable */

#define SD_CARD_POWER_CTRL(on_true)    px4_arch_gpiowrite(GPIO_VDD_3V3V_SD_CARD_EN, !(on_true))

#define GPIO_VDD_5V_PERIPH_OC     /* PE15 */ (GPIO_INPUT|GPIO_PULLUP|GPIO_PORTE|GPIO_PIN15)

/* Tone alarm output */
#define TONE_ALARM_TIMER		12	/* timer 12 */
#define TONE_ALARM_CHANNEL      /* PH6 TIM12_CH1 */ 1	/* channel 1 */
#define GPIO_TONE_ALARM_IDLE    /* PH6 */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTH|GPIO_PIN6)
#define GPIO_TONE_ALARM         /* PH6 */ (GPIO_ALT|GPIO_AF1|GPIO_SPEED_2MHz|GPIO_PUSHPULL|GPIO_PORTH|GPIO_PIN6)

/* USB OTG FS
 *
 * PA9  OTG_FS_VBUS VBUS sensing
 */
#define GPIO_OTGFS_VBUS         /* PA9 */ (GPIO_INPUT|GPIO_FLOAT|GPIO_SPEED_100MHz|GPIO_PORTA|GPIO_PIN9)

/* High-resolution timer */
#define HRT_TIMER		    8	/* use timer8 for the HRT */
#define HRT_TIMER_CHANNEL   3	/* use capture/compare channel 3 */

#define HRT_PPM_CHANNEL         /* T8C1 */  1	/* use capture/compare channel 1 */
#define GPIO_PPM_IN             /* PI5 T8C1 */ GPIO_TIM8_CH1IN_2

#define RC_UXART_BASE        STM32_USART6_BASE /* NOT FMUv5 test HW ONLY*/
#define RC_SERIAL_PORT       "/dev/ttyS5"      /* NOT FMUv5 test HW ONLY*/

#define GPS_DEFAULT_UART_PORT "/dev/ttyS0" /* UART1 on FMUv5 */

/* PWM input driver. Use FMU AUX5 pins attached to timer4 channel 2 */
#define PWMIN_TIMER             4
#define PWMIN_TIMER_CHANNEL     /* T4C2 */ 2
#define GPIO_PWM_IN             /* PD13 */ GPIO_TIM4_CH2IN

#define GPIO_RSSI_IN            /* PB0  */ (GPIO_INPUT|GPIO_PULLUP|GPIO_PORTB|GPIO_PIN0)

#define GPIO_LED_SAFETY         /* PE12 */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTE|GPIO_PIN12)
#define GPIO_BTN_SAFETY         /* PE10 */ (GPIO_INPUT|GPIO_PULLUP|GPIO_PORTE|GPIO_PIN10)

#define INVERT_RC_INPUT(_s)		board_rc_input(_s);


/* Power switch controls ******************************************************/

#define POWER_SPEKTRUM(_s)      px4_arch_gpiowrite(GPIO_SPEKTRUM_POWER_EN, (1-_s))
#define SPEKTRUM_RX_AS_UART()   px4_arch_configgpio(GPIO_USART6_RX) /* NOT FMUv5 test HW ONLY*/

// FMUv5 has a separate GPIO for serial RC output
#define GPIO_RC_OUT            /* PG9 */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTG|GPIO_PIN9)
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

#define	BOARD_NAME "PX4FMU_V5"

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
		{GPIO_GPIO0_INPUT,       GPIO_GPIO0_OUTPUT,        0}, \
		{GPIO_GPIO1_INPUT,       GPIO_GPIO1_OUTPUT,        0}, \
		{GPIO_GPIO2_INPUT,       GPIO_GPIO2_OUTPUT,        0}, \
		{GPIO_GPIO3_INPUT,       GPIO_GPIO3_OUTPUT,        0}, \
		{GPIO_GPIO4_INPUT,       GPIO_GPIO4_OUTPUT,        0}, \
		{GPIO_GPIO5_INPUT,       GPIO_GPIO5_OUTPUT,        0}, \
		{GPIO_POWER_IN_A,        0,                        0}, \
		{GPIO_POWER_IN_B,        0,                        0}, \
		{GPIO_POWER_IN_C,        0,                        0}, \
		{0,                      GPIO_VDD_3V3_SENSORS_EN,  0}, \
		{GPIO_VDD_BRICK_VALID,   0,                        0}, \
		{0,                      GPIO_SPEKTRUM_POWER_EN,   0}, \
		{0,                      GPIO_PERIPH_5V_EN,        0}, \
		{0,                      GPIO_VDD_5V_RC_EN,        0}, \
		{0,                      GPIO_VDD_5V_WIFI_EN,      0}, \
		{0,                      GPIO_VDD_3V3V_SD_CARD_EN, 0}, }

/*
 * GPIO numbers.
 *
 * There are no alternate functions on this board.
 */
#define GPIO_SERVO_1             (1<<0)   /**< servo 1 output */
#define GPIO_SERVO_2             (1<<1)   /**< servo 2 output */
#define GPIO_SERVO_3             (1<<2)   /**< servo 3 output */
#define GPIO_SERVO_4             (1<<3)   /**< servo 4 output */
#define GPIO_SERVO_5             (1<<4)   /**< servo 5 output */
#define GPIO_SERVO_6             (1<<5)   /**< servo 6 output */

#define GPIO_POWER_INPUT_A       (1<<6)   /**<PG1 GPIO_POWER_IN_A */
#define GPIO_POWER_INPUT_B       (1<<7)   /**<PG2 GPIO_POWER_IN_B */
#define GPIO_POWER_INPUT_C       (1<<8)   /**<PG3 GPIO_POWER_IN_C */

#define GPIO_3V3_SENSORS_EN      (1<<9)   /**< PE3  - VDD_3V3_SENSORS_EN */
#define GPIO_BRICK_VALID         (1<<10)  /**< PB10 - !VDD_BRICK_VALID */
#define GPIO_SPEKTRUM_POWER      (1<<11)  /**< PE4  - GPIO_SPEKTRUM_POWER_EN */

#define GPIO_PERIPH_5V_POWER_EN  (1<<12)  /**< PG4  - GPIO_PERIPH_5V_EN        */
#define GPIO_RC_POWER_EN         (1<<13)  /**< PG5  - GPIO_VDD_5V_RC_EN        */
#define GPIO_WIFI_POWER_EN       (1<<14)  /**< PG6  - GPIO_VDD_5V_WIFI_EN      */
#define GPIO_SD_CARD_POWER_EN    (1<<15)  /**< PG7  - GPIO_VDD_3V3V_SD_CARD_EN */

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
