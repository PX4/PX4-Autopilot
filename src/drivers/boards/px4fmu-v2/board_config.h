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

/****************************************************************************************************
 * Definitions
 ****************************************************************************************************/
/* Configuration ************************************************************************************/

/* PX4IO connection configuration */
#define BOARD_USES_PX4IO_VERSION       2
#define PX4IO_SERIAL_DEVICE	"/dev/ttyS4"
#define PX4IO_SERIAL_TX_GPIO	GPIO_USART6_TX
#define PX4IO_SERIAL_RX_GPIO	GPIO_USART6_RX
#define PX4IO_SERIAL_BASE	STM32_USART6_BASE	/* hardwired on the board */
#define PX4IO_SERIAL_VECTOR	STM32_IRQ_USART6
#define PX4IO_SERIAL_TX_DMAMAP	DMAMAP_USART6_TX_2
#define PX4IO_SERIAL_RX_DMAMAP	DMAMAP_USART6_RX_2
#define PX4IO_SERIAL_CLOCK	STM32_PCLK2_FREQUENCY
#define PX4IO_SERIAL_BITRATE	1500000			/* 1.5Mbps -> max rate for IO */


/* PX4FMU GPIOs ***********************************************************************************/
/* LEDs */

#define GPIO_LED1		(GPIO_OUTPUT|GPIO_OPENDRAIN|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTE|GPIO_PIN12)

/*
 *  Define the ability to shut off off the sensor signals
 *  by changing the signals to inputs
 */

#define _PIN_OFF(def) (((def) & (GPIO_PORT_MASK | GPIO_PIN_MASK)) | (GPIO_INPUT|GPIO_PULLDOWN|GPIO_SPEED_50MHz))

/* Due to inconsistent use of chip select and dry signal on
 * different board that use this build. We are defining the GPIO
 * inclusive of the SPI port and GPIO to help identify pins the
 * are part of the sensor Net's controlled by different power
 * domains.
 *
 *  Only the GPIO_SPIb_xxx_Ppi will be used in the code to insure this are no
 *  cross connections.
 *
 *  --------------- SPI1 -------------------- SPI4 --------------     Incompatibilities    ---------
 *  FMUv2:                                                        FmuV3 Cube        PixhawkMini
 *   Power Domain:  VDD_3V3_SENSORS_EN        nVDD_5V_PERIPH_EN    V3V:SPI1&SPI4    V3V:SPI1 No SPI4
 *   PA5            SPI_INT_SCK
 *   PA6            SPI_INT_MISO
 *   PA7            SPI_INT_MOSI
 *   PB0            GYRO_DRDY                                      SPI4:EXTERN_DRDY        NC
 *   PB1            MAG_DRDY                                       +SPI4:nEXTERN_CS        NC
 *   PB4            ACCEL_DRDY                                     NC                      NC
 *   PC1            Spare ADC ( NC )                               +SPI1:SPI_INT_MAG_!CS
 *   PC2            nMPU_CS                                        @MPU6000|MPU9250        @MPU9250
 *   PC13           nGYRO_CS                                       SPI4:nGYRO_EXT_CS       NC
 *   PC14                                     GPIO_EXT_1           nBARO_EXT_CS            -20608_DRDY
 *   PC15           nACCEL_MAG_CS                                  SPI4:nACCEL_MAG_EXT_CS  20608_CS
 *   PD7            nBARO_CS
 *   PD15           nMPU_DRDY                                      @MPU6000|MPU9250        @MPU9250
 *   PE2                                      SPI_EXT_SCK                                  NC
 *   PE4                                      nSPI_EXT_NSS         SPI4:nMPU_EXT_CS        NC
 *   PE5                                      SPI_EXT_MISO                                 NC
 *   PE6                                      SPI_EXT_MOSI                                 NC
 *
 *   Notes: Prefixed with @ Does not effect board control
 *          Prefixed with + Input used as Output
 *          Prefixed with - Output used as Input
 *          Prefixed with SPIn: Bus changed
 *
 *  The board API provides for mechanism to perform a SPI bus reset.
 *  To facilitate a SPI bus reset
 *
 *    1) All the pins: SPIn, CD, DRDY associated with the SPI bus are turned to inputs
 *       with outputs driven low. (OFFIng)
 *    2) The power domain of that bus is turned off.
 *    3) A usleep it done for ms.
 *    4) The power domain of that bus is turned back on.
 *    5) The SPIn pins are re-initialized.
 *    6) The SPI CS, DRDY pins are re-initialized.
 *
 * To insure the complete net is de-energized and is not bing back fed, it is important to
 * note the all signals in the net list of the parts/bus.
 *
 * I.E. Not OFFIng PC1 on V3 would leave that pin back feeding the HMC part. As would not
 * OFFIng PE4, not associated with SPI1 on V2, but would back feed an MPUxxxx on V3
 *
 */


/*----------------------------------------------------------*/
/*            FMUv2 SPI chip selects and DRDY               */
/*----------------------------------------------------------*/

/* FMUv2 SPI1 chip selects */
/*                   PC1 Spare ADC IN10                     */
#define GPIO_SPI1_CS_PC2                 (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTC|GPIO_PIN2)
#define GPIO_SPI1_CS_PC13                (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTC|GPIO_PIN13)
#define GPIO_SPI1_CS_PC15                (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTC|GPIO_PIN15)
#define GPIO_SPI1_CS_PD7                 (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTD|GPIO_PIN7)

/* FMUv2 SPI2 chip selects */
#define GPIO_SPI2_CS_PD10                (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTD|GPIO_PIN10)

/* FMUv2 SPI4 chip selects */
#define GPIO_SPI4_GPIO_PC14  /* !V2M */  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTC|GPIO_PIN14)
#define GPIO_SPI4_NSS_PE4                (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTE|GPIO_PIN4)

/* FMUv2 SPI1 chip selects Assignments */

#define GPIO_SPI1_CS_MPU                 GPIO_SPI1_CS_PC2
#define GPIO_SPI1_CS_GYRO                GPIO_SPI1_CS_PC13
#define GPIO_SPI1_CS_ACCEL_MAG           GPIO_SPI1_CS_PC15
#define GPIO_SPI1_CS_BARO                GPIO_SPI1_CS_PD7

/* FMUv2 SPI2 chip selects Assignments */

#define GPIO_SPI2_CS_FRAM                GPIO_SPI2_CS_PD10

/* FMUv2 SPI4 chip selects Assignments */

#define GPIO_SPI4_GPIO_EXT               GPIO_SPI4_GPIO_PC14
#define GPIO_SPI4_EXT_NSS                GPIO_SPI4_NSS_PE4

/* FMUv2 DRDY */

#define GPIO_SPI1_EXTI_DRDY_PB0          (GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI|GPIO_PORTB|GPIO_PIN0)
#define GPIO_SPI1_EXTI_DRDY_PB1 /*!V3 */ (GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI|GPIO_PORTB|GPIO_PIN1)
#define GPIO_SPI1_EXTI_DRDY_PB4          (GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI|GPIO_PORTB|GPIO_PIN4)
#define GPIO_SPI1_EXTI_DRDY_PD15         (GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI|GPIO_PORTD|GPIO_PIN15)

/* FMUv2 DRDY Assignments */

#define GPIO_SPI1_EXTI_GYRO_DRDY         GPIO_SPI1_EXTI_DRDY_PB0
#define GPIO_SPI1_EXTI_MAG_DRDY          GPIO_SPI1_EXTI_DRDY_PB1
#define GPIO_SPI1_EXTI_ACCEL_DRDY        GPIO_SPI1_EXTI_DRDY_PB4
#define GPIO_SPI1_EXTI_MPU_DRDY          GPIO_SPI1_EXTI_DRDY_PD15

/*----------------------------------------------------------*/
/*        End FMUv2 SPI chip selects and DRDY               */
/*----------------------------------------------------------*/

#define PX4_SPI_BUS_SENSORS	1
#define PX4_SPI_BUS_RAMTRON	2
#define PX4_SPI_BUS_EXT		4
#define PX4_SPI_BUS_BARO	PX4_SPI_BUS_SENSORS

/* Use these in place of the spi_dev_e enumeration to select a specific SPI device on SPI1 */
#define PX4_SPIDEV_GYRO           1
#define PX4_SPIDEV_ACCEL_MAG      2
#define PX4_SPIDEV_BARO           3
#define PX4_SPIDEV_MPU            4

/* External bus */
#define PX4_SPIDEV_EXT_MPU        10
#define PX4_SPIDEV_EXT_GYRO       12
#define PX4_SPIDEV_EXT_ACCEL_MAG  13
#define PX4_SPIDEV_EXT_BARO       14
#define PX4_SPIDEV_EXT_BMI        15

/* I2C busses */
#define PX4_I2C_BUS_EXPANSION	1
#define PX4_I2C_BUS_ONBOARD	2
#define PX4_I2C_BUS_LED		PX4_I2C_BUS_ONBOARD

/* Devices on the onboard bus.
 *
 * Note that these are unshifted addresses.
 */
#define PX4_I2C_OBDEV_LED	0x55
#define PX4_I2C_OBDEV_HMC5883	0x1e
#define PX4_I2C_OBDEV_LIS3MDL	0x1e

/*
 * ADC channels
 *
 * These are the channel numbers of the ADCs of the microcontroller that can be used by the Px4 Firmware in the adc driver
 */
#define ADC_CHANNELS (1 << 2) | (1 << 3) | (1 << 4) | (1 << 10) | (1 << 11) | (1 << 12) | (1 << 13) | (1 << 14) | (1 << 15)

// ADC defines to be used in sensors.cpp to read from a particular channel
#define ADC_BATTERY_VOLTAGE_CHANNEL	2
#define ADC_BATTERY_CURRENT_CHANNEL	3
#define ADC_5V_RAIL_SENSE		4
#define ADC_AIRSPEED_VOLTAGE_CHANNEL	15

/* Define Battery 1 Voltage Divider and A per V
 */

#define BOARD_BATTERY1_V_DIV   (10.177939394f)
#define BOARD_BATTERY1_A_PER_V (15.391030303f)

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
#define GPIO_VDD_5V_PERIPH_EN	(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTA|GPIO_PIN8)
#define GPIO_VDD_BRICK_VALID	(GPIO_INPUT|GPIO_PULLUP|GPIO_PORTB|GPIO_PIN5)
#define GPIO_VDD_SERVO_VALID	(GPIO_INPUT|GPIO_PULLUP|GPIO_PORTB|GPIO_PIN7)
#define GPIO_VDD_USB_VALID		(GPIO_INPUT|GPIO_PULLUP|GPIO_PORTC|GPIO_PIN0)
#define GPIO_VDD_3V3_SENSORS_EN	(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTE|GPIO_PIN3)
#define GPIO_VDD_5V_HIPOWER_OC	(GPIO_INPUT|GPIO_PULLUP|GPIO_PORTE|GPIO_PIN10)
#define GPIO_VDD_5V_PERIPH_OC	(GPIO_INPUT|GPIO_PULLUP|GPIO_PORTE|GPIO_PIN15)

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
 * PA9  OTG_FS_VBUS VBUS sensing (also connected to the green LED)
 */
#define GPIO_OTGFS_VBUS		(GPIO_INPUT|GPIO_FLOAT|GPIO_SPEED_100MHz|GPIO_OPENDRAIN|GPIO_PORTA|GPIO_PIN9)

/* High-resolution timer */
#define HRT_TIMER		8	/* use timer8 for the HRT */
#define HRT_TIMER_CHANNEL	1	/* use capture/compare channel */

/* PWM input driver. Use FMU AUX5 pins attached to timer4 channel 2 */
#define PWMIN_TIMER		4
#define PWMIN_TIMER_CHANNEL	2
#define GPIO_PWM_IN		GPIO_TIM4_CH2IN_2

#define BOARD_NAME "PX4FMU_V2"

/* By Providing BOARD_ADC_USB_CONNECTED (using the px4_arch abstraction)
 * this board support the ADC system_power interface, and therefore
 * provides the true logic GPIO BOARD_ADC_xxxx macros.
 */
#define BOARD_ADC_USB_CONNECTED (px4_arch_gpioread(GPIO_OTGFS_VBUS))
#define BOARD_ADC_BRICK_VALID   (!px4_arch_gpioread(GPIO_VDD_BRICK_VALID))
#define BOARD_ADC_SERVO_VALID   (!px4_arch_gpioread(GPIO_VDD_SERVO_VALID))
#define BOARD_ADC_USB_VALID     (!px4_arch_gpioread(GPIO_VDD_USB_VALID))
#define BOARD_ADC_PERIPH_5V_OC  (!px4_arch_gpioread(GPIO_VDD_5V_PERIPH_OC))
#define BOARD_ADC_HIPOWER_5V_OC (!px4_arch_gpioread(GPIO_VDD_5V_HIPOWER_OC))

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
		{GPIO_VDD_BRICK_VALID,   0,                       0}, \
		{GPIO_VDD_SERVO_VALID,   0,                       0}, \
		{GPIO_VDD_USB_VALID,     0,                       0}, \
		{GPIO_VDD_5V_HIPOWER_OC, 0,                       0}, \
		{GPIO_VDD_5V_PERIPH_OC,  0,                       0}, }

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

#define GPIO_5V_PERIPH_EN     (1<<6)  /**< PA8 - !VDD_5V_PERIPH_EN */
#define GPIO_3V3_SENSORS_EN   (1<<7)  /**< PE3 - VDD_3V3_SENSORS_EN */
#define GPIO_BRICK_VALID      (1<<8)  /**< PB5 - !VDD_BRICK_VALID */
#define GPIO_SERVO_VALID      (1<<9)  /**< PB7 - !VDD_SERVO_VALID */
#define GPIO_USB_VALID        (1<<10) /**< PC0 - !GPIO_VDD_USB_VALID */
#define GPIO_5V_HIPOWER_OC    (1<<11) /**< PE10 - !VDD_5V_HIPOWER_OC */
#define GPIO_5V_PERIPH_OC     (1<<12) /**< PE10 - !VDD_5V_PERIPH_OC */

/* This board provides a DMA pool and APIs */
#define BOARD_DMA_ALLOC_POOL_SIZE 5120

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
