/****************************************************************************
 *
 *   Copyright (c) 2012-2016 PX4 Development Team. All rights reserved.
 *         Author: David Sidrane <david_s5@nscdg.com>
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
 * TAP_V1 internal definitions
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

/* PX4FMU GPIOs ***********************************************************************************/
/* LEDs
 *
 * PC4     BLUE_LED                  D4 Blue LED cathode
 * PC5     RED_LED                   D5 Red LED cathode
*/

#define GPIO_LED1		(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTC|GPIO_PIN4)
#define GPIO_LED2		(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTC|GPIO_PIN5)
#define GPIO_BLUE_LED GPIO_LED1
#define GPIO_RED_LED  GPIO_LED2
/*
 * SPI
 *
 * Peripheral   Port     Signal Name               CONN
 * SPI2_NSS       PB12    SD_SPI2_NSS               SD-2 CS
 * SPI2_SCK       PB13    SD_SPI2_SCK               SD-5 CLK
 * SPI2_MISO      PB14    SD_SPI2_MISO              SD-7 D0
 * SPI2_MOSI      PB15    SD_SPI2_MOSI              SD-3 DI
 *
 *                PC2     SD_SW                     SD-9 SW
 *
 */
#define GPIO_SPI_CS_SDCARD	(GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTB|GPIO_PIN12)
#define GPIO_SPI_SD_SW	    (GPIO_INPUT|GPIO_PULLUP|GPIO_PORTC|GPIO_PIN2)


/*
 * I2C busses
 *
 * Peripheral   Port     Signal Name               CONN
 * I2C1_SDA     PB9     I2C1_SDA                  J2-4,9,16,21 mpu6050, U4 MS6507
 * I2C1_SDL     PB8     I2C1_SCL                  J2-3,10,15,22 mpu6050, U4 MS6507
 *
 * I2C2_SDA     PB11    Sonar Echo/I2C_SDA        JP2-31,32
 * I2C2_SDL     PB10    Sonar Trig/I2C_SCL        JP2-29,30
 *
 * I2C3_SDA     PC9     COMPASS_I2C3_SDA          JP1-27,28
 * I2C3_SDL     PA8     COMPASS_I2C3_SCL          JP1-25,26
 *
 */
#define PX4_I2C_BUS_ONBOARD    1
#define PX4_I2C_BUS_SONAR      2
#define PX4_I2C_BUS_EXPANSION  3

#define PX4_I2C_OBDEV_HMC5883	0x1e

#define PX4_I2C_BUS_ONBOARD_HZ      400000
#define PX4_I2C_BUS_SONAR_HZ        400000
#define PX4_I2C_BUS_EXPANSION_HZ    400000
/*
 * Devices on the onboard bus.
 *
 * Note that these are unshifted addresses (not includinf R/W).
 */

/*
 * The slave address of the MPU-60X0 is b110100X which is 7 bits long.
 * The LSB bit of the 7 bit address is determined by the logic level
 * on pin AD0. This allows two MPU-60X0s to be connected to the same I2C bus.
 * When used in this configuration, the address of the one of the devices
 * should be b1101000 (pin AD0 is logic low) and the address of the other
 * should be b1101001 (pin AD0 is logic high).
 */
#define PX4_I2C_MPU6050_ADDR 0x68

/*
 * ADC channels
 *
 * These are the channel numbers of the ADCs of the microcontroller that can be used by the Px4 Firmware in the adc driver
 * PC0     VOLTAGE                   JP2-13,14          - 1.84 @16.66  1.67 @15.12 Scale 0.1105
 *
 */
#define ADC_CHANNELS (1 << 10)
/* todo:Revisit - cannnot tell from schematic - some could be ADC */

// ADC defines to be used in sensors.cpp to read from a particular channel
#define ADC_BATTERY_VOLTAGE_CHANNEL	10
#define ADC_BATTERY_CURRENT_CHANNEL	((uint8_t)(-1))

/* Define Battery 1 Voltage Divider
 * Use Default for A per V
 */

#define BOARD_BATTERY1_V_DIV (9.0f)

/* No User GPIOs
 *
 * TIM3_CH1     PA6     LED_R                     JP2-23,24
 * TIM3_CH2     PA7     LED_G                     JP2-25,26
 * TIM3_CH3     PB0     LED_B                     JP2-27,28
 * TIM3_CH4     PB1     nPWM_1 AUX1(Landing Gear) JP1-21,22
 *
 * I2C2_SDA     PB11    Sonar Echo/I2C_SDA        JP2-31,32
 * I2C2_SDL     PB10    Sonar Trig/I2C_SCL        JP2-29,30
 */

/*
 * Tone alarm output
 */
/* todo:Revisit - cannnot tell from schematic - one could be tone alarm*/
#define TONE_ALARM_TIMER        8   /* timer 8 */
#define TONE_ALARM_CHANNEL      3   /* channel 3 */
#define GPIO_TONE_ALARM_IDLE    (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTC|GPIO_PIN8)
#define GPIO_TONE_ALARM         (GPIO_ALT|GPIO_AF2|GPIO_SPEED_2MHz|GPIO_FLOAT|GPIO_PUSHPULL|GPIO_PORTC|GPIO_PIN8)

/*
 * PWM
 *
 * Four PWM outputs can be configured on pins
 *
 *
 * Peripheral   Port     Signal Name               CONN
 * TIM3_CH1     PA6     LED_R                     JP2-23,24
 * TIM3_CH2     PA7     LED_G                     JP2-25,26
 * TIM3_CH3     PB0     LED_B                     JP2-27,28
 * TIM3_CH4     PB1     nPWM_1 AUX1(Landing Gear) JP1-21,22
 *
 */
#define GPIO_TIM3_CH1OUT	GPIO_TIM3_CH1OUT_1
#define GPIO_TIM3_CH2OUT	GPIO_TIM3_CH2OUT_1
#define GPIO_TIM3_CH3OUT	GPIO_TIM3_CH3OUT_1
#define GPIO_TIM3_CH4OUT	GPIO_TIM3_CH4OUT_1
#define DIRECT_PWM_OUTPUT_CHANNELS	4

#define GPIO_TIM3_CH1IN		GPIO_TIM3_CH1IN_1
#define GPIO_TIM3_CH2IN		GPIO_TIM3_CH2IN_1
#define GPIO_TIM3_CH3IN		GPIO_TIM3_CH3IN_1
#define GPIO_TIM3_CH4IN		GPIO_TIM3_CH4IN_1
#define DIRECT_INPUT_TIMER_CHANNELS  4

#define BOARD_HAS_LED_PWM
#define BOARD_HAS_SHARED_PWM_TIMERS
#define LED_TIM3_CH1OUT  GPIO_TIM3_CH1OUT
#define LED_TIM3_CH2OUT  GPIO_TIM3_CH2OUT
#define LED_TIM3_CH3OUT  GPIO_TIM3_CH3OUT

#define BOARD_PWM_DRIVE_ACTIVE_LOW 1


/* USB OTG FS
 *
 * PA9  OTG_FS_VBUS VBUS sensing
 */
#define GPIO_OTGFS_VBUS (GPIO_INPUT|GPIO_FLOAT|GPIO_PORTA|GPIO_PIN9)

#define RC_SERIAL_PORT		"/dev/ttyS5"
#define INVERT_RC_INPUT(_invert_true)		while(0)

/* High-resolution timer
 */
#define HRT_TIMER           1  /* use timer1 for the HRT */
#define HRT_TIMER_CHANNEL   1  /* use capture/compare channel */

#define	BOARD_NAME "TAP_V1"

/* By Providing BOARD_ADC_USB_CONNECTED this board support the ADC
 * system_power interface, and herefore provides the true logic
 * GPIO BOARD_ADC_xxxx macros.
 */
#define BOARD_ADC_USB_CONNECTED (px4_arch_gpioread(GPIO_OTGFS_VBUS))
#define BOARD_ADC_BRICK_VALID   (1)
#define BOARD_ADC_SERVO_VALID   (1)
#define BOARD_ADC_PERIPH_5V_OC  (0)
#define BOARD_ADC_HIPOWER_5V_OC (0)

#define BOARD_HAS_PWM	DIRECT_PWM_OUTPUT_CHANNELS

#define BOARD_HAS_POWER_CONTROL	1

/* This board provides a DMA pool and APIs */

#define BOARD_DMA_ALLOC_POOL_SIZE 5120

#define MS_PWR_BUTTON_DOWN 200
#define KEY_AD_GPIO    (GPIO_INPUT|GPIO_PULLDOWN|GPIO_EXTI|GPIO_PORTC|GPIO_PIN1)
#define POWER_ON_GPIO  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTA|GPIO_PIN4)
#define POWER_OFF_GPIO (GPIO_INPUT|GPIO_PULLDOWN|GPIO_PORTA|GPIO_PIN4)

#define GPIO_S0  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTC|GPIO_PIN15)
#define GPIO_S1  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTC|GPIO_PIN14)
#define GPIO_S2  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTC|GPIO_PIN13)

#define GPIO_PCON_RADIO (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTC|GPIO_PIN3)
#define RF_RADIO_POWER_CONTROL(_on_true)	px4_arch_gpiowrite(GPIO_PCON_RADIO, !(_on_true))

#define GPIO_TEMP_CONT (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTB|GPIO_PIN4)
#define TEMP_CONTROL(_on_true)	px4_arch_gpiowrite(GPIO_TEMP_CONT, (_on_true))

#define  FLASH_BASED_PARAMS

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

#define board_spi_reset(ms)
#define board_peripheral_reset(ms)

/****************************************************************************************************
 * Name: stm32_usbinitialize
 *
 * Description:
 *   Called to configure USB IO.
 *
 ****************************************************************************************************/

extern void stm32_usbinitialize(void);

/************************************************************************************
 * Name: board_sdio_initialize
 *
 * Description:
 *   Called to configure SDIO.
 *
 ************************************************************************************/

extern int board_sdio_initialize(void);

/****************************************************************************
 * Name: board_i2c_initialize
 *
 * Description:
 *   Called to set I2C bus frequencies.
 *
 ****************************************************************************/

int board_i2c_initialize(void);

/************************************************************************************
 * Name: board_pwr_init()
 *
 * Description:
 *   Called to configure power control for the tap-v1 board.
 *
 * Input Parameters:
 *   stage- 0 for boot, 1 for board init
 *
 ************************************************************************************/

void board_pwr_init(int stage);

/****************************************************************************
 * Name: board_pwr_button_down
 *
 * Description:
 *   Called to Read the logical state of the power button
 ****************************************************************************/

bool board_pwr_button_down(void);

/****************************************************************************
 * Name: board_pwr
 *
 * Description:
 *   Called to turn on or off the TAP
 *
 ****************************************************************************/

__EXPORT bool px4_board_pwr(bool on_not_off);

#include "../common/board_common.h"

#endif /* __ASSEMBLY__ */

__END_DECLS
