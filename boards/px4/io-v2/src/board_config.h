/****************************************************************************
 *
 *   Copyright (c) 2013, 2014 PX4 Development Team. All rights reserved.
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
 * PX4IOV2 internal definitions
 */

#pragma once

/******************************************************************************
 * Included Files
 ******************************************************************************/

#include <px4_platform_common/px4_config.h>
#include <nuttx/compiler.h>
#include <stdint.h>

/******************************************************************************
 * Definitions
 ******************************************************************************/
/* Configuration **************************************************************/

/******************************************************************************
 * Serial
 ******************************************************************************/
#define PX4FMU_SERIAL_BASE	STM32_USART2_BASE
#define PX4FMU_SERIAL_VECTOR	STM32_IRQ_USART2
#define PX4FMU_SERIAL_TX_GPIO	GPIO_USART2_TX
#define PX4FMU_SERIAL_RX_GPIO	GPIO_USART2_RX
#define PX4FMU_SERIAL_TX_DMA	DMACHAN_USART2_TX
#define PX4FMU_SERIAL_RX_DMA	DMACHAN_USART2_RX
#define PX4FMU_SERIAL_CLOCK	STM32_PCLK1_FREQUENCY
#define PX4FMU_SERIAL_BITRATE	1500000

/******************************************************************************
 * GPIOS
 ******************************************************************************/

/* LEDS  **********************************************************************/

#define GPIO_LED_BLUE   (GPIO_OUTPUT|GPIO_CNF_OUTOD|GPIO_MODE_50MHz|GPIO_OUTPUT_SET|GPIO_PORTB|GPIO_PIN14) // IO-LED_BLUE or IMU_TEMPERATURE_CONTROL on Pixhawk 2.1
#define GPIO_LED_AMBER  (GPIO_OUTPUT|GPIO_CNF_OUTOD|GPIO_MODE_50MHz|GPIO_OUTPUT_SET|GPIO_PORTB|GPIO_PIN15) // IO-LED_AMBER
#define GPIO_LED_SAFETY (GPIO_OUTPUT|GPIO_CNF_OUTOD|GPIO_MODE_50MHz|GPIO_OUTPUT_SET|GPIO_PORTB|GPIO_PIN13) // IO-LED_SAFETY
#define GPIO_LED_GREEN  (GPIO_OUTPUT|GPIO_CNF_OUTOD|GPIO_MODE_50MHz|GPIO_OUTPUT_SET|GPIO_PORTA|GPIO_PIN11) // IO-LED_POWER_BREATHING

#define GPIO_PC14 (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTC|GPIO_PIN14)
#define GPIO_PC15 (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTC|GPIO_PIN15)

/*  PixHawk 1:
 *      PC14 Floating
 *      PC15 Floating
 *
 *  PixHawk 2:
 *      PC14 3.3v
 *      PC15 GND
 */
#define GPIO_SENSE_PC14_DN (GPIO_INPUT|GPIO_CNF_INPULLDWN|GPIO_MODE_INPUT|GPIO_PORTC|GPIO_PIN14)
#define GPIO_SENSE_PC15_UP (GPIO_INPUT|GPIO_CNF_INPULLUP|GPIO_MODE_INPUT|GPIO_PORTC|GPIO_PIN15)
# define SENSE_PH1 0b10 /* Floating pulled as set */
# define SENSE_PH2 0b01 /* Driven as tied */

#define SENSE_PIXHAWK2() (((stm32_gpioread(GPIO_SENSE_PC15_UP) << 1 | stm32_gpioread(GPIO_SENSE_PC14_DN)) == SENSE_PH2) ? 1 : 0)

#define LED_BLUE(on_true)   (SENSE_PIXHAWK2() ? (void)0 : stm32_gpiowrite(GPIO_LED_BLUE, !(on_true)))
#define LED_AMBER(on_true)  stm32_gpiowrite(GPIO_LED_AMBER, !(on_true))
#define LED_SAFETY(on_true) stm32_gpiowrite(GPIO_LED_SAFETY, !(on_true))
#define LED_GREEN(on_true)   stm32_gpiowrite(GPIO_LED_GREEN, (on_true))


/* HEATER */
#define GPIO_HEATER_OUTPUT   /* PB14 */ (GPIO_OUTPUT|GPIO_CNF_OUTPP|GPIO_MODE_50MHz|GPIO_OUTPUT_SET|GPIO_PORTB|GPIO_PIN14)
#define HEATER_OUTPUT_EN(on_true)       (SENSE_PIXHAWK2() ? stm32_gpiowrite(GPIO_HEATER_OUTPUT, !(on_true)) : (void)0)


#define GPIO_USART1_RX_SPEKTRUM		(GPIO_OUTPUT|GPIO_CNF_OUTPP|GPIO_MODE_50MHz|GPIO_OUTPUT_SET|GPIO_PORTA|GPIO_PIN10)

/* Safety switch button *******************************************************/

#define GPIO_BTN_SAFETY (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTB|GPIO_PIN5)

/* Power switch controls ******************************************************/

#define GPIO_SPEKTRUM_PWR_EN         (GPIO_OUTPUT|GPIO_CNF_OUTPP|GPIO_MODE_50MHz|GPIO_OUTPUT_SET|GPIO_PORTC|GPIO_PIN13)
#define SPEKTRUM_POWER(_on_true)     px4_arch_gpiowrite(GPIO_SPEKTRUM_PWR_EN, (_on_true))

#define SPEKTRUM_OUT(_one_true)      px4_arch_gpiowrite(GPIO_USART1_RX_SPEKTRUM, (_one_true))
#define SPEKTRUM_RX_AS_UART()        px4_arch_configgpio(GPIO_USART1_RX)
#define SPEKTRUM_RX_AS_GPIO_OUTPUT() px4_arch_configgpio(GPIO_USART1_RX_SPEKTRUM)

#define GPIO_SERVO_FAULT_DETECT (GPIO_INPUT|GPIO_CNF_INPULLUP|GPIO_MODE_INPUT|GPIO_PORTA|GPIO_PIN15)

/* Analog inputs **************************************************************/

#define GPIO_ADC_VSERVO (GPIO_INPUT|GPIO_CNF_ANALOGIN|GPIO_MODE_INPUT|GPIO_PORTA|GPIO_PIN4)

/* the same rssi signal goes to both an adc and a timer input */
#define GPIO_ADC_RSSI   (GPIO_INPUT|GPIO_CNF_ANALOGIN|GPIO_MODE_INPUT|GPIO_PORTA|GPIO_PIN5)
#define GPIO_TIM_RSSI   (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTA|GPIO_PIN12)

#define SYSTEM_ADC_BASE STM32_ADC1_BASE

/* PWM pins  **************************************************************/

#define GPIO_PPM (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTA|GPIO_PIN8)

#define GPIO_PWM1 (GPIO_OUTPUT|GPIO_CNF_OUTPP|GPIO_MODE_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTA|GPIO_PIN0)
#define GPIO_PWM2 (GPIO_OUTPUT|GPIO_CNF_OUTPP|GPIO_MODE_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTA|GPIO_PIN1)
#define GPIO_PWM3 (GPIO_OUTPUT|GPIO_CNF_OUTPP|GPIO_MODE_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTB|GPIO_PIN8)
#define GPIO_PWM4 (GPIO_OUTPUT|GPIO_CNF_OUTPP|GPIO_MODE_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTB|GPIO_PIN9)
#define GPIO_PWM5 (GPIO_OUTPUT|GPIO_CNF_OUTPP|GPIO_MODE_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTA|GPIO_PIN6)
#define GPIO_PWM6 (GPIO_OUTPUT|GPIO_CNF_OUTPP|GPIO_MODE_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTA|GPIO_PIN7)
#define GPIO_PWM7 (GPIO_OUTPUT|GPIO_CNF_OUTPP|GPIO_MODE_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTB|GPIO_PIN0)
#define GPIO_PWM8 (GPIO_OUTPUT|GPIO_CNF_OUTPP|GPIO_MODE_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTB|GPIO_PIN1)

#define DIRECT_PWM_OUTPUT_CHANNELS 8
#define BOARD_HAS_NO_CAPTURE

/* SBUS pins  *************************************************************/

/* XXX these should be UART pins */
#define GPIO_SBUS_INPUT   (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTB|GPIO_PIN11)
#define GPIO_SBUS_OUTPUT  (GPIO_OUTPUT|GPIO_CNF_OUTPP|GPIO_MODE_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTB|GPIO_PIN10)
#define GPIO_SBUS_OENABLE (GPIO_OUTPUT|GPIO_CNF_OUTPP|GPIO_MODE_50MHz|GPIO_OUTPUT_SET|GPIO_PORTB|GPIO_PIN4)

/*
 * High-resolution timer
 */
#define HRT_TIMER		1	/* use timer1 for the HRT */
#define HRT_TIMER_CHANNEL	2	/* use capture/compare channel 2 */
#define HRT_PPM_CHANNEL		1	/* use capture/compare channel 1 PA8 */
#define GPIO_PPM_IN		(GPIO_ALT|GPIO_CNF_INPULLUP|GPIO_PORTA|GPIO_PIN8)

#define BOARD_NUM_IO_TIMERS 3

#include <px4_platform_common/board_common.h>
