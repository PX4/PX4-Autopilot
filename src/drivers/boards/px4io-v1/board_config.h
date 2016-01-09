/****************************************************************************
 *
 *   Copyright (c) 2012-2014 PX4 Development Team. All rights reserved.
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
 * PX4IO hardware definitions.
 */

#pragma once

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <px4_config.h>
#include <nuttx/compiler.h>
#include <stdint.h>

/* these headers are not C++ safe */
#include <stm32.h>
#include <arch/board/board.h>

/************************************************************************************
 * Definitions
 ************************************************************************************/

/* PX4IO GPIOs **********************************************************************/
/* LEDs */

#define GPIO_LED1       (GPIO_OUTPUT|GPIO_CNF_OUTOD|GPIO_MODE_50MHz|\
			 GPIO_OUTPUT_CLEAR|GPIO_PORTB|GPIO_PIN14)
#define GPIO_LED2       (GPIO_OUTPUT|GPIO_CNF_OUTOD|GPIO_MODE_50MHz|\
			 GPIO_OUTPUT_CLEAR|GPIO_PORTB|GPIO_PIN15)
#define GPIO_LED3       (GPIO_OUTPUT|GPIO_CNF_OUTOD|GPIO_MODE_50MHz|\
			 GPIO_OUTPUT_CLEAR|GPIO_PORTB|GPIO_PIN10)

#define GPIO_USART1_RX_SPEKTRUM		(GPIO_OUTPUT | GPIO_CNF_OUTPP | GPIO_MODE_50MHz | GPIO_OUTPUT_SET | GPIO_PORTA | GPIO_PIN10)

/* Safety switch button *************************************************************/

#define GPIO_BTN_SAFETY (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTB|GPIO_PIN5)

/* Power switch controls ************************************************************/

#define GPIO_ACC1_PWR_EN  (GPIO_OUTPUT|GPIO_CNF_OUTPP|GPIO_MODE_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTC|GPIO_PIN13)
#define GPIO_ACC2_PWR_EN  (GPIO_OUTPUT|GPIO_CNF_OUTPP|GPIO_MODE_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTC|GPIO_PIN14)
#define GPIO_SERVO_PWR_EN (GPIO_OUTPUT|GPIO_CNF_OUTPP|GPIO_MODE_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTC|GPIO_PIN15)

#define GPIO_ACC_OC_DETECT   (GPIO_INPUT|GPIO_CNF_INPULLUP|GPIO_MODE_INPUT|GPIO_PORTB|GPIO_PIN12)
#define GPIO_SERVO_OC_DETECT (GPIO_INPUT|GPIO_CNF_INPULLUP|GPIO_MODE_INPUT|GPIO_PORTB|GPIO_PIN13)

#define GPIO_RELAY1_EN (GPIO_OUTPUT|GPIO_CNF_OUTPP|GPIO_MODE_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTA|GPIO_PIN12)
#define GPIO_RELAY2_EN (GPIO_OUTPUT|GPIO_CNF_OUTPP|GPIO_MODE_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTA|GPIO_PIN11)

#define GPIO_SPEKTRUM_PWR_EN GPIO_RELAY1_EN
#define POWER_SPEKTRUM(_s)		stm32_gpiowrite(GPIO_RELAY1_EN, (_s))

#define SPEKTRUM_RX_HIGH(_s)	stm32_gpiowrite(GPIO_USART1_RX_SPEKTRUM, (_s))
#define SPEKTRUM_RX_AS_UART()		stm32_configgpio(GPIO_USART1_RX)
#define SPEKTRUM_RX_AS_GPIO()		stm32_configgpio(GPIO_USART1_RX_SPEKTRUM)

/* Analog inputs ********************************************************************/

#define GPIO_ADC_VBATT	(GPIO_INPUT|GPIO_CNF_ANALOGIN|GPIO_MODE_INPUT|GPIO_PORTA|GPIO_PIN4)
#define GPIO_ADC_IN5	(GPIO_INPUT|GPIO_CNF_ANALOGIN|GPIO_MODE_INPUT|GPIO_PORTA|GPIO_PIN5)

/*
 * High-resolution timer
 */
#define HRT_TIMER		1	/* use timer1 for the HRT */
#define HRT_TIMER_CHANNEL	2	/* use capture/compare channel 2 */
#define HRT_PPM_CHANNEL		1	/* use capture/compare channel 1 */
#define GPIO_PPM_IN		(GPIO_ALT|GPIO_CNF_INPULLUP|GPIO_PORTE|GPIO_PIN9)
