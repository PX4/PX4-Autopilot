/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
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
 * @file mavstation_internal.h
 *
 * Mavstation internal definitions
 */

#pragma once

/******************************************************************************
 * Included Files
 ******************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>
#include <stdint.h>

/* these headers are not C++ safe */
#include <stm32.h>

 
/******************************************************************************
 * Definitions
 ******************************************************************************/
/* Configuration **************************************************************/

/******************************************************************************
 * GPIOS
 ******************************************************************************/

/* LEDS  **********************************************************************/

#define GPIO_LED1 (GPIO_OUTPUT|GPIO_CNF_OUTPP|GPIO_MODE_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTB|GPIO_PIN14)
#define GPIO_LED2 (GPIO_OUTPUT|GPIO_CNF_OUTPP|GPIO_MODE_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTB|GPIO_PIN15)
#define GPIO_LED3 (GPIO_OUTPUT|GPIO_CNF_OUTPP|GPIO_MODE_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTB|GPIO_PIN13)

#define GPIO_LED_BLUE   BOARD_GPIO_LED1
#define GPIO_LED_AMBER  BOARD_GPIO_LED2
#define GPIO_LED_SAFETY BOARD_GPIO_LED3

/* Safety switch button *******************************************************/

#define GPIO_BTN_SAFETY (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTB|GPIO_PIN5)

/* Power switch controls ******************************************************/

#define GPIO_SPEKTRUM_PWR_EN (GPIO_OUTPUT|GPIO_CNF_OUTPP|GPIO_MODE_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTC|GPIO_PIN13)

#define GPIO_SERVO_PWR_EN (GPIO_OUTPUT|GPIO_CNF_OUTPP|GPIO_MODE_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTC|GPIO_PIN15)

#define GPIO_SERVO_FAULT_DETECT (GPIO_INPUT|GPIO_CNF_INPULLUP|GPIO_MODE_INPUT|GPIO_PORTB|GPIO_PIN13)


/* Analog inputs **************************************************************/

#define GPIO_ADC_VSERVO (GPIO_INPUT|GPIO_CNF_ANALOGIN|GPIO_MODE_INPUT|GPIO_PORTA|GPIO_PIN4)
/* the same rssi signal goes to both an adc and a timer input */
#define GPIO_ADC_RSSI   (GPIO_INPUT|GPIO_CNF_ANALOGIN|GPIO_MODE_INPUT|GPIO_PORTA|GPIO_PIN5)
/* floating pin */
#define GPIO_TIM_RSSI   (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTA|GPIO_PIN12)

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

/* SBUS pins  *************************************************************/

#define GPIO_SBUS_INPUT   (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTB|GPIO_PIN11)
#define GPIO_SBUS_OUTPUT  (GPIO_OUTPUT|GPIO_CNF_OUTPP|GPIO_MODE_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTB|GPIO_PIN10)
#define GPIO_SBUS_OENABLE (GPIO_OUTPUT|GPIO_CNF_OUTPP|GPIO_MODE_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTB|GPIO_PIN4)

