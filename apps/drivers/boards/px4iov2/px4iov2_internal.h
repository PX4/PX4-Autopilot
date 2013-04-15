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
 * @file px4iov2_internal.h
 *
 * PX4IOV2 internal definitions
 */

#pragma once

/****************************************************************************************************
 * Included Files
 ****************************************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>
#include <stdint.h>

__BEGIN_DECLS

/* these headers are not C++ safe */
#include <stm32_internal.h>

 
/****************************************************************************************************
 * Definitions
 ****************************************************************************************************/
/* Configuration ************************************************************************************/

/******************************************************************************
 * GPIOS
 ******************************************************************************/

#define BOARD_GPIO_OUTPUT(pin) (GPIO_OUTPUT|GPIO_CNF_OUTPP|GPIO_MODE_50MHz|\
                                GPIO_OUTPUT_CLEAR|(pin))
#define BOARD_GPIO_INPUT_FLOAT(pin)  (GPIO_INPUT|GPIO_CNF_INFLOAT|\
                                      GPIO_MODE_INPUT|(pin))
#define BOARD_GPIO_INPUT_PUP(pin) (GPIO_INPUT|GPIO_CNF_INPULLUP|\
                                   GPIO_MODE_INPUT|(pin))
#define BOARD_GPIO_INPUT_ANALOG(pin) (GPIO_INPUT|GPIO_CNF_ANALOGIN|\
                                   GPIO_MODE_INPUT|(pin))

/* LEDS  **********************************************************************/

#define BOARD_GPIO_LED1 (GPIO_PORTB|GPIO_PIN14)
#define BOARD_GPIO_LED2 (GPIO_PORTB|GPIO_PIN15)
#define BOARD_GPIO_LED3 (GPIO_PORTB|GPIO_PIN13)

#define BOARD_GPIO_LED_BLUE   BOARD_GPIO_LED1
#define BOARD_GPIO_LED_AMBER  BOARD_GPIO_LED2
#define BOARD_GPIO_LED_SAFETY BOARD_GPIO_LED3

/* Safety switch button *******************************************************/

#define BOARD_GPIO_BTN_SAFETY (GPIO_PORTB|GPIO_PIN5)

/* Power switch controls ******************************************************/

#define BOARD_GPIO_SPEKTRUM_PWR_EN (GPIO_PORTC|GPIO_PIN13)

#define BOARD_GPIO_SERVO_PWR_EN (GPIO_PORTC|GPIO_PIN15)

#define BOARD_GPIO_SERVO_FAULT_DETECT (GPIO_PORTB|GPIO_PIN13)


/* Analog inputs **************************************************************/

#define BOARD_GPIO_ADC_VSERVO (GPIO_PORTA|GPIO_PIN4)
/* the same rssi signal goes to both an adc and a timer input */
#define BOARD_GPIO_ADC_RSSI   (GPIO_PORTA|GPIO_PIN5)
#define BOARD_GPIO_TIM_RSSI   (GPIO_PORTA|GPIO_PIN12)

/* PWM pins  **************************************************************/

#define BOARD_GPIO_PPM (GPIO_PORTA|GPIO_PIN8)

#define BOARD_GPIO_PWM1 (GPIO_PORTA|GPIO_PIN0)
#define BOARD_GPIO_PWM2 (GPIO_PORTA|GPIO_PIN1)
#define BOARD_GPIO_PWM3 (GPIO_PORTB|GPIO_PIN8)
#define BOARD_GPIO_PWM4 (GPIO_PORTB|GPIO_PIN9)
#define BOARD_GPIO_PWM5 (GPIO_PORTA|GPIO_PIN6)
#define BOARD_GPIO_PWM6 (GPIO_PORTA|GPIO_PIN7)
#define BOARD_GPIO_PWM7 (GPIO_PORTB|GPIO_PIN0)
#define BOARD_GPIO_PWM8 (GPIO_PORTB|GPIO_PIN1)

/* SBUS pins  *************************************************************/

#define BOARD_GPIO_SBUS_INPUT   (GPIO_PORTB|GPIO_PIN11)
#define BOARD_GPIO_SBUS_OUTPUT  (GPIO_PORTB|GPIO_PIN10)
#define BOARD_GPIO_SBUS_OENABLE (GPIO_PORTB|GPIO_PIN4)

/****************************************************************************************************
 * Public Types
 ****************************************************************************************************/

/****************************************************************************************************
 * Public data
 ****************************************************************************************************/

#ifndef __ASSEMBLY__

#endif /* __ASSEMBLY__ */

__END_DECLS
