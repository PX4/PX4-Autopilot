/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
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
 * board internal definitions
 */

#pragma once

#include <px4_platform_common/px4_config.h>
#include <nuttx/compiler.h>
#include <stdint.h>

/* CAN  Silent mode control */
#define GPIO_CAN1_SILENT_S0   /* PC14  */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTC|GPIO_PIN14)

/* CAN termination software control */
#define GPIO_CAN1_TERMINATION /* PC15 */  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTC|GPIO_PIN15)
#define GPIO_CAN_TERM                    GPIO_CAN1_TERMINATION

/* Boot config */
#define GPIO_BOOT_CONFIG      /* PH1 */ (GPIO_INPUT|GPIO_PULLUP|GPIO_PORTH|GPIO_PIN1|GPIO_EXTI)

/* ICM42688p FSYNC */
#define GPIO_42688P_FSYNC     /* PB8  */  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTB|GPIO_PIN8)

/* LEDs are driven with open drain to support Anode to 5V or 3.3V */
#define GPIO_TIM1_CH1         /* PA8  */  (GPIO_TIM1_CH1_1|GPIO_OPENDRAIN|GPIO_SPEED_2MHz)
#define GPIO_TIM1_CH2         /* PA9  */  (GPIO_TIM1_CH2_1|GPIO_OPENDRAIN|GPIO_SPEED_2MHz)
#define GPIO_TIM1_CH3         /* PA10 */  (GPIO_TIM1_CH3_1|GPIO_OPENDRAIN|GPIO_SPEED_2MHz)

/* PWM Outputs */
#define BOARD_NUM_IO_TIMERS         3
#define DIRECT_PWM_OUTPUT_CHANNELS  8

#define GPIO_TIM2_CH1_RESET   /* PA0  */  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTA|GPIO_PIN0)
#define GPIO_TIM2_CH2_RESET   /* PA1  */  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTA|GPIO_PIN1)
#define GPIO_TIM2_CH3_RESET   /* PB10 */  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTB|GPIO_PIN10)
#define GPIO_TIM3_CH1_RESET   /* PB4  */  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTB|GPIO_PIN4)
#define GPIO_TIM3_CH2_RESET   /* PB5  */  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTB|GPIO_PIN5)
#define GPIO_TIM3_CH3_RESET   /* PB0  */  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTB|GPIO_PIN0)
#define GPIO_TIM3_CH4_RESET   /* PB1  */  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTB|GPIO_PIN1)
#define GPIO_TIM4_CH4_RESET   /* PB7  */  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTB|GPIO_PIN7)

#define GPIO_I2C1_SCL_RESET   /* PB6  */  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTB|GPIO_PIN6)
#define GPIO_I2C1_SDA_RESET   /* PB9  */  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTB|GPIO_PIN9)

#define GPIO_USART1_RX_GPIO   /* PB3  */  (GPIO_INPUT|GPIO_PULLUP|GPIO_SPEED_50MHz|GPIO_PORTB|GPIO_PIN3)
#define GPIO_USART1_TX_GPIO   /* PA15  */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_PULLUP|GPIO_SPEED_50MHz|GPIO_PORTA|GPIO_PIN15)

#define GPIO_USART2_RX_GPIO   /* PA3  */  (GPIO_INPUT|GPIO_PULLUP|GPIO_SPEED_50MHz|GPIO_PORTA|GPIO_PIN3)
#define GPIO_USART2_TX_GPIO   /* PA2  */  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_PULLUP|GPIO_SPEED_50MHz|GPIO_PORTA|GPIO_PIN2)

#define FLASH_BASED_PARAMS

/* High-resolution timer */
#define HRT_TIMER                    8  /* use timer 8 for the HRT */
#define HRT_TIMER_CHANNEL            1  /* use capture/compare channel 1 */

#define PX4_GPIO_INIT_LIST { \
		GPIO_CAN1_SILENT_S0,              \
		GPIO_CAN1_TERMINATION,            \
		GPIO_42688P_FSYNC,                \
		GPIO_CAN1_TX,                     \
		GPIO_CAN1_RX,                     \
		GPIO_I2C1_SCL_RESET,              \
		GPIO_I2C1_SDA_RESET,              \
	}

__BEGIN_DECLS

#define BOARD_HAS_N_S_RGB_LED       1
#define BOARD_MAX_LEDS              BOARD_HAS_N_S_RGB_LED

#ifndef __ASSEMBLY__

extern void stm32_spiinitialize(void);

#include <px4_platform_common/board_common.h>

#endif /* __ASSEMBLY__ */

__END_DECLS
