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
#define GPIO_CAN1_SILENT_S0   /* PB11 */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTB|GPIO_PIN11)

/* CAN termination software control */
#define GPIO_CAN1_TERMINATION /* PB13 */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTB|GPIO_PIN13)
#define GPIO_CAN_TERM                    GPIO_CAN1_TERMINATION

/* Boot config */
#define GPIO_BOOT_CONFIG      /* PC15 */ (GPIO_INPUT|GPIO_PULLUP|GPIO_PORTC|GPIO_PIN15|GPIO_EXTI)

/* LEDs are driven with open drain to support Anode to 5V or 3.3V */
#define GPIO_TIM1_CH1         /* PA8  */ (GPIO_TIM1_CH1_1|GPIO_OPENDRAIN|GPIO_SPEED_2MHz)
#define GPIO_TIM1_CH2         /* PA9  */ (GPIO_TIM1_CH2_1|GPIO_OPENDRAIN|GPIO_SPEED_2MHz)
#define GPIO_TIM1_CH3         /* PA10 */ (GPIO_TIM1_CH3_1|GPIO_OPENDRAIN|GPIO_SPEED_2MHz)

#define BROADCOM_AFBR_S50_S2PI_SPI_BUS 2
#define BROADCOM_AFBR_S50_S2PI_CS   /* PB12 */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTB|GPIO_PIN12)
#define BROADCOM_AFBR_S50_S2PI_IRQ  /* PB4  */ (GPIO_INPUT|GPIO_PULLUP|GPIO_PORTB|GPIO_PIN4|GPIO_EXTI)
#define BROADCOM_AFBR_S50_S2PI_CLK  /* PB10 */  GPIO_SPI2_SCK_1
#define BROADCOM_AFBR_S50_S2PI_MOSI /* PB15 */  GPIO_SPI2_MOSI_1
#define BROADCOM_AFBR_S50_S2PI_MISO /* PB14 */  GPIO_SPI2_MISO_1

#define FLASH_BASED_PARAMS

/* High-resolution timer */
#define HRT_TIMER                    3  /* use timer 3 for the HRT */
#define HRT_TIMER_CHANNEL            4  /* use capture/compare channel 4 */

#define PX4_GPIO_INIT_LIST { \
		GPIO_BOOT_CONFIG,                 \
		GPIO_CAN1_TX,                     \
		GPIO_CAN1_RX,                     \
		GPIO_CAN1_SILENT_S0,              \
		GPIO_CAN1_TERMINATION,            \
	}

__BEGIN_DECLS

#define BOARD_HAS_N_S_RGB_LED       1
#define BOARD_MAX_LEDS              BOARD_HAS_N_S_RGB_LED

#ifndef __ASSEMBLY__

extern void stm32_spiinitialize(void);

#include <px4_platform_common/board_common.h>

#endif /* __ASSEMBLY__ */

__END_DECLS
