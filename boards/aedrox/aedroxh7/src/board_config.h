/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
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
 * AEDROX AEDROXH7 internal definitions
 */

#pragma once

#include <px4_platform_common/px4_config.h>
#include <nuttx/compiler.h>
#include <stdint.h>

#include <stm32_gpio.h>

/* Configuration ************************************************************/

#define BOARD_HAS_USB_VALID             1
#define BOARD_HAS_NBAT_V                1
#define BOARD_HAS_NBAT_I                1

/* LEDs — active high (driven by transistor switch).
 * Green (status / LED4) on PE5, Blue (notify / LED1) on PE2.
 * Initial state low = LED off.
 *
 * Prototype boards: the blue LED was on PC13 on early prototype hardware
 * (RevB and earlier). Patch GPIO_LED_BLUE to PORTC/PIN13 if running on those.
 */
#define GPIO_LED_GREEN   /* PE5  */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTE|GPIO_PIN5)
#define GPIO_LED_BLUE    /* PE2  */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTE|GPIO_PIN2)

#define BOARD_HAS_CONTROL_STATUS_LEDS   1
#define BOARD_ARMED_LED                 LED_BLUE   /* PE2 — solid when armed */
#define BOARD_ARMED_STATE_LED           LED_AMBER  /* PE5  — state blinking */
/* No BOARD_OVERLOAD_LED — only two LEDs, both used above. */

/* ADC channels
 *
 * PC0 — battery voltage (ADC1 INP10)
 * PC1 — battery current (ADC1 INP11)
 */
#define ADC1_CH(n)                  (n)

#define PX4_ADC_GPIO  \
	/* PC0 */  GPIO_ADC123_INP10, \
	/* PC1 */  GPIO_ADC123_INP11

#define ADC_BATTERY_VOLTAGE_CHANNEL     /* PC0 */  ADC1_CH(10)
#define ADC_BATTERY_CURRENT_CHANNEL     /* PC1 */  ADC1_CH(11)

#define ADC_CHANNELS \
	((1 << ADC_BATTERY_VOLTAGE_CHANNEL) | \
	 (1 << ADC_BATTERY_CURRENT_CHANNEL))

#define BOARD_ADC_OPEN_CIRCUIT_V        (5.6f)

/* Battery scaling: 11V divider, 40A/V — calibrate per build */
#define BOARD_BATTERY1_V_DIV            (11.0f)
#define BOARD_BATTERY1_A_PER_V          (40.0f)

/* PWM
 * 8 motor outputs on TIM1 (M5-M8) and TIM8 (M1-M4).
 * PA5 addressable LED (TIM2) is left as a pure GPIO for v0.
 */
#define DIRECT_PWM_OUTPUT_CHANNELS   8
#define BOARD_NUM_IO_TIMERS          2

/* Tone alarm — GPIO mode on PA7. The board uses a transistor low-side switch
 * (NPN: drive HIGH = base on = buzzer pulled to GND), which is the typical
 * AEDROX wiring. Use an active (self-resonating) buzzer for this design.
 */
#define GPIO_TONE_ALARM_IDLE    /* PA7 */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTA|GPIO_PIN7)
#define GPIO_TONE_ALARM_GPIO    /* PA7 */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTA|GPIO_PIN7)

/* USB OTG FS — VBUS sensing on PC15 (non-standard; board ties VBUS via divider to PC15). */
#define GPIO_OTGFS_VBUS         /* PC15 */ (GPIO_INPUT|GPIO_PULLDOWN|GPIO_SPEED_100MHz|GPIO_PORTC|GPIO_PIN15)

/* High-resolution timer */
#define HRT_TIMER               4  /* use TIM4 for HRT */
#define HRT_TIMER_CHANNEL       1  /* capture/compare channel 1 */

/*
 * Serial port mapping (NuttX order of enabled UARTs):
 *   USART1 -> /dev/ttyS0  TEL1      PA9/PA10
 *   USART2 -> /dev/ttyS1  GPS1      PD5/PD6
 *   USART3 -> /dev/ttyS2  RC in     PD8/PD9
 *   UART4  -> /dev/ttyS3  TEL2      PD1/PD0
 *   UART7  -> /dev/ttyS4  ESC telem PE8/PE7 (NODMA)
 *   UART8 (console)       DJI/MSP / debug console  PE1/PE0
 */
#define RC_SERIAL_PORT          "/dev/ttyS2"
#define BOARD_SUPPORTS_RC_SERIAL_PORT_OUTPUT

/* User-controlled GPIOs */
#define GPIO_VTX_ON             /* PB1  */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTB|GPIO_PIN1)
#define GPIO_CAM_SWITCH         /* PD15 */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTD|GPIO_PIN15)
#define GPIO_PINIO1             /* PA2  */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTA|GPIO_PIN2)
#define GPIO_PINIO2             /* PA3  */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTA|GPIO_PIN3)
#define GPIO_CAN1_SILENT        /* PD12 */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTD|GPIO_PIN12)

/* By providing BOARD_ADC_USB_CONNECTED (using the px4_arch abstraction) this
 * board supports the ADC system_power interface, and therefore provides the
 * true logic GPIO BOARD_ADC_xxxx macros.
 */
#define BOARD_ADC_USB_CONNECTED (px4_arch_gpioread(GPIO_OTGFS_VBUS))

/* Board never powers off the Servo rail */
#define BOARD_ADC_SERVO_VALID     (1)
#define BOARD_ADC_BRICK1_VALID    (1)

/* This board provides a DMA pool and APIs */
#define BOARD_DMA_ALLOC_POOL_SIZE 5120

/* This board provides the board_on_reset interface */
#define BOARD_HAS_ON_RESET        1

#define PX4_GPIO_INIT_LIST { \
		PX4_ADC_GPIO,          \
		GPIO_LED_GREEN,        \
		GPIO_LED_BLUE,         \
		GPIO_TONE_ALARM_IDLE,  \
		GPIO_VTX_ON,           \
		GPIO_CAM_SWITCH,       \
		GPIO_PINIO1,           \
		GPIO_PINIO2,           \
		GPIO_CAN1_SILENT,      \
		GPIO_CAN1_TX,          \
		GPIO_CAN1_RX,          \
	}

#define BOARD_ENABLE_CONSOLE_BUFFER

#define FLASH_BASED_PARAMS

__BEGIN_DECLS

#ifndef __ASSEMBLY__

extern void stm32_spiinitialize(void);
extern void stm32_usbinitialize(void);
extern void board_peripheral_reset(int ms);

#include <px4_platform_common/board_common.h>

#endif /* __ASSEMBLY__ */

__END_DECLS
