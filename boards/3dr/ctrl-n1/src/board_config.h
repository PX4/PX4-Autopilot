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
 * Board internal definitions
 */

#pragma once

#include <px4_platform_common/px4_config.h>
#include <nuttx/compiler.h>
#include <stdint.h>
#include <stm32_gpio.h>

/* LEDs */
#define GPIO_nLED_BL /* PD10 */ (GPIO_OUTPUT|GPIO_OPENDRAIN|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTD|GPIO_PIN10)
#define GPIO_RGB_S   /* PB1  */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTB|GPIO_PIN1)
#define BOARD_SRGBLED_PORT    STM32_GPIOB_ODR
#define BOARD_SRGBLED_BIT     1

//#define BOARD_HAS_CONTROL_STATUS_LEDS      1
//#define BOARD_OVERLOAD_LED     LED_RED
//#define BOARD_ARMED_STATE_LED  LED_BLUE

/* ADC channels */
#define PX4_ADC_GPIO  \
	/* PA4 */  GPIO_ADC12_INP18,  \
	/* PC0 */  GPIO_ADC123_INP10,  \
	/* PC5 */  GPIO_ADC12_INP8

/* Define Channel numbers must match above GPIO pins */
#define ADC_BATTERY_VOLTAGE_CHANNEL       18 /* PA2 BATT_VOLT_SENS */
#define ADC_BATTERY_CURRENT_CHANNEL       10 /* PC0 BATT_CURRENT_SENS */
#define ADC_SCALED_V5_CHANNEL             8  /* PC5 VDD_5V_SENS */

#define ADC_CHANNELS \
	((1 << ADC_BATTERY_VOLTAGE_CHANNEL)       | \
	 (1 << ADC_BATTERY_CURRENT_CHANNEL)       | \
	 (1 << ADC_SCALED_V5_CHANNEL))


/* HW has to large of R termination on ADC todo:change when HW value is chosen */
#define BOARD_ADC_OPEN_CIRCUIT_V     (5.6f)

/* PWM */
#define DIRECT_PWM_OUTPUT_CHANNELS  12

/* Tone alarm output */
#define TONE_ALARM_TIMER         17  /* timer 17 */
#define TONE_ALARM_CHANNEL        1  /* PB9 TIM17_CH1 */

#define GPIO_BUZZER_1           /* PB9 */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTB|GPIO_PIN9)

#define GPIO_TONE_ALARM_IDLE    GPIO_BUZZER_1
#define GPIO_TONE_ALARM         GPIO_TIM17_CH1OUT_1

/* NO USB OTG FS VBUS */
#define BOARD_USB_VBUS_SENSE_DISABLED 1
#define GPIO_OTGFS_VBUS 0

/* High-resolution timer */
#define HRT_TIMER               12  /* use timer3 for the HRT */
#define HRT_TIMER_CHANNEL       1 /* use capture/compare channel 1 */

/* RC Serial port */
//Uncomment to accept "legacy" protocols
//#define RC_SERIAL_PORT          "/dev/ttyS5"

/* By Providing BOARD_ADC_USB_CONNECTED (using the px4_arch abstraction)
 * this board support the ADC system_power interface, and therefore
 * provides the true logic GPIO BOARD_ADC_xxxx macros.
 */
#define BOARD_ADC_USB_CONNECTED (px4_arch_gpioread(ADC_SCALED_V5_CHANNEL))
#define BOARD_ADC_USB_VALID     BOARD_ADC_USB_CONNECTED
#define BOARD_ADC_SERVO_VALID   (1) /* never powers off the Servo rail */

/* This board provides a DMA pool and APIs */
#define BOARD_DMA_ALLOC_POOL_SIZE 5120

/* This board has 6 DMA channels available for bidirectional dshot */
#define BOARD_DMA_NUM_DSHOT_CHANNELS 6

/* This board provides the board_on_reset interface */
#define BOARD_HAS_ON_RESET 1

#define BOARD_HAS_STATIC_MANIFEST 1

#define BOARD_NUM_IO_TIMERS 3

#define BOARD_ENABLE_CONSOLE_BUFFER

#define PX4_GPIO_INIT_LIST { \
		PX4_ADC_GPIO,                     \
		GPIO_CAN1_TX,                     \
		GPIO_CAN1_RX,                     \
		GPIO_CAN2_TX,                     \
		GPIO_CAN2_RX,                     \
		GPIO_TONE_ALARM_IDLE,             \
	}

__BEGIN_DECLS


#if defined(USE_S_RGB_LED_DMA)
#define BOARD_HAS_N_S_RGB_LED       1
#define BOARD_MAX_LEDS              BOARD_HAS_N_S_RGB_LED
#define S_RGB_LED_DMA               DMAMAP_DMA12_TIM3CH4_0
#define S_RGB_LED_TIMER             3   /* timer 3    */
#define S_RGB_LED_CHANNEL           4   /* channel 4  */
#define S_RGB_LED_TIM_GPIO          GPIO_TIM3_CH4OUT_1
#endif

#ifndef __ASSEMBLY__

extern void stm32_spiinitialize(void);
extern void board_peripheral_reset(int ms);

#include <px4_platform_common/board_common.h>
#endif /* __ASSEMBLY__ */
__END_DECLS
