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
 * @file Board initialisation and configuration data.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>
#include <stdio.h>
#include <debug.h>
#include <errno.h>

#include <arch/board/board.h>
#include <arch/board/up_boardinitialize.h>
#include <arch/board/up_hrt.h>
#include <arch/board/drv_pwm_servo.h>
#include <arch/board/drv_gpio.h>

#include "chip.h"
#include "up_internal.h"
#include "up_arch.h"

#include "stm32_internal.h"
#include "stm32_gpio.h"
#include "stm32_tim.h"

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

/* Debug ********************************************************************/

/* Configuration ************************************************************/

#if CONFIG_PWM_SERVO
  /*
   * Servo configuration for the PX4IO board.
   */
  static const struct pwm_servo_config servo_config = {
    .update_rate = 50,
    .timers = {
      {
        .base = STM32_TIM2_BASE,
        .clock_register = STM32_RCC_APB1ENR,
        .clock_bit = RCC_APB1ENR_TIM2EN,
        .clock_freq = STM32_APB1_TIM2_CLKIN
      },
      {
        .base = STM32_TIM3_BASE,
        .clock_register = STM32_RCC_APB1ENR,
        .clock_bit = RCC_APB1ENR_TIM3EN,
        .clock_freq = STM32_APB1_TIM3_CLKIN
      },
      {
        .base = STM32_TIM4_BASE,
        .clock_register = STM32_RCC_APB1ENR,
        .clock_bit = RCC_APB1ENR_TIM4EN,
        .clock_freq = STM32_APB1_TIM4_CLKIN
      },
    },
    .channels = {
      {
        .gpio = GPIO_TIM2_CH1OUT,
        .timer_index = 0,
        .timer_channel = 1,
        .default_value = 1000,
      },
      {
        .gpio = GPIO_TIM2_CH2OUT,
        .timer_index = 0,
        .timer_channel = 2,
        .default_value = 1000,
      },
      {
        .gpio = GPIO_TIM4_CH3OUT,
        .timer_index = 2,
        .timer_channel = 3,
        .default_value = 1000,
      },
      {
        .gpio = GPIO_TIM4_CH4OUT,
        .timer_index = 2,
        .timer_channel = 4,
        .default_value = 1000,
      },
      {
        .gpio = GPIO_TIM3_CH1OUT,
        .timer_index = 1,
        .timer_channel = 1,
        .default_value = 1000,
      },
      {
        .gpio = GPIO_TIM3_CH2OUT,
        .timer_index = 1,
        .timer_channel = 2,
        .default_value = 1000,
      },
      {
        .gpio = GPIO_TIM3_CH3OUT,
        .timer_index = 1,
        .timer_channel = 3,
        .default_value = 1000,
      },
      {
        .gpio = GPIO_TIM3_CH4OUT,
        .timer_index = 1,
        .timer_channel = 4,
        .default_value = 1000,
      },
    }
  };
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nsh_archinitialize
 *
 * Description:
 *   Perform architecture specific initialization
 *
 ****************************************************************************/

int up_boardinitialize()
{
	/* configure the high-resolution time/callout interface */
#ifdef CONFIG_HRT_TIMER
	hrt_init(CONFIG_HRT_TIMER);
#endif

	/* configure the PWM servo driver */
#if CONFIG_PWM_SERVO
	pwm_servo_init(&servo_config);
#endif

  /* configure the GPIO driver */
  gpio_drv_init();

  return OK;
}
