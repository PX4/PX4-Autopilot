/************************************************************************************
 * arch/arm/src/stm32/stm32_pwm.h
 *
 *   Copyright (C) 2011 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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
 * 3. Neither the name NuttX nor the names of its contributors may be
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
 ************************************************************************************/

#ifndef __ARCH_ARM_SRC_STM32_STM32_PWM_H
#define __ARCH_ARM_SRC_STM32_STM32_PWM_H

/* The STM32 does not have dedicated PWM hardware.  Rather, pulsed output control
 * is a capabilitiy of the STM32 timers.  The logic in this file implements the
 * lower half of the standard, NuttX PWM interface using the STM32 timers.  That
 * interface is described in include/nuttx/pwm.h.
 */

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/
/* Configuration ********************************************************************/
/* Timer devices may be used for different purposes.  One special purpose is
 * to generate modulated outputs for such things as motor control.  If CONFIG_STM32_TIMn
 * is defined then the CONFIG_STM32_TIMn_PWM must also be defined to indicate that
 * timer "n" is intended to be used for pulsed output signal generation.
 */

#ifndef CONFIG_STM32_TIM1
#  undef CONFIG_STM32_TIM1_PWM
#endif
#ifndef CONFIG_STM32_TIM2
#  undef CONFIG_STM32_TIM2_PWM
#endif
#ifndef CONFIG_STM32_TIM3
#  undef CONFIG_STM32_TIM3_PWM
#endif
#ifndef CONFIG_STM32_TIM4
#  undef CONFIG_STM32_TIM4_PWM
#endif
#ifndef CONFIG_STM32_TIM5
#  undef CONFIG_STM32_TIM5_PWM
#endif
#ifndef CONFIG_STM32_TIM8
#  undef CONFIG_STM32_TIM8_PWM
#endif
#ifndef CONFIG_STM32_TIM9
#  undef CONFIG_STM32_TIM9_PWM
#endif
#ifndef CONFIG_STM32_TIM10
#  undef CONFIG_STM32_TIM10_PWM
#endif
#ifndef CONFIG_STM32_TIM11
#  undef CONFIG_STM32_TIM11_PWM
#endif
#ifndef CONFIG_STM32_TIM12
#  undef CONFIG_STM32_TIM12_PWM
#endif
#ifndef CONFIG_STM32_TIM13
#  undef CONFIG_STM32_TIM13_PWM
#endif
#ifndef CONFIG_STM32_TIM14
#  undef CONFIG_STM32_TIM14_PWM
#endif

/* The basic timers (timer 6 and 7) are not capable of generating output pulses */

#undef CONFIG_STM32_TIM6_PWM
#undef CONFIG_STM32_TIM7_PWM

/* Check if PWM support for any channel is enabled. */

#if defined(CONFIG_STM32_TIM1_PWM)  || defined(CONFIG_STM32_TIM2_PWM)  || \
    defined(CONFIG_STM32_TIM3_PWM)  || defined(CONFIG_STM32_TIM4_PWM)  || \
    defined(CONFIG_STM32_TIM5_PWM)  || defined(CONFIG_STM32_TIM8_PWM)  || \
    defined(CONFIG_STM32_TIM9_PWM)  || defined(CONFIG_STM32_TIM10_PWM) || \
    defined(CONFIG_STM32_TIM11_PWM) || defined(CONFIG_STM32_TIM12_PWM) || \
    defined(CONFIG_STM32_TIM13_PWM) || defined(CONFIG_STM32_TIM14_PWM)

#include <arch/board/board.h>
#include "chip/stm32_tim.h"

/* For each timer that is enabled for PWM usage, we need the following additional
 * configuration settings:
 *
 * CONFIG_STM32_TIMx_CHANNEL - Specifies the timer output channel {1,..,4}
 * PWM_TIMx_CHn - One of the values defined in chip/stm32*_pinmap.h.  In the case
 *   where there are multiple pin selections, the correct setting must be provided
 *   in the arch/board/board.h file.
 *
 * NOTE: The STM32 timers are each capable of generating different signals on
 * each of the four channels with different duty cycles.  That capability is
 * not supported by this driver:  Only one output channel per timer.
 */

#ifdef CONFIG_STM32_TIM1_PWM
#  if !defined(CONFIG_STM32_TIM1_CHANNEL)
#    error "CONFIG_STM32_TIM1_CHANNEL must be provided"
#  elif CONFIG_STM32_TIM1_CHANNEL == 1
#    define PWM_TIM1_PINCFG GPIO_TIM1_CH1OUT
#  elif CONFIG_STM32_TIM1_CHANNEL == 2
#    define PWM_TIM1_PINCFG GPIO_TIM1_CH2OUT
#  elif CONFIG_STM32_TIM1_CHANNEL == 3
#    define PWM_TIM1_PINCFG GPIO_TIM1_CH3OUT
#  elif CONFIG_STM32_TIM1_CHANNEL == 4
#    define PWM_TIM1_PINCFG GPIO_TIM1_CH4OUT
#  else
#    error "Unsupported value of CONFIG_STM32_TIM1_CHANNEL"
#  endif
#endif

#ifdef CONFIG_STM32_TIM2_PWM
#  if !defined(CONFIG_STM32_TIM2_CHANNEL)
#    error "CONFIG_STM32_TIM2_CHANNEL must be provided"
#  elif CONFIG_STM32_TIM2_CHANNEL == 1
#    define PWM_TIM2_PINCFG GPIO_TIM2_CH1OUT
#  elif CONFIG_STM32_TIM2_CHANNEL == 2
#    define PWM_TIM2_PINCFG GPIO_TIM2_CH2OUT
#  elif CONFIG_STM32_TIM2_CHANNEL == 3
#    define PWM_TIM2_PINCFG GPIO_TIM2_CH3OUT
#  elif CONFIG_STM32_TIM2_CHANNEL == 4
#    define PWM_TIM2_PINCFG GPIO_TIM2_CH4OUT
#  else
#    error "Unsupported value of CONFIG_STM32_TIM2_CHANNEL"
#  endif
#endif

#ifdef CONFIG_STM32_TIM3_PWM
#  if !defined(CONFIG_STM32_TIM3_CHANNEL)
#    error "CONFIG_STM32_TIM3_CHANNEL must be provided"
#  elif CONFIG_STM32_TIM3_CHANNEL == 1
#    define PWM_TIM3_PINCFG GPIO_TIM3_CH1OUT
#  elif CONFIG_STM32_TIM3_CHANNEL == 2
#    define PWM_TIM3_PINCFG GPIO_TIM3_CH2OUT
#  elif CONFIG_STM32_TIM3_CHANNEL == 3
#    define PWM_TIM3_PINCFG GPIO_TIM3_CH3OUT
#  elif CONFIG_STM32_TIM3_CHANNEL == 4
#    define PWM_TIM3_PINCFG GPIO_TIM3_CH4OUT
#  else
#    error "Unsupported value of CONFIG_STM32_TIM3_CHANNEL"
#  endif
#endif

#ifdef CONFIG_STM32_TIM4_PWM
#  if !defined(CONFIG_STM32_TIM4_CHANNEL)
#    error "CONFIG_STM32_TIM4_CHANNEL must be provided"
#  elif CONFIG_STM32_TIM4_CHANNEL == 1
#    define PWM_TIM4_PINCFG GPIO_TIM4_CH1OUT
#  elif CONFIG_STM32_TIM4_CHANNEL == 2
#    define PWM_TIM4_PINCFG GPIO_TIM4_CH2OUT
#  elif CONFIG_STM32_TIM4_CHANNEL == 3
#    define PWM_TIM4_PINCFG GPIO_TIM4_CH3OUT
#  elif CONFIG_STM32_TIM4_CHANNEL == 4
#    define PWM_TIM4_PINCFG GPIO_TIM4_CH4OUT
#  else
#    error "Unsupported value of CONFIG_STM32_TIM4_CHANNEL"
#  endif
#endif

#ifdef CONFIG_STM32_TIM5_PWM
#  if !defined(CONFIG_STM32_TIM5_CHANNEL)
#    error "CONFIG_STM32_TIM5_CHANNEL must be provided"
#  elif CONFIG_STM32_TIM5_CHANNEL == 1
#    define PWM_TIM5_PINCFG GPIO_TIM5_CH1OUT
#  elif CONFIG_STM32_TIM5_CHANNEL == 2
#    define PWM_TIM5_PINCFG GPIO_TIM5_CH2OUT
#  elif CONFIG_STM32_TIM5_CHANNEL == 3
#    define PWM_TIM5_PINCFG GPIO_TIM5_CH3OUT
#  elif CONFIG_STM32_TIM5_CHANNEL == 4
#    define PWM_TIM5_PINCFG GPIO_TIM5_CH4OUT
#  else
#    error "Unsupported value of CONFIG_STM32_TIM5_CHANNEL"
#  endif
#endif

#ifdef CONFIG_STM32_TIM8_PWM
#  if !defined(CONFIG_STM32_TIM8_CHANNEL)
#    error "CONFIG_STM32_TIM8_CHANNEL must be provided"
#  elif CONFIG_STM32_TIM8_CHANNEL == 1
#    define PWM_TIM8_PINCFG GPIO_TIM8_CH1OUT
#  elif CONFIG_STM32_TIM8_CHANNEL == 2
#    define PWM_TIM8_PINCFG GPIO_TIM8_CH2OUT
#  elif CONFIG_STM32_TIM8_CHANNEL == 3
#    define PWM_TIM8_PINCFG GPIO_TIM8_CH3OUT
#  elif CONFIG_STM32_TIM8_CHANNEL == 4
#    define PWM_TIM8_PINCFG GPIO_TIM8_CH4OUT
#  else
#    error "Unsupported value of CONFIG_STM32_TIM8_CHANNEL"
#  endif
#endif

#ifdef CONFIG_STM32_TIM9_PWM
#  if !defined(CONFIG_STM32_TIM9_CHANNEL)
#    error "CONFIG_STM32_TIM9_CHANNEL must be provided"
#  elif CONFIG_STM32_TIM9_CHANNEL == 1
#    define PWM_TIM9_PINCFG GPIO_TIM9_CH1OUT
#  elif CONFIG_STM32_TIM9_CHANNEL == 2
#    define PWM_TIM9_PINCFG GPIO_TIM9_CH2OUT
#  elif CONFIG_STM32_TIM9_CHANNEL == 3
#    define PWM_TIM9_PINCFG GPIO_TIM9_CH3OUT
#  elif CONFIG_STM32_TIM9_CHANNEL == 4
#    define PWM_TIM9_PINCFG GPIO_TIM9_CH4OUT
#  else
#    error "Unsupported value of CONFIG_STM32_TIM9_CHANNEL"
#  endif
#endif

#ifdef CONFIG_STM32_TIM10_PWM
#  if !defined(CONFIG_STM32_TIM10_CHANNEL)
#    error "CONFIG_STM32_TIM10_CHANNEL must be provided"
#  elif CONFIG_STM32_TIM10_CHANNEL == 1
#    define PWM_TIM10_PINCFG GPIO_TIM10_CH1OUT
#  elif CONFIG_STM32_TIM10_CHANNEL == 2
#    define PWM_TIM10_PINCFG GPIO_TIM10_CH2OUT
#  elif CONFIG_STM32_TIM10_CHANNEL == 3
#    define PWM_TIM10_PINCFG GPIO_TIM10_CH3OUT
#  elif CONFIG_STM32_TIM10_CHANNEL == 4
#    define PWM_TIM10_PINCFG GPIO_TIM10_CH4OUT
#  else
#    error "Unsupported value of CONFIG_STM32_TIM10_CHANNEL"
#  endif
#endif

#ifdef CONFIG_STM32_TIM11_PWM
#  if !defined(CONFIG_STM32_TIM11_CHANNEL)
#    error "CONFIG_STM32_TIM11_CHANNEL must be provided"
#  elif CONFIG_STM32_TIM11_CHANNEL == 1
#    define PWM_TIM11_PINCFG GPIO_TIM11_CH1OUT
#  elif CONFIG_STM32_TIM11_CHANNEL == 2
#    define PWM_TIM11_PINCFG GPIO_TIM11_CH2OUT
#  elif CONFIG_STM32_TIM11_CHANNEL == 3
#    define PWM_TIM11_PINCFG GPIO_TIM11_CH3OUT
#  elif CONFIG_STM32_TIM11_CHANNEL == 4
#    define PWM_TIM11_PINCFG GPIO_TIM11_CH4OUT
#  else
#    error "Unsupported value of CONFIG_STM32_TIM11_CHANNEL"
#  endif
#endif

#ifdef CONFIG_STM32_TIM12_PWM
#  if !defined(CONFIG_STM32_TIM12_CHANNEL)
#    error "CONFIG_STM32_TIM12_CHANNEL must be provided"
#  elif CONFIG_STM32_TIM12_CHANNEL == 1
#    define PWM_TIM12_PINCFG GPIO_TIM12_CH1OUT
#  elif CONFIG_STM32_TIM12_CHANNEL == 2
#    define PWM_TIM12_PINCFG GPIO_TIM12_CH2OUT
#  elif CONFIG_STM32_TIM12_CHANNEL == 3
#    define PWM_TIM12_PINCFG GPIO_TIM12_CH3OUT
#  elif CONFIG_STM32_TIM12_CHANNEL == 4
#    define PWM_TIM12_PINCFG GPIO_TIM12_CH4OUT
#  else
#    error "Unsupported value of CONFIG_STM32_TIM12_CHANNEL"
#  endif
#endif

#ifdef CONFIG_STM32_TIM13_PWM
#  if !defined(CONFIG_STM32_TIM13_CHANNEL)
#    error "CONFIG_STM32_TIM13_CHANNEL must be provided"
#  elif CONFIG_STM32_TIM13_CHANNEL == 1
#    define PWM_TIM13_PINCFG GPIO_TIM13_CH1OUT
#  elif CONFIG_STM32_TIM13_CHANNEL == 2
#    define PWM_TIM13_PINCFG GPIO_TIM13_CH2OUT
#  elif CONFIG_STM32_TIM13_CHANNEL == 3
#    define PWM_TIM13_PINCFG GPIO_TIM13_CH3OUT
#  elif CONFIG_STM32_TIM13_CHANNEL == 4
#    define PWM_TIM13_PINCFG GPIO_TIM13_CH4OUT
#  else
#    error "Unsupported value of CONFIG_STM32_TIM13_CHANNEL"
#  endif
#endif

#ifdef CONFIG_STM32_TIM14_PWM
#  if !defined(CONFIG_STM32_TIM14_CHANNEL)
#    error "CONFIG_STM32_TIM14_CHANNEL must be provided"
#  elif CONFIG_STM32_TIM14_CHANNEL == 1
#    define PWM_TIM14_PINCFG GPIO_TIM14_CH1OUT
#  elif CONFIG_STM32_TIM14_CHANNEL == 2
#    define PWM_TIM14_PINCFG GPIO_TIM14_CH2OUT
#  elif CONFIG_STM32_TIM14_CHANNEL == 3
#    define PWM_TIM14_PINCFG GPIO_TIM14_CH3OUT
#  elif CONFIG_STM32_TIM14_CHANNEL == 4
#    define PWM_TIM14_PINCFG GPIO_TIM14_CH4OUT
#  else
#    error "Unsupported value of CONFIG_STM32_TIM14_CHANNEL"
#  endif
#endif

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: stm32_pwminitialize
 *
 * Description:
 *   Initialize one timer for use with the upper_level PWM driver.
 *
 * Input Parameters:
 *   timer - A number identifying the timer use.  The number of valid timer
 *     IDs varies with the STM32 MCU and MCU family but is somewhere in
 *     the range of {1,..,14}.
 *
 * Returned Value:
 *   On success, a pointer to the STM32 lower half PWM driver is returned.
 *   NULL is returned on any failure.
 *
 ************************************************************************************/

EXTERN FAR struct pwm_lowerhalf_s *stm32_pwminitialize(int timer);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* CONFIG_STM32_TIMx_PWM */
#endif /* __ARCH_ARM_SRC_STM32_STM32_PWM_H */
