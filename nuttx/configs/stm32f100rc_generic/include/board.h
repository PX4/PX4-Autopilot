/************************************************************************************
 * configs/stm32f100rc_generic/include/board.h
 *
 *   Copyright (C) 2012 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *           Freddie Chopin <freddie_chopin@op.pl>
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

#ifndef __ARCH_BOARD_BOARD_H
#define __ARCH_BOARD_BOARD_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#ifndef __ASSEMBLY__
# include <stdint.h>
#endif
#include "stm32_rcc.h"
#include "stm32_internal.h"

/************************************************************************************
 * Definitions
 ************************************************************************************/

/* Clocking *************************************************************************/

/* On-board crystal frequency is 8MHz (HSE) */

#define STM32_BOARD_XTAL        8000000ul

/* PLL source is HSE / 1, PLL multiplier is 3: PLL output frequency is 8MHz (XTAL) x 3 = 24MHz */

#define STM32_CFGR2_PREDIV1     RCC_CFGR2_PREDIV1d1
#define STM32_CFGR_PLLSRC       RCC_CFGR_PLLSRC
#define STM32_CFGR_PLLXTPRE     0
#define STM32_CFGR_PLLMUL       RCC_CFGR_PLLMUL_CLKx3
#define STM32_PLL_FREQUENCY     (3 * STM32_BOARD_XTAL)

/* Use the PLL and set the SYSCLK source to be the PLL */

#define STM32_SYSCLK_SW         RCC_CFGR_SW_PLL
#define STM32_SYSCLK_SWS        RCC_CFGR_SWS_PLL
#define STM32_SYSCLK_FREQUENCY  STM32_PLL_FREQUENCY

/* AHB clock (HCLK) is SYSCLK (24MHz) */

#define STM32_RCC_CFGR_HPRE     RCC_CFGR_HPRE_SYSCLK
#define STM32_HCLK_FREQUENCY    STM32_PLL_FREQUENCY

/* APB2 clock (PCLK2) is HCLK (24MHz) */

#define STM32_RCC_CFGR_PPRE2    RCC_CFGR_PPRE2_HCLK
#define STM32_PCLK2_FREQUENCY   STM32_HCLK_FREQUENCY

/* APB2 timers (1, 15-17) will receive PCLK2. */

#define STM32_APB2_TIM1_CLKIN   STM32_PCLK2_FREQUENCY
#define STM32_APB2_TIM15_CLKIN  STM32_PCLK2_FREQUENCY
#define STM32_APB2_TIM16_CLKIN  STM32_PCLK2_FREQUENCY
#define STM32_APB2_TIM17_CLKIN  STM32_PCLK2_FREQUENCY

/* APB1 clock (PCLK1) is HCLK (24MHz) */

#define STM32_RCC_CFGR_PPRE1    RCC_CFGR_PPRE1_HCLK
#define STM32_PCLK1_FREQUENCY   STM32_HCLK_FREQUENCY

/* APB1 timers (2-7, 12-14) will receive PCLK1. */

#define STM32_APB1_TIM2_CLKIN   STM32_PCLK1_FREQUENCY
#define STM32_APB1_TIM3_CLKIN   STM32_PCLK1_FREQUENCY
#define STM32_APB1_TIM4_CLKIN   STM32_PCLK1_FREQUENCY
#define STM32_APB1_TIM5_CLKIN   STM32_PCLK1_FREQUENCY
#define STM32_APB1_TIM6_CLKIN   STM32_PCLK1_FREQUENCY
#define STM32_APB1_TIM7_CLKIN   STM32_PCLK1_FREQUENCY
#define STM32_APB1_TIM12_CLKIN  STM32_PCLK1_FREQUENCY
#define STM32_APB1_TIM13_CLKIN  STM32_PCLK1_FREQUENCY
#define STM32_APB1_TIM14_CLKIN  STM32_PCLK1_FREQUENCY

/* Timer Frequencies, if APBx is set to 1, frequency is same to APBx
 * otherwise frequency is 2xAPBx.
 * Note: TIM1,15-17 are on APB2, others on APB1 */

#define STM32_TIM18_FREQUENCY   STM32_HCLK_FREQUENCY
#define STM32_TIM27_FREQUENCY   STM32_HCLK_FREQUENCY

/* LED definitions ******************************************************************/

/* It is assumed that a generic board has 1 LED. Thus only two different states
 * can be shown. Statuses defined as "1" will light the LED, the ones defined as
 * "0" will turn the LED off. */

#define LED_STARTED       1
#define LED_HEAPALLOCATE  1
#define LED_IRQSENABLED   1
#define LED_STACKCREATED  1
#define LED_INIRQ         1
#define LED_SIGNAL        1
#define LED_ASSERTION     0
#define LED_PANIC         0

/* Button definitions ***************************************************************/

/* It is assumed that a generic board has 1 button. */

#define BUTTON_0           0

#define NUM_BUTTONS        1

#define BUTTON_0_BIT       (1 << BUTTON_0)

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
 * Public Function Prototypes
 ************************************************************************************/
/************************************************************************************
 * Name: stm32_boardinitialize
 *
 * Description:
 *   All STM32 architectures must provide the following entry point.  This entry point
 *   is called early in the intitialization -- after all memory has been configured
 *   and mapped but before any devices have been initialized.
 *
 ************************************************************************************/

EXTERN void stm32_boardinitialize(void);

/************************************************************************************
 * Name:  stm32_ledinit, stm32_setled, and stm32_setleds
 *
 * Description:
 *   If CONFIG_ARCH_LEDS is defined, then NuttX will control the on-board LEDs.  If
 *   CONFIG_ARCH_LEDS is not defined, then the following interfacesare available to
 *   control the LEDs from user applications.
 *
 ************************************************************************************/

#ifndef CONFIG_ARCH_LEDS
EXTERN void stm32_ledinit(void);
EXTERN void stm32_setled(int led, bool ledon);
EXTERN void stm32_setleds(uint8_t ledset);
#endif

/************************************************************************************
 * Button support.
 *
 * Description:
 *   up_buttoninit() must be called to initialize button resources.  After
 *   that, up_buttons() may be called to collect the current state of all
 *   buttons or up_irqbutton() may be called to register button interrupt
 *   handlers.
 *
 *   After up_buttoninit() has been called, up_buttons() may be called to
 *   collect the state of all buttons.  up_buttons() returns an 8-bit bit set
 *   with each bit associated with a button.  See the BUTTON_*_BIT and JOYSTICK_*_BIT
 *   definitions in board.h for the meaning of each bit.
 *
 *   up_irqbutton() may be called to register an interrupt handler that will
 *   be called when a button is depressed or released.  The ID value is a
 *   button enumeration value that uniquely identifies a button resource. See the
 *   BUTTON_* and JOYSTICK_* definitions in board.h for the meaning of enumeration
 *   value.  The previous interrupt handler address is returned (so that it may
 *   restored, if so desired).
 *
 ************************************************************************************/

#ifdef CONFIG_ARCH_BUTTONS
EXTERN void up_buttoninit(void);
EXTERN uint8_t up_buttons(void);
#ifdef CONFIG_ARCH_IRQBUTTONS
EXTERN xcpt_t up_irqbutton(int id, xcpt_t irqhandler);
#endif
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif  /* __ARCH_BOARD_BOARD_H */
