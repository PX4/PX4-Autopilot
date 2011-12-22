/************************************************************************************
 * configs/vsn/include/board.h
 * include/arch/board/board.h
 *
 *   Copyright (C) 2009 Gregory Nutt. All rights reserved.
 *   Copyright (C) 2011 Uros Platise. All rights reserved
 * 
 *   Authors: Gregory Nutt <spudmonkey@racsa.co.cr>
 *            Uros Platise <uros.platise@isotel.eu>
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
#include "stm32_sdio.h"
#include "stm32_internal.h"

#include "muxbus.h"


/************************************************************************************
 * Definitions
 ************************************************************************************/

/* Clocking *************************************************************************/

/* On-board external frequency source is 9MHz (HSE) provided by the CC1101, so it is
 * not available on power-up. Instead we are about to run on HSI*9 = 36 MHz, see 
 * up_sysclock.c for details. 
 */

#define STM32_BOARD_XTAL        9000000UL
#define STM32_BOARD_HCLK       36000000UL

/* PLL source is either HSI or HSE
 * When HSI: PLL multiplier is 9, out frequency 36 MHz
 * When HSE: PLL multiplier is 8: out frequency is 9 MHz x 8 = 72MHz 
 */
 
#define STM32_CFGR_PLLSRC_HSI  0
#define STM32_CFGR_PLLMUL_HSI  RCC_CFGR_PLLMUL_CLKx9

#define STM32_CFGR_PLLXTPRE_HSE 0
#define STM32_CFGR_PLLSRC_HSE  RCC_CFGR_PLLSRC
#define STM32_CFGR_PLLMUL_HSE  RCC_CFGR_PLLMUL_CLKx8

/* Use the PLL and set the SYSCLK source to be the PLL */

#define STM32_SYSCLK_SW        RCC_CFGR_SW_PLL
#define STM32_SYSCLK_SWS       RCC_CFGR_SWS_PLL

/* AHB clock (HCLK, 36 MHz) is SYSCLK on HSI or SYSCLK/2 on HSE */

#define STM32_RCC_CFGR_HPRE_HSI	RCC_CFGR_HPRE_SYSCLK
#define STM32_RCC_CFGR_HPRE_HSE	RCC_CFGR_HPRE_SYSCLKd2
#define STM32_HCLK_FREQUENCY   	STM32_BOARD_HCLK

/* APB2 clock (PCLK2) is HCLK (36MHz) */

#define STM32_RCC_CFGR_PPRE2    RCC_CFGR_PPRE2_HCLK
#define STM32_PCLK2_FREQUENCY   STM32_BOARD_HCLK

/* APB2 timers 1 and 8 will receive PCLK2. */

#define STM32_APB2_TIM1_CLKIN   (STM32_PCLK2_FREQUENCY)
#define STM32_APB2_TIM8_CLKIN   (STM32_PCLK2_FREQUENCY)

/* APB1 clock (PCLK1) is HCLK (36MHz) */

#define STM32_RCC_CFGR_PPRE1    RCC_CFGR_PPRE1_HCLK
#define STM32_PCLK1_FREQUENCY   STM32_BOARD_HCLK

/* APB1 timers 2-4 will receive PCLK1. */

#define STM32_APB1_TIM2_CLKIN   (STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM3_CLKIN   (STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM4_CLKIN   (STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM5_CLKIN   (STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM6_CLKIN   (STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM7_CLKIN   (STM32_PCLK1_FREQUENCY)

/* Timer 1..8 Frequencies */

#define STM32_TIM27_FREQUENCY   (STM32_BOARD_HCLK)
#define STM32_TIM18_FREQUENCY   (STM32_BOARD_HCLK)

/* USB divider -- Divide PLL clock by 1.5 */

#define STM32_CFGR_USBPRE      0

/* SDIO dividers.  Note that slower clocking is required when DMA is disabled 
 * in order to avoid RX overrun/TX underrun errors due to delayed responses
 * to service FIFOs in interrupt driven mode. 
 * 
 * SDcard default speed has max SDIO_CK freq of 25 MHz (12.5 Mbps)
 * After selection of high speed freq may be 50 MHz (25 Mbps)
 * Recommended default voltage: 3.3 V
 *
 * HCLK=36MHz, SDIOCLK=36 MHz, SDIO_CK=HCLK/(88+2)=400 KHz 
 */
  
#define SDIO_INIT_CLKDIV       (88 << SDIO_CLKCR_CLKDIV_SHIFT)

/* DMA ON:  HCLK=36 MHz, SDIOCLK=36MHz, SDIO_CK=HCLK/(0+2)=18 MHz
 * DMA OFF: HCLK=36 MHz, SDIOCLK=36MHz, SDIO_CK=HCLK/(1+2)=12 MHz
 */

#ifdef CONFIG_SDIO_DMA
#  define SDIO_MMCXFR_CLKDIV   (0 << SDIO_CLKCR_CLKDIV_SHIFT) 
#else
#  ifndef CONFIG_DEBUG
#    define SDIO_MMCXFR_CLKDIV    (1 << SDIO_CLKCR_CLKDIV_SHIFT)
#  else
#    define SDIO_MMCXFR_CLKDIV    (10 << SDIO_CLKCR_CLKDIV_SHIFT)
#  endif
#endif

/* DMA ON:  HCLK=72 MHz, SDIOCLK=72MHz, SDIO_CK=HCLK/(0+2)=18 MHz
 * DMA OFF: HCLK=72 MHz, SDIOCLK=72MHz, SDIO_CK=HCLK/(1+2)=12 MHz
 * Extra slow down in debug mode to get rid of underruns.
 */

#ifdef CONFIG_SDIO_DMA
#  define SDIO_SDXFR_CLKDIV    (0 << SDIO_CLKCR_CLKDIV_SHIFT)
#else
#  ifndef CONFIG_DEBUG
#    define SDIO_SDXFR_CLKDIV    (1 << SDIO_CLKCR_CLKDIV_SHIFT)
#  else
#    define SDIO_SDXFR_CLKDIV    (10 << SDIO_CLKCR_CLKDIV_SHIFT)
#  endif
#endif

/* LED definitions ******************************************************************/

/* The VSN has one LED that we will encode as: */

#define LED_STARTED       0  /* ... */
#define LED_HEAPALLOCATE  1  /* ... */
#define LED_IRQSENABLED   2  /* ... */
#define LED_STACKCREATED  3  /* ... */
#define LED_INIRQ         4  /* ... */
#define LED_SIGNAL        5  /* ... */
#define LED_ASSERTION     6  /* ... */
#define LED_PANIC         7  /* ... */
#define LED_IDLE		  8	 /* shows idle state */


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
 * Board Clock Configuration, called immediatelly after boot
 ************************************************************************************/
 
EXTERN void stm32_board_clockconfig(void);

 
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
 * Button support. (TODO: button is not yet supported)
 ************************************************************************************/

#ifdef CONFIG_ARCH_BUTTONS
EXTERN void up_buttoninit(void);
EXTERN uint8_t up_buttons(void);
#endif


#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_BOARD_BOARD_H */
