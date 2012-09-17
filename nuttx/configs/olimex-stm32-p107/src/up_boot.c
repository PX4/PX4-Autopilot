/************************************************************************************
 * configs/olimex-stm32-p107/src/up_boot.c
 * arch/arm/src/board/up_boot.c
 *
 *   Copyright (C) 2009, 2012 Gregory Nutt. All rights reserved.
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

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include <debug.h>

#include <arch/board/board.h>

#include "up_arch.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/************************************************************************************
 * Private Functions
 ************************************************************************************/

/************************************************************************************
 * Public Functions
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

void stm32_boardinitialize(void)
{
}

/************************************************************************************
 * Name: stm32_board_clockconfig
 *
 * Description:
 *   Any STM32 board may replace the "standard" board clock configuration logic with
 *   its own, custom clock cofiguration logic.
 *
 ************************************************************************************/

#ifdef CONFIG_ARCH_BOARD_STM32_CUSTOM_CLOCKCONFIG
void stm32_board_clockconfig(void)
{
  uint32_t regval;
    
  regval  = getreg32(STM32_RCC_CR);
  regval &= ~RCC_CR_HSEBYP;         /* Disable HSE clock bypass */
  regval |= RCC_CR_HSEON;           /* Enable HSE */
  putreg32(regval, STM32_RCC_CR);

  /* Set flash wait states
   * Sysclk runs with 72MHz -> 2 waitstates.
   * 0WS from 0-24MHz
   * 1WS from 24-48MHz
   * 2WS from 48-72MHz
   */

  regval  = getreg32(STM32_FLASH_ACR);
  regval &= ~FLASH_ACR_LATENCY_MASK;
  regval |= (FLASH_ACR_LATENCY_2|FLASH_ACR_PRTFBE);
  putreg32(regval, STM32_FLASH_ACR);
     
  regval = getreg32(STM32_RCC_CFGR2);
  regval &= ~(RCC_CFGR2_PREDIV2_MASK 
        | RCC_CFGR2_PLL2MUL_MASK
        | RCC_CFGR2_PREDIV1SRC_MASK
        | RCC_CFGR2_PREDIV1_MASK);
  regval |= RCC_CFGR2_PREDIV2d5;       /* 25MHz / 5 */
  regval |= RCC_CFGR2_PLL2MULx8;       /* 5MHz * 8 => 40MHz */
  regval |= RCC_CFGR2_PREDIV1SRC_PLL2; /* Use PLL2 as input for PREDIV1 */
  regval |= RCC_CFGR2_PREDIV1d5;       /* 40MHz / 5 => 8MHz */
  putreg32(regval, STM32_RCC_CFGR2);

  /* Set the PCLK2 divider */

  regval = getreg32(STM32_RCC_CFGR);
  regval &= ~(RCC_CFGR_PPRE2_MASK | RCC_CFGR_HPRE_MASK);
  regval |= STM32_RCC_CFGR_PPRE2;
  regval |= RCC_CFGR_HPRE_SYSCLK;
  putreg32(regval, STM32_RCC_CFGR);
  
  /* Set the PCLK1 divider */

  regval = getreg32(STM32_RCC_CFGR);
  regval &= ~RCC_CFGR_PPRE1_MASK;
  regval |= STM32_RCC_CFGR_PPRE1;
  putreg32(regval, STM32_RCC_CFGR);

  regval = getreg32(STM32_RCC_CR);
  regval |= RCC_CR_PLL2ON;
  putreg32(regval, STM32_RCC_CR);

  /* Wait for PLL2 ready */

  while((getreg32(STM32_RCC_CR) & RCC_CR_PLL2RDY) == 0);

  /* Setup PLL3 for RMII clock on MCO */

  regval = getreg32(STM32_RCC_CFGR2);
  regval &= ~(RCC_CFGR2_PLL3MUL_MASK);
  regval |= RCC_CFGR2_PLL3MULx10;
  putreg32(regval, STM32_RCC_CFGR2);

  /* Switch PLL3 on */

  regval = getreg32(STM32_RCC_CR);
  regval |= RCC_CR_PLL3ON;
  putreg32(regval, STM32_RCC_CR);
    
  while ((getreg32(STM32_RCC_CR) & RCC_CR_PLL3RDY) == 0);

  /* Set main PLL source 8MHz * 9 => 72MHz*/

  regval = getreg32(STM32_RCC_CFGR);
  regval &= ~(RCC_CFGR_PLLSRC | RCC_CFGR_PLLMUL_MASK);
  regval |= (RCC_CFGR_PLLSRC | RCC_CFGR_PLLMUL_CLKx9);
  putreg32(regval, STM32_RCC_CFGR);
    
  /* Switch main PLL on */

  regval = getreg32(STM32_RCC_CR);
  regval |= RCC_CR_PLLON;
  putreg32(regval, STM32_RCC_CR);

  while ((getreg32(STM32_RCC_CR) & RCC_CR_PLLRDY) == 0);

  /* Select PLL as system clock source */

  regval  = getreg32(STM32_RCC_CFGR);
  regval &= ~RCC_CFGR_SW_MASK;
  regval |= RCC_CFGR_SW_PLL;
  putreg32(regval, STM32_RCC_CFGR);
   
  /* Wait until PLL is used as the system clock source */

  while ((getreg32(STM32_RCC_CFGR) & RCC_CFGR_SWS_PLL) == 0);
}
#endif
