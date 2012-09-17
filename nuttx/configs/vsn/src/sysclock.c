/****************************************************************************
 * configs/vsn/src/sysclock.c
 *
 *   Copyright (C) 2011 Uros Platise. All rights reserved.
 * 
 *   Author: Uros Platise <uros.platise@isotel.eu>
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
 ****************************************************************************/

/** \file
 *  \author Uros Platise
 *  \brief VSN System Clock Configuration
 */

#include "vsn.h"


/****************************************************************************
 * Private Functions
 ****************************************************************************/ 

/** Selects internal HSI Clock, SYSCLK = 36 MHz, HCLK = 36 MHz
  *  - HSI at 8 MHz, :2 enters DPLL * 9, to get 36 MHz
  *  - AHB prescaler is set to 1 for maximum performance of all peripherals and FRAM (18 Mbps)
  *  - Suitable for start-up and lower power operation
  *  - Flash Wait State = 1, since it is 64-bit prefetch, it satisfies two 32-bit instructions
  *    (and branch losses a single cycle only, I found this as the best performance vs. frequency)
  *  - Sleep with peripherals disabled is about 2.5 mA @ 36 MHz, HSI
  * 
  * \todo:
  *   - dynamic clock scalling according to cross-peripheral requirements, AHB prescaler could
  *     change if all other prescalers increase, to maintain the ratio and to have min. HCLK
  *     possible; This is of interest when peripherals consume 50% of all power, as for instance
  *     in sleep mode @ 36 MHz, HSI with all peripherals enabled, i = 7 mA, on 24 Mhz 4.8 mA and
  *     on 16 MHz 3.2 mA only. 
  * 
  * \return Nothing, operation is always successful.
  */
void sysclock_select_hsi(void)
{
    uint32_t regval;

    // Are we running on HSE?
    regval = getreg32(STM32_RCC_CR);
    if (regval & RCC_CR_HSEON) {
        
        // \todo: check is if we are running on HSE, we need the step down sequenuce from HSE -> HSI
        
        return; // do nothing at this time
    }
    
    // Set FLASH prefetch buffer and 1 wait state
    regval  = getreg32(STM32_FLASH_ACR);
    regval &= ~FLASH_ACR_LATENCY_MASK;
    regval |= (FLASH_ACR_LATENCY_1|FLASH_ACR_PRTFBE);
    putreg32(regval, STM32_FLASH_ACR);
     
    // Set the HCLK source/divider
    regval = getreg32(STM32_RCC_CFGR);
    regval &= ~RCC_CFGR_HPRE_MASK;
    regval |= STM32_RCC_CFGR_HPRE_HSI;
    putreg32(regval, STM32_RCC_CFGR);

    // Set the PCLK2 divider
    regval = getreg32(STM32_RCC_CFGR);
    regval &= ~RCC_CFGR_PPRE2_MASK;
    regval |= STM32_RCC_CFGR_PPRE2;
    putreg32(regval, STM32_RCC_CFGR);
  
    // Set the PCLK1 divider
    regval = getreg32(STM32_RCC_CFGR);
    regval &= ~RCC_CFGR_PPRE1_MASK;
    regval |= STM32_RCC_CFGR_PPRE1;
    putreg32(regval, STM32_RCC_CFGR);
    
    // Set the TIM1..8 clock multipliers
#ifdef STM32_TIM27_FREQMUL2  
#endif

#ifdef STM32_TIM18_FREQMUL2
#endif
 
    // Set the PLL source = HSI, divider (/2) and multipler (*9)
    regval = getreg32(STM32_RCC_CFGR);
    regval &= ~(RCC_CFGR_PLLSRC|RCC_CFGR_PLLXTPRE|RCC_CFGR_PLLMUL_MASK);
    regval |= (STM32_CFGR_PLLSRC_HSI|STM32_CFGR_PLLMUL_HSI);
    putreg32(regval, STM32_RCC_CFGR);
 
    // Enable the PLL
    regval = getreg32(STM32_RCC_CR);
    regval |= RCC_CR_PLLON;
    putreg32(regval, STM32_RCC_CR);
 
    // Wait until the PLL is ready
    while ((getreg32(STM32_RCC_CR) & RCC_CR_PLLRDY) == 0);
 
    // Select the system clock source (probably the PLL)
    regval  = getreg32(STM32_RCC_CFGR);
    regval &= ~RCC_CFGR_SW_MASK;
    regval |= STM32_SYSCLK_SW;
    putreg32(regval, STM32_RCC_CFGR);

    // Wait until the selected source is used as the system clock source
    while ((getreg32(STM32_RCC_CFGR) & RCC_CFGR_SWS_MASK) != STM32_SYSCLK_SWS);
    
    // map port PD0 and PD1 on OSC pins
    regval = getreg32(STM32_AFIO_MAPR);
    regval |= AFIO_MAPR_PD01_REMAP;
    putreg32(regval, STM32_AFIO_MAPR);
}


/** Selects external HSE Clock, SYSCLK = 72 MHz, HCLK = 36/72 MHz
  *  - HSE at 9 MHz, DPLL * 8, to get 72 MHz
  *  - Suitable for maximum performance and USB 
  *  - Sleep power consumption at HSE and at 72 MHz is 5.5 mA (3.1 @ 36 MHz)
  *  - Option AHB prescaler is set to :2 to be compatible with HSI to remain on HCLK = 36 MHz
  *  - Flash memory running on 72 MHz needs two wait states
  * 
  * \return Clock selection status
  * \retval 0 Successful
  * \retval -1 External clock is not provided
  * \retval -2 Could not lock to external clock
  */
int sysclock_select_hse(void)
{
    uint32_t regval;

    // be sure to release PD0 and PD1 pins from the OSC pins
    regval = getreg32(STM32_AFIO_MAPR);
    regval &= ~AFIO_MAPR_PD01_REMAP;
    putreg32(regval, STM32_AFIO_MAPR);

    // if (is cc1101 9 MHz clock output enabled), otherwise return with -1
    // I think that clock register provides HSE valid signal to detect that as well.
    
    return 0;
}


/****************************************************************************
 * Interrupts, Callbacks
 ****************************************************************************/ 


/** TODO: Interrupt on lost HSE clock, change it to HSI, ... restarting is 
  *   more complex as the step requires restart of CC1101 device driver;
  *   so spawn a task for that... once cc1101 is restarted signal an event
  *   to restart clock.
  */
void sysclock_hse_lost(void)
{
}


/****************************************************************************
 * Public Functions
 ****************************************************************************/ 

/** Setup system clock, enabled when:
  *   - CONFIG_ARCH_BOARD_STM32_CUSTOM_CLOCKCONFIG
  *  option is set in .config
  */
void stm32_board_clockconfig(void)
{
    sysclock_select_hsi();
}
