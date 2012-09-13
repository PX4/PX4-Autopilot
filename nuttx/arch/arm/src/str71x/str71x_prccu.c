/********************************************************************************
 * arch/arm/src/str71x/str71x_prccu.c
 *
 *   Copyright (C) 2008-2009 Gregory Nutt. All rights reserved.
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
 ********************************************************************************/

/********************************************************************************
 * Included Files
 ********************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>

#include "chip.h"
#include "up_arch.h"
#include "str71x_internal.h"

/********************************************************************************
 * Pre-procesor Definitions
 ********************************************************************************/

/* Select set of peripherals to be enabled */

/* APB1 periperals */

#ifndef CONFIG_STR71X_I2C0
#  define APB1EN_I2C0 STR71X_APB1_I2C0
#else
#  define APB1EN_I2C0 (0)
#endif

#ifndef CONFIG_STR71X_I2C1
#  define APB1EN_I2C1 STR71X_APB1_I2C1
#else
#  ifndef CONFIG_STR71X_GPIO0
#    error "I2C1 requires GPIO0"
#  endif
#  ifdef CONFIG_STR71X_BSPI0
#    error "I2C1 is incompatible with BSPI0"
#  endif
#  define APB1EN_I2C1 (0)
#endif

#ifndef CONFIG_STR71X_UART0
#  define APB1EN_UART0 STR71X_APB1_UART0
#else
#  ifndef CONFIG_STR71X_GPIO0
#    error "CONFIG_STR71X_UART0 requires CONFIG_STR71X_GPIO0"
#  endif
#  define APB1EN_UART0 (0)
#endif

#ifndef CONFIG_STR71X_UART1
#  define APB1EN_UART1 STR71X_APB1_UART1
#else
#  ifndef CONFIG_STR71X_GPIO0
#    error "CONFIG_STR71X_UART1 requires CONFIG_STR71X_GPIO0"
#  endif
#  define APB1EN_UART1 (0)
#endif

#ifndef CONFIG_STR71X_UART2
#  define APB1EN_UART2 STR71X_APB1_UART2
#else
#  ifndef CONFIG_STR71X_GPIO0
#    error "CONFIG_STR71X_UART2 requires CONFIG_STR71X_GPIO0"
#  endif
#  define APB1EN_UART2 (0)
#endif

#ifndef CONFIG_STR71X_UART3
#  define APB1EN_UART3 STR71X_APB1_UART3
#else
#  ifndef CONFIG_STR71X_GPIO0
#    error "UART3 requires GPIO0"
#  endif
#  ifdef CONFIG_STR71X_BSPI0
#    error "UART3 is incompatible with BSPI0"
#  endif
#  define APB1EN_UART3 (0)
#endif

#ifndef CONFIG_STR71X_USB
#  define APB1EN_USB STR71X_APB1_USB
#else
#  define APB1EN_USB (0)
#endif

#ifndef CONFIG_STR71X_CAN
#  define APB1EN_CAN STR71X_APB1_CAN
#else
#  define APB1EN_CAN (0)
#endif

#ifndef CONFIG_STR71X_BSPI0
#  define APB1EN_BSPI0 STR71X_APB1_BSPI0
#else
#  ifndef CONFIG_STR71X_GPIO0
#    error "BSPI0 requires GPIO0"
#  endif
#  ifdef CONFIG_STR71X_UART3
#    error "BSPI0 is incompatible with UART3"
#  endif
#  ifdef CONFIG_STR71X_I2C1
#    error "BSPI0 is incompatible with I2C1"
#  endif
#  define APB1EN_BSPI0 (0)
#endif

#ifndef CONFIG_STR71X_BSPI1
#  define APB1EN_BSPI1 STR71X_APB1_BSPI1
#else
#  ifndef CONFIG_STR71X_GPIO0
#    error "BSPI1 requires GPIO0"
#  endif
#  define APB1EN_BSPI1 (0)
#endif

#ifndef CONFIG_STR71X_HDLC
#  define APB1EN_HDLC STR71X_APB1_HDLC
#else
#  define APB1EN_HDLC (0)
#endif

#define APB1EN_ALL (APB1EN_I2C0|APB1EN_I2C1|APB1EN_UART0|APB1EN_UART1|\
                    APB1EN_UART2|APB1EN_UART3|APB1EN_USB|APB1EN_CAN|\
                    APB1EN_BSPI0|APB1EN_BSPI1|APB1EN_HDLC)

/* APB2 Peripherals */

#ifndef CONFIG_STR71X_XTI
#  define APB2EN_XTI STR71X_APB2_XTI
#else
#  define APB2EN_XTI (0)
#endif

#ifndef CONFIG_STR71X_GPIO0
#  define APB2EN_GPIO0 STR71X_APB2_GPIO0
#else
#  define APB2EN_GPIO0 (0)
#endif

#ifndef CONFIG_STR71X_GPIO1
#  define APB2EN_GPIO1 STR71X_APB2_GPIO1
#else
#  define APB2EN_GPIO1 (0)
#endif

#ifndef CONFIG_STR71X_GPIO2
#  define APB2EN_GPIO2 STR71X_APB2_GPIO2
#else
#  define APB2EN_GPIO2 (0)
#endif

#ifndef CONFIG_STR71X_ADC12
#  define APB2EN_ADC12 STR71X_APB2_ADC12
#else
#  define APB2EN_ADC12 (0)
#endif

#ifndef CONFIG_STR71X_CKOUT
#  define APB2EN_CKOUT STR71X_APB2_CKOUT
#else
#  define APB2EN_CKOUT (0)
#endif

#define APB2EN_TIM0 (0) /* System timer -- always enabled */

#ifndef CONFIG_STR71X_TIM1
#  define APB2EN_TIM1 STR71X_APB2_TIM1
#else
#  define APB2EN_TIM1 (0)
#endif

#ifndef CONFIG_STR71X_TIM2
#  define APB2EN_TIM2 STR71X_APB2_TIM2
#else
#  define APB2EN_TIM2 (0)
#endif

#ifndef CONFIG_STR71X_TIM3
#  define APB2EN_TIM3 STR71X_APB2_TIM3
#else
#  define APB2EN_TIM3 (0)
#endif

#ifndef CONFIG_STR71X_RTC
#  define APB2EN_RTC STR71X_APB2_RTC
#else
#  define APB2EN_RTC (0)
#endif

#define APB2EN_EIC (0) /* Interrupt controller always enabled */

#define APB2EN_ALL (APB2EN_XTI|APB2EN_GPIO0|APB2EN_GPIO1|APB2EN_GPIO2|\
                    APB2EN_ADC12|APB2EN_CKOUT|APB2EN_TIM0|APB2EN_TIM1|\
                    APB2EN_TIM2|APB2EN_TIM3|APB2EN_RTC|APB2EN_EIC)


#if STR71X_PLL1OUT_MUL == 12
#  define PLL1MUL STR71X_RCCUPLL1CR_MUL12
#elif STR71X_PLL1OUT_MUL == 16
#  define PLL1MUL STR71X_RCCUPLL1CR_MUL16
#elif STR71X_PLL1OUT_MUL == 20
#  define PLL1MUL STR71X_RCCUPLL1CR_MUL20
#elif STR71X_PLL1OUT_MUL == 24
#  define PLL1MUL STR71X_RCCUPLL1CR_MUL24
#else
#  error "Unsupporetd value for STR71X_PLL1OUT_MUL"
#endif

#if STR71X_PLL1OUT_DIV == 1
#  define PLL1DIV STR71X_RCCUPLL1CR_DIV1
#elif STR71X_PLL1OUT_DIV == 2
#  define PLL1DIV STR71X_RCCUPLL1CR_DIV2
#elif STR71X_PLL1OUT_DIV == 3
#  define PLL1DIV STR71X_RCCUPLL1CR_DIV3
#elif STR71X_PLL1OUT_DIV == 4
#  define PLL1DIV STR71X_RCCUPLL1CR_DIV4
#elif STR71X_PLL1OUT_DIV == 5
#  define PLL1DIV STR71X_RCCUPLL1CR_DIV5
#elif STR71X_PLL1OUT_DIV == 6
#  define PLL1DIV STR71X_RCCUPLL1CR_DIV6
#elif STR71X_PLL1OUT_DIV == 7
#  define PLL1DIV STR71X_RCCUPLL1CR_DIV7
#else
#  error "Unsupported value for STR71X_PLL1OUT_DIV"
#endif

#if STR71X_APB1_DIV == 1
#  define APB1DIV STR71X_PCUPDIVR_APB1DIV1
#elif STR71X_APB1_DIV == 2
#  define APB1DIV STR71X_PCUPDIVR_APB1DIV2
#elif STR71X_APB1_DIV == 4
#  define APB1DIV STR71X_PCUPDIVR_APB1DIV4
#elif STR71X_APB1_DIV == 8
#  define APB1DIV STR71X_PCUPDIVR_APB1DIV8
#else
#  error "Unsupported value for STR71X_APB1_DIV"
#endif

#if STR71X_APB2_DIV == 1
#  define APB2DIV STR71X_PCUPDIVR_APB2DIV1
#elif STR71X_APB2_DIV == 2
#  define APB2DIV STR71X_PCUPDIVR_APB2DIV2
#elif STR71X_APB2_DIV == 4
#  define APB2DIV STR71X_PCUPDIVR_APB2DIV4
#elif STR71X_APB2_DIV == 8
#  define APB2DIV STR71X_PCUPDIVR_APB2DIV8
#else
#  error "Unsupported value for STR71X_APB2_DIV"
#endif

#if STR71X_MCLK_DIV == 1
#  define MCLKDIV STR71X_PCUMDIVR_DIV1
#elif STR71X_MCLK_DIV == 2
#  define MCLKDIV STR71X_PCUMDIVR_DIV2
#elif STR71X_MCLK_DIV == 4
#  define MCLKDIV STR71X_PCUMDIVR_DIV4
#elif STR71X_MCLK_DIV == 8
#  define MCLKDIV STR71X_PCUMDIVR_DIV8
#else
#  error "Unsupported value for STR71X_MCLK_DIV"
#endif

#if STR71X_PLL2OUT_MUL == 12
#  define PLL2MUL STR71X_PCUPPL2CR_MUL12
#elif STR71X_PLL2OUT_MUL == 16
#  define PLL2MUL STR71X_PCUPPL2CR_MUL16
#elif STR71X_PLL2OUT_MUL == 20
#  define PLL2MUL STR71X_PCUPPL2CR_MUL20
#elif STR71X_PLL2OUT_MUL == 28
#  define PLL2MUL STR71X_PCUPPL2CR_MUL28
#else
#  error "Unsupporetd value for STR71X_PLL2OUT_MUL"
#endif

#if STR71X_PLL2OUT_DIV == 1
#  define PLL2DIV STR71X_PCUPPL2CR_DIV1
#elif STR71X_PLL2OUT_DIV == 2
#  define PLL2DIV STR71X_PCUPPL2CR_DIV2
#elif STR71X_PLL2OUT_DIV == 3
#  define PLL2DIV STR71X_PCUPPL2CR_DIV3
#elif STR71X_PLL2OUT_DIV == 4
#  define PLL2DIV STR71X_PCUPPL2CR_DIV4
#elif STR71X_PLL2OUT_DIV == 5
#  define PLL2DIV STR71X_PCUPPL2CR_DIV5
#elif STR71X_PLL2OUT_DIV == 6
#  define PLL2DIV STR71X_PCUPPL2CR_DIV6
#elif STR71X_PLL2OUT_DIV == 7
#  define PLL2DIV STR71X_PCUPPL2CR_DIV7
#else
#  error "Unsupported value for STR71X_PLL2OUT_DIV"
#endif

/********************************************************************************
 * Private Types
 ********************************************************************************/

/********************************************************************************
 * Public Data
 ********************************************************************************/

/********************************************************************************
 * Private Data
 ********************************************************************************/

/********************************************************************************
 * Private Functions
 ********************************************************************************/

/********************************************************************************
 * Public Funstions
 ********************************************************************************/

/********************************************************************************
 * Name: str71x_prccuinit
 *
 * Description:
 *   Initialize the PCU/RCCU based on the NuttX configuration and the board-specific
 *   settings in board.h
 *
 ********************************************************************************/

void str71x_prccuinit(void)
{
  uint32_t reg32;
  uint16_t reg16;

  /* Divide RCLK to obtain PCLK1 & 2 clock for the APB1 & 2 peripherals.  The divider
   * values are provided in board.h
   */

  reg16  = getreg16(STR71X_PCU_PDIVR);
  reg16 &= ~(STR71X_PCUPDIVR_FACT1MASK|STR71X_PCUPDIVR_FACT2MASK);
  reg16 |= (APB1DIV|APB2DIV);
  putreg16(reg16, STR71X_PCU_PDIVR);

  /* Configure the main system clock (MCLK) divider with value from board.h */

  reg16  = getreg16(STR71X_PCU_MDIVR);
  reg16 &= ~STR71X_PCUMDIVR_FACTMASK;
  reg16 |= MCLKDIV;
  putreg16(reg16 , STR71X_PCU_MDIVR);

  /* Turn off the PLL1 by setting bits DX[2:0] */

  putreg32(STR71X_RCCUPLL1CR_CLK2, STR71X_RCCU_PLL1CR);

  /* Configure the PLL1CR register using the provided multiplier and
   * divider.  The FREF_RANGE bit is also set if the input frequency
   * (CLK2) is greater than 3MHz.
   */

#if STR71X_CLK2 > 3000000
  putreg32(PLL1MUL|PLL1DIV, STR71X_RCCU_PLL1CR);
#else
  putreg32(PLL1MUL|PLL1DIV|STR71X_RCCUPLL1CR_FREFRANGE, STR71X_RCCU_PLL1CR);
#endif

  /* Wait for the PLL to lock */

  while ((getreg16(STR71X_RCCU_CFR) & STR71X_RCCUCFR_LOCK) == 0);

  /* Set the CK2_16 Bit in the CFR to use CLK2/PLL1OUT as CLK3 */

  reg32  = getreg32(STR71X_RCCU_CFR);
  reg32 |= STR71X_RCCUCFR_CK216;

  /* Should the main oscillator divided down by 2? */

#ifdef STR71X_PLL1IN_DIV2
  reg32 |= STR71X_RCCUCFR_DIV2;
#else
  reg32 &= ~STR71X_RCCUCFR_DIV2;
#endif
  putreg32(reg32, STR71X_RCCU_CFR);

  /* Wait for the PLL to lock */

  while ((getreg16(STR71X_RCCU_CFR) & STR71X_RCCUCFR_LOCK) == 0);

  /* Select CLK3 (vs the alternative source) for RCLK in the clock
   * control register (CCR)
   */

  reg16 = getreg16(STR71X_RCCU_CCR);
  reg16 &= ~STR71X_RCCUCCR_CKAFSEL;
  putreg16(reg16, STR71X_RCCU_CCR);

  /* Select PLL1OUT as the CLK3 */

  reg32 = getreg32(STR71X_RCCU_CFR);
  putreg32(reg32 | STR71X_RCCUCFR_CSUCKSEL, STR71X_RCCU_CFR);

  /* Enable clocking on selected periperals */

  putreg32(APB1EN_ALL, STR71X_APB1_CKDIS);
  putreg32(APB2EN_ALL, STR71X_APB2_CKDIS);

  /* Configure PLL2 */

#if defined(CONFIG_STR71X_HDLC) || (defined(CONFIG_STR71X_USB) && defined(STR71X_USBIN_PLL2))
  reg16  = getreg16(STR71X_PCU_PLL2CR);
  reg16 &= ~(STR71X_PCUPPL2CR_MXMASK|STR71X_PCUPPL2CR_DXMASK);
  reg16 |= (PLL2MUL|PLL2DIV);

  /* Set the PLL2 FRQRNG bit according to the PLL2 input frequency */

#if STR71X_PCU_HCLK_OSC < 3000000
  reg16 &= ~STR71X_PCUPPL2CR_FRQRNG;
#else
  reg16 |= STR71X_PCUPPL2CR_FRQRNG;
#endif
  putreg16(reg16, STR71X_PCU_PLL2CR);

  /* Wait for PLL2 to lock in */
  // while ((getreg16(STR71X_PCU_PLL2CR) & STR71X_PCUPPL2CR_LOCK) == 0);
#endif

  /* Select the USB clock source */

#ifdef CONFIG_STR71X_USB
  reg16 = getreg16(STR71X_PCU_PLL2CR);
#ifdef STR71X_USBIN_PLL2
  /* PLL2 is the clock source to the USB */

  reg16 |= STR71X_PCUPPL2CR_USBEN;
#else
  /* USBCLK pin is the clock source to the USB */

  reg16 &= ~STR71X_PCUPPL2CR_USBEN;
#endif
  putreg16(reg16, STR71X_PCU_PLL2CR);
#endif
}


