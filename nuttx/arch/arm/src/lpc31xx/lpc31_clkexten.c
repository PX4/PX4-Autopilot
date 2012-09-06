/****************************************************************************
 * arch/arm/src/lpc31xx/lpc31_exten.c
 *
 *   Copyright (C) 2009 Gregory Nutt. All rights reserved.
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
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#include <arch/board/board.h>

#include "lpc31_cgudrvr.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/
 
/****************************************************************************
 * Name: lpc31_enableexten
 *
 * Description:
 *   Enable external enabling for the the specified possible clocks.
 *
 ****************************************************************************/

void lpc31_enableexten(enum lpc31_clockid_e clkid)
{
  uint32_t regaddr;
  uint32_t regval;
 
  switch (clkid)
    {
      case CLKID_DMACLKGATED:      /*  9 DMA_CLK_GATED */
      case CLKID_EVENTROUTERPCLK:  /* 31 EVENT_ROUTER_PCLK */
      case CLKID_ADCPCLK:          /* 32 ADC_PCLK */
      case CLKID_IOCONFPCLK:       /* 35 IOCONF_PCLK */
      case CLKID_CGUPCLK:          /* 36 CGU_PCLK */
      case CLKID_SYSCREGPCLK:      /* 37 SYSCREG_PCLK */
      case CLKID_OTPPCLK:          /* 38 OTP_PCLK (Reserved on LPC313X) */
      case CLKID_PWMPCLKREGS:      /* 46 PWM_PCLK_REGS */
      case CLKID_PCMAPBPCLK:       /* 52 PCM_APB_PCLK */
      case CLKID_SPIPCLKGATED:     /* 57 SPI_PCLK_GATED */
      case CLKID_SPICLKGATED:      /* 90 SPI_CLK_GATED */
      case CLKID_PCMCLKIP:         /* 71 PCM_CLK_IP */
        regaddr = LPC31_CGU_PCR(clkid);
        regval  = getreg32(regaddr);
        regval |= CGU_PCR_EXTENEN;
        putreg32(regval, regaddr);
        break;

      /* Otherwise, force disable for the clocks.  NOTE that a larger set will
       * be disabled than will be enabled.
       */

      default:
        lpc31_disableexten(clkid);
        break;
    }
}

/****************************************************************************
 * Name: lpc31_disableexten
 *
 * Description:
 *   Disable external enabling for the the specified possible clocks.
 *
 ****************************************************************************/

void lpc31_disableexten(enum lpc31_clockid_e clkid)
{
  uint32_t regaddr;
  uint32_t regval;
 
  switch (clkid)
    {
      case CLKID_DMACLKGATED:      /*  9 DMA_CLK_GATED */
      case CLKID_EVENTROUTERPCLK:  /* 31 EVENT_ROUTER_PCLK */
      case CLKID_ADCPCLK:          /* 32 ADC_PCLK */
      case CLKID_WDOGPCLK:         /* 34 WDOG_PCLK */
      case CLKID_IOCONFPCLK:       /* 35 IOCONF_PCLK */
      case CLKID_CGUPCLK:          /* 36 CGU_PCLK */
      case CLKID_SYSCREGPCLK:      /* 37 SYSCREG_PCLK */
      case CLKID_OTPPCLK:          /* 38 OTP_PCLK (Reserved on LPC313X) */
      case CLKID_PWMPCLKREGS:      /* 46 PWM_PCLK_REGS */
      case CLKID_I2C0PCLK:         /* 48 I2C0_PCLK */
      case CLKID_I2C1PCLK:         /* 49 I2C1_PCLK */
      case CLKID_PCMAPBPCLK:       /* 52 PCM_APB_PCLK */
      case CLKID_UARTAPBCLK:       /* 53 UART_APB_CLK */
      case CLKID_SPIPCLKGATED:     /* 57 SPI_PCLK_GATED */
      case CLKID_SPICLKGATED:      /* 90 SPI_CLK_GATED */
      case CLKID_PCMCLKIP:         /* 71 PCM_CLK_IP */
      case CLKID_LCDPCLK:          /* 54 LCD_PCLK */
        regaddr = LPC31_CGU_PCR(clkid);
        regval  = getreg32(regaddr);
        regval &= ~CGU_PCR_EXTENEN;
        putreg32(regval, regaddr);
        break;

      default:
        break;
    }
}
