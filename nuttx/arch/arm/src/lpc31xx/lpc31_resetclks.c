/****************************************************************************
 * arch/arm/src/lpc31xx/lpc31_resetclks.c
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
 * Name: lpc31_resetclks
 *
 * Description:
 *   Put all clocks into a known, initial state
 *
 ****************************************************************************/

void lpc31_resetclks(void)
{
  uint32_t regaddr;
  uint32_t regval;
  int bcrndx;
  int esrndx;
  int i;

  /* Switch all domain reference clocks to FFAST */

  for (i = 0; i < CGU_NDOMAINS; i++)
    {
      /* Switch reference clock in to FFAST */

      lpc31_selectfreqin((enum lpc31_domainid_e)i, CGU_FS_FFAST);

      /* Check if the domain has a BCR */

      bcrndx = lpc31_bcrndx((enum lpc31_domainid_e)i);
      if (bcrndx != BCRNDX_INVALID)
        {
          /* Yes.. disable all BCRs */

          putreg32(0, LPC31_CGU_BCR(bcrndx));
        }
    }

  /* Disable all clocks except those that are necessary */

  for (i = CLKID_FIRST; i <= CLKID_LAST; i++)
  {
    /* Check if this clock has an ESR register */

    esrndx = lpc31_esrndx((enum lpc31_clockid_e)i);
    if (esrndx != ESRNDX_INVALID)
    {
      /* Yes.. Clear the clocks ESR to deselect fractional divider */

      putreg32(0, LPC31_CGU_ESR(esrndx));
    }

    /* Enable external enabling for all possible clocks to conserve power */

    lpc31_enableexten((enum lpc31_clockid_e)i);

    /* Set enable-out's for only the following clocks */

    regaddr = LPC31_CGU_PCR(i);
    regval  = getreg32(regaddr);
    if (i == (int)CLKID_ARM926BUSIFCLK || i == (int)CLKID_MPMCCFGCLK)
      {
        regval |=  CGU_PCR_ENOUTEN;
      }
    else
      {
        regval &= ~CGU_PCR_ENOUTEN;
      }
    putreg32(regval, regaddr);

    /* Set/clear the RUN bit in the PCR regiser of  all clocks, depending
     * upon if the clock is needed by the board logic or not
     */

    (void)lpc31_defclk((enum lpc31_clockid_e)i);
  }

  /* Disable all fractional dividers */

  for (i = 0; i < CGU_NFRACDIV; i++)
    {
      regaddr = LPC31_CGU_FDC(i);
      regval  = getreg32(regaddr);
      regval &= ~CGU_FDC_RUN;
      putreg32(regval, regaddr);
    }
}
