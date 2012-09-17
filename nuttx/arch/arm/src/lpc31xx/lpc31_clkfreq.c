/************************************************************************
 * arch/arm/src/lpc31xx/lpc31_clkfreq.c
 *
 *   Copyright (C) 2009-2010 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * References:
 *   - UM10314 LPC3130/31 User manual Rev. 1.01 — 9 September 2009
 *   - lpc313x.cdl.drivers.zip example driver code
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
 ************************************************************************/

/************************************************************************
 * Included Files
 ************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>

#include "up_arch.h"
#include "lpc31_cgudrvr.h"

/************************************************************************
 * Pre-processor Definitions
 ************************************************************************/

/************************************************************************
 * Private Data
 ************************************************************************/

/************************************************************************
 * Private Functions
 ************************************************************************/

/************************************************************************
 * Public Functions
 ************************************************************************/

/************************************************************************
 * Name: lpc31_fdcndx
 *
 * Description:
 *   Given a clock ID and its domain ID, return the frequency of the
 *   clock.
 *
 ************************************************************************/

uint32_t lpc31_clkfreq(enum lpc31_clockid_e clkid,
                         enum lpc31_domainid_e dmnid)
{
  uint32_t freq = 0;
  uint32_t fdcndx;
  uint32_t regval;

  /* Get then fractional divider register index for this clock */

  fdcndx = lpc31_fdcndx(clkid, dmnid);

  /* Get base frequency for the domain */

  freq = lpc31_getbasefreq(dmnid);

  /* If there is no fractional divider associated with the clodk, then the
   * connection is directo and we just return the base frequency.
   */

  if (fdcndx == FDCNDX_INVALID)
    {
      return freq;
    }

  /* Read fractional divider control (FDC) register value and double check that
   * it is enabled (not necessary since lpc31_fdcndx() also does this check
   */

  regval = getreg32(LPC31_CGU_FDC(fdcndx));
  if ((regval & CGU_ESR_ESREN) != 0)
    {
      int32_t msub;
      int32_t madd;
      int32_t n;
      int32_t m;

      /* Yes, extract modulo subtraction and addition values, msub and madd.
       * Fractional divider 17 is a special case because its msub and madd
       * fields have greater range.
       */

      if (fdcndx == 17)
        {
          /* Range is 0-0x1fff for both */

          msub = ((regval & CGU_FDC17_MSUB_MASK) >> CGU_FDC17_MSUB_SHIFT) | CGU_FDC17_MSUB_EXTEND;
          madd = (regval & CGU_FDC17_MADD_MASK) >> CGU_FDC17_MADD_SHIFT;
        }
      else
        {
          /* Range is 0-255 for both */

          msub = ((regval & CGU_FDC_MSUB_MASK) >> CGU_FDC_MSUB_SHIFT) | CGU_FDC_MSUB_EXTEND;
          madd = (regval & CGU_FDC_MADD_MASK) >> CGU_FDC_MADD_SHIFT;
        }

      /* Handle a corner case that would result in an infinite loop below */

      if (msub == 0 && madd == 0)
        {
          return 0;
        }

      /* Reduce to the greatest common power-of-2 denominator.  To minimize
       * power consumption, the lpc313x user manual recommends that madd and msub
       * be shifted right to have as many trailing zero's as possible.  The
       * following undoes that shift.
       */

      while ((msub & 1) == 0 && (madd & 1) == 0)
        {
          madd = madd >> 1;
          msub = msub >> 1;
        }

      /* Then compute n and m values:
       *
       *   fout = n/m * fin
       *
       * where
       *
       *   madd = m - n
       *   msub = -n
       */

      n = -msub;
      m = madd + n;

      /* Check that both m and n are non-zero values */

      if ((n == 0) || (m == 0))
        {
          return 0;
        }

     /* Finally, calculate the frequency based on m and n values */

    freq = (freq * n) / m ;
  }

  return  freq;
}

