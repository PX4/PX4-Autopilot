/****************************************************************************
 * arch/arm/src/lpc31xx/lpc31_fdivinit.c
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

#include "lpc31_cgu.h"
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
 * Name: lpc31_bitwidth
 *
 * Description:
 *   Find the bit width of a msub or madd value.  This will be use to
 *   extend the msub or madd values.  To minimize power consumption, the
 *   lpc313x user manual recommends that madd and msub be shifted right
 *   to have as many trailing zero's as possible.  This function detmines
 *   the pre-shifted with of one of the msub or madd values.
 *
 * EXAMPLE:
 *
 *  Say an input frequency of 13 MHz is given while a frequency of 12
 *  MHz is required. In this case we want a frequency
 *
 *    f’ = 12/13 × f
 *
 *  So n = 12 and m = 13. This then gives
 *
 *    madd = m - n = 13 - 12 = 1
 *    msub = -n = -12
 *
 * In order to minimize power consumption madd and msub must be as
 * large as possible. The limit of their values is determined by the
 * madd/msub bit width. In this case msub is the largest value,
 * in order to express -12, five bits are required. However since msub is
 * always negative the fractional divider does not need the sign bit, leaving
 * 4 bits. If madd/msub bit width has been set to say 8 bits, it is allowed
 * to shift 4 bits, giving:
 *
 *   msub’ = -(12<<4)= -12 × 24 = -12 × 16 = -192
 *   madd’ = 1<<4 = 24 = 16
 *
 ****************************************************************************/

static inline unsigned int
lpc31_bitwidth(unsigned int value, unsigned int fdwid)
{
  unsigned int width = 0;
  int bit;

  /* Examine bits from the most significant down */

  for (bit = fdwid-1; bit >= 0; bit--)
    {
      /* Is this bit set?  If so, then the width of the value is 0 to bit,
       * or bit+1.
       */
    
      if ((value & (1 << bit)) != 0)
        {
          width = bit + 1;
          break;
        }
    }
  return width;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/
/****************************************************************************
 * Name: lpc31_fdivinit
 *
 * Description:
 *   Enable and configure (or disable) a fractional divider.
 *
 ****************************************************************************/

uint32_t lpc31_fdivinit(int fdcndx,
                          const struct lpc31_fdivconfig_s *fdiv, bool enable)
{
  uint32_t     regaddr;
  uint32_t     regval;
  unsigned int fdshift;
  unsigned int fdwid;
  unsigned int fdmask;
  unsigned int maddshift;
  unsigned int msubshift;
  int          madd;
  int          msub;

  /* Calculating the (unshifted) divider values.To minimize power
   * consumption, the lpc313x user manual recommends that madd and msub
   * be shifted right to have as many trailing zero's as possible.
   */
 
  madd = fdiv->m - fdiv->n;
  msub = -fdiv->n;

  /* Determine the width of the madd and msub fields in the fractional divider
   * register.  They are all 8-bits in width except for fractional divider 17.
   */

  fdwid     = CGU_FDC_FIELDWIDTH;
  maddshift = CGU_FDC_MADD_SHIFT;
  msubshift = CGU_FDC_MSUB_SHIFT;

  if (fdcndx == 17)
    {
      /* For fractional divider 17, the msub/madd field width is 13 */

      fdwid     = CGU_FDC17_FIELDWIDTH;
      maddshift = CGU_FDC17_MADD_SHIFT;
      msubshift = CGU_FDC17_MSUB_SHIFT;
    }

  /* Find maximum bit width of madd & msub.  Here we calculate the width of the OR
   * of the two values.  The width of the OR will be the width of the wider value 
   */

  fdshift = fdwid - lpc31_bitwidth((unsigned int)madd | (unsigned int)fdiv->n, fdwid);

  /* Calculate the fractional divider register values */

  fdmask = (1 << fdwid) - 1;
  madd   = (madd << fdshift) & fdmask;
  msub   = (msub << fdshift) & fdmask;
  regval = (madd << maddshift) | (msub << msubshift);
  
  /* Check if 50% duty cycle is needed for this divider */

  if (fdiv->stretch)
    {
      regval |= CGU_FDC_STRETCH;
    }

  /* Check if we should enable the divider immediately */

  if (enable)
    {
      regval |= CGU_FDC_RUN;
    }

  /* Finally configure the divider */

  regaddr = LPC31_CGU_FDC(fdcndx);
  putreg32(regval, regaddr);
  return regval;
}
