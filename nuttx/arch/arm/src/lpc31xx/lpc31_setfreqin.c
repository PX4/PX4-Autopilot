/****************************************************************************
 * arch/arm/src/lpc31xx/lpc31_setfreqin.c
 *
 *   Copyright (C) 2009-2010 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * References:
 *   - NXP UM10314 LPC3130/31 User manual Rev. 1.01 — 9 September 2009
 *   - NXP lpc313x.cdl.drivers.zip example driver code
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
 * Name: lpc31_selectfreqin
 *
 * Description:
 *   Set the base frequency source selection for with a clock domain
 *
 ****************************************************************************/

void lpc31_selectfreqin(enum lpc31_domainid_e dmnid, uint32_t finsel)
{
  uint32_t scraddr = LPC31_CGU_SCR(dmnid);
  uint32_t fs1addr = LPC31_CGU_FS1(dmnid);
  uint32_t fs2addr = LPC31_CGU_FS2(dmnid);
  uint32_t scrbits;

  /* Get the frequency selection from the switch configuration register (SCR)
   * for this domain.
   */

  scrbits = getreg32(scraddr) & ~(CGU_SCR_ENF1|CGU_SCR_ENF2);

  /* If FS1 is currently enabled set the reference clock to FS2 and enable FS2 */

  if ((getreg32(LPC31_CGU_SSR(dmnid)) & CGU_SSR_FS1STAT) != 0)
    {
      /* Check if the selected frequency, FS1, is same as requested */

      if ((getreg32(fs1addr) & CGU_FS_MASK) != finsel)
        {
          /* No.. Set up FS2 */

          putreg32(finsel, fs2addr);
          putreg32(scrbits | CGU_SCR_ENF2, scraddr);
        }
    }

  /* FS1 is not currently enabled, check if the selected frequency, FS2,
   * is same as requested
   */

  else if ((getreg32(fs2addr) & CGU_FS_MASK) != finsel)
    {
      /* No.. Set up FS1 */

      putreg32(finsel, fs1addr);
      putreg32(scrbits | CGU_SCR_ENF1, scraddr);
    }
}
