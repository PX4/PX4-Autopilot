/****************************************************************************
 * arch/arm/src/lpc31xx/lpc31_defclk.c
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
#include <stdbool.h>

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
 * Name: lpc31_defclk
 *
 * Description:
 *   Enable the specified clock if it is one of the default clocks needed
 *   by the board.
 *
 ****************************************************************************/

bool lpc31_defclk(enum lpc31_clockid_e clkid)
{

  uint32_t regaddr;
  uint32_t regval;
  bool     enable;

  /* Check if this clock should be enabled.  This is determined by
   * 3 bitsets provided by board-specific logic in board/board.h.
   */
 
  if ((int)clkid < 32)
    {
      enable = ((BOARD_CLKS_0_31 & (1 << (int)clkid)) != 0);
    }
  else if ((int)clkid < 64)
    {
      enable = ((BOARD_CLKS_32_63 & (1 << ((int)clkid - 32))) != 0);
    }
  else
    {
      enable = ((BOARD_CLKS_64_92 & (1 << ((int)clkid - 64))) != 0);
    }

  /* Then set/clear the RUN bit in the PCR register for this clock
   * accordingly.
   */

  regaddr = LPC31_CGU_PCR((int)clkid);
  regval  = getreg32(regaddr);
  if (enable)
    {
      regval |= CGU_PCR_RUN;
    }
  else
    {
      regval &= ~CGU_PCR_RUN;
    }
  putreg32(regval, regaddr);
  return enable;
}

