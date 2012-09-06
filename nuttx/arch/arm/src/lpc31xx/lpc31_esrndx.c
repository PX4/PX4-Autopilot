/************************************************************************
 * arch/arm/src/lpc31xx/lpc31_esrndx.c
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
 * Name: lpc31_esrndx
 *
 * Description:
 *   Given a clock ID, return the index of the corresponding ESR
 *   register (or ESRNDX_INVALID if there is no ESR associated with
 *   this clock ID).  Indexing of ESRs differs slightly from the clock
 *   ID:  There are 92 clock IDs but only 89 ESR regisers. There are no
 *  ESR registers for :
 *
 *
 *  CLKID_I2SRXBCK0         Clock ID 87: I2SRX_BCK0
 *  CLKID_I2SRXBCK1,        Clock ID 88: I2SRX_BCK1
 *
 * and
 *
 *  CLKID_SYSCLKO           Clock ID 91: SYSCLK_O
 *
 ************************************************************************/

int lpc31_esrndx(enum lpc31_clockid_e clkid)
{
  int esrndx = (int)clkid;

  /* There ar 89 Enable Select Registers (ESR).  Indexing for these
   * registers is identical to indexing to other registers (like PCR),
   * except that there are no ESR registers for 
   *
   *
   *  CLKID_I2SRXBCK0         Clock ID 87: I2SRX_BCK0
   *  CLKID_I2SRXBCK1,        Clock ID 88: I2SRX_BCK1
   *
   * and
   *
   *  CLKID_SYSCLKO           Clock ID 91: SYSCLK_O
   */

  switch (clkid)
  {
    /* There are no ESR registers corresponding to the following
     * three clocks:
     */

    case CLKID_I2SRXBCK0:
    case CLKID_I2SRXBCK1:
    case CLKID_SYSCLKO:
      esrndx = ESRNDX_INVALID;
      break;

    /* These clock IDs are a special case and need to be adjusted
     * by two:
     *
     * CLKID_SPICLK          Clock ID 89, ESR index 87
     * CLKID_SPICLKGATED     Clock ID 90, ESR index 88
     */

    case CLKID_SPICLK:
    case CLKID_SPICLKGATED:
      esrndx = esrndx - 2;
      break;

    /* The rest of the indices match up and we don't have to do anything. */

    default:
      break;
  }

  return esrndx;
}
