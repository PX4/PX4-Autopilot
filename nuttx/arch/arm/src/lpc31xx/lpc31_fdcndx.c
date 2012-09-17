/************************************************************************
 * arch/arm/src/lpc31xx/lpc31_fdcndx.c
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

/* The select register in the ESR registers vary in width from 1-3 bits.
 * Below is a macro to select the widest case (which is OK because the
 * undefined bits will be read as zero).  Within the field, bits 0-7 to
 * indicate the offset from the base FDC index.
 */

#define CGU_ESRSEL(n)       (((n)>>1)&7)

/************************************************************************
 * Private Data
 ************************************************************************/

static const uint8_t g_fdcbase[CGU_NDOMAINS] =
{
  FRACDIV_BASE0_LOW,  /* Domain 0: SYS_BASE */
  FRACDIV_BASE1_LOW,  /* Domain 1: AHB0APB0_BASE */
  FRACDIV_BASE2_LOW,  /* Domain 2: AHB0APB1_BASE */
  FRACDIV_BASE3_LOW,  /* Domain 3: AHB0APB2_BASE */
  FRACDIV_BASE4_LOW,  /* Domain 4: AHB0APB3_BASE */
  FRACDIV_BASE5_LOW,  /* Domain 5: PCM_BASE */
  FRACDIV_BASE6_LOW,  /* Domain 6: UART_BASE */
  FRACDIV_BASE7_LOW,  /* Domain 7: CLK1024FS_BASE */
  0,                  /* Domain 8: BCK0_BASE (no ESR register) */
  0,                  /* Domain 9: BCK1_BASE (no ESR register)  */
  FRACDIV_BASE10_LOW, /* Domain 10: SPI_BASE */
  0,                  /* Domain 11: SYSCLKO_BASE (no ESR register) */
};

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
 *   Given a clock ID and its domain ID, return the index of the
 *   corresponding fractional divider register (or FDCNDX_INVALID if
 *   there is no fractional divider associated with this clock).
 *
 ************************************************************************/

int lpc31_fdcndx(enum lpc31_clockid_e clkid, enum lpc31_domainid_e dmnid)
{
  int esrndx;
  int fdcndx = FDCNDX_INVALID;

  /* Check if there is an ESR register associate with this clock ID */

  esrndx = lpc31_esrndx(clkid);
  if (esrndx != ESRNDX_INVALID)
  {
    /* Read the clock's ESR to get the fractional divider */

    uint32_t regval = getreg32(LPC31_CGU_ESR(esrndx));

    /* Check if any fractional divider is enabled for this clock. */

    if ((regval & CGU_ESR_ESREN) != 0)
    {
      /* Yes.. The FDC index is an offset from this fractional
       * divider base for this domain.
       */

      fdcndx = CGU_ESRSEL(regval) + (int)g_fdcbase[dmnid];
    }
  }
  return fdcndx;
}

