/************************************************************************
 * arch/arm/src/lpc31xx/lpc31_bcrndx.c
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
 * Name: lpc31_bcrndx
 *
 * Description:
 *   Only 5 of the 12 domains have an associated BCR register.  This
 *   function returns the index to the associated BCR register (if any)
 *   or BCRNDX_INVALID otherwise.
 *
 ************************************************************************/

int lpc31_bcrndx(enum lpc31_domainid_e dmnid)
{
  switch (dmnid)
    {
      /* BCR0-3 correspond directly to domains 0-3 */

      case DOMAINID_SYS:        /* Domain 0: SYS_BASE */
      case DOMAINID_AHB0APB0:   /* Domain 1: AHB0APB0_BASE */
      case DOMAINID_AHB0APB1:   /* Domain 2: AHB0APB1_BASE */
      case DOMAINID_AHB0APB2:   /* Domain 3: AHB0APB2_BASE */
        return (int)dmnid;

      /* There is a BCR register corresponding to domain 7, but it is at
       * index 4
       */

      case DOMAINID_CLK1024FS: /* Domain 7: CLK1024FS_BASE */
        return 4;

      default:                 /* There is no BCR register for the other
                                * domains. */
        break;
    }
  return BCRNDX_INVALID;
}
