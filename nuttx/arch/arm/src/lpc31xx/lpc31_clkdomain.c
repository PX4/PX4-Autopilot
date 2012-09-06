/************************************************************************
 * arch/arm/src/lpc31xx/lpc31_clkdomain.c
 *
 *   Copyright (C) 2009 Gregory Nutt. All rights reserved.
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
 * Name: lpc31_clkdomain
 *
 * Description:
 *   Given a clock ID, return the ID of the domain in which the clock
 *   resides.
 *
 ************************************************************************/

enum lpc31_domainid_e lpc31_clkdomain(enum lpc31_clockid_e clkid)
{
  if (clkid <= CLKID_SYSBASE_LAST)             /* Domain 0: SYS_BASE */
    {
      return DOMAINID_SYS;
    }
  else if (clkid <= CLKID_AHB0APB0_LAST)       /* Domain 1: AHB0APB0_BASE */
    {
      return DOMAINID_AHB0APB0;
    }
  else if (clkid <= CLKID_AHB0APB1_LAST)       /* Domain 2: AHB0APB1_BASE */
    {
      return DOMAINID_AHB0APB1;
    }
  else if (clkid <= CLKID_AHB0APB2_LAST)       /* Domain 3: AHB0APB2_BASE */
    {
      return DOMAINID_AHB0APB2;
    }
  else if (clkid <= CLKID_AHB0APB3_LAST)       /* Domain 4: AHB0APB3_BASE */
    {
      return DOMAINID_AHB0APB3;
    }
  else if (clkid <= CLKID_PCM_LAST)            /* Domain 5: PCM_BASE */
    {
      return DOMAINID_PCM;
    }
  else if (clkid <= CLKID_UART_LAST)           /* Domain 6: UART_BASE */
    {
      return DOMAINID_UART;
    }
  else if (clkid <= CLKID_CLK1024FS_LAST)      /* Domain 7: CLK1024FS_BASE */
    {
      return DOMAINID_CLK1024FS;
    }
  else if (clkid <= CLKID_I2SRXBCK0_LAST)      /* Domain 8: BCK0_BASE */
    {
      return DOMAINID_BCK0;
    }
  else if (clkid <= CLKID_I2SRXBCK1_LAST)      /* Domain 9: BCK1_BASE */
    {
      return DOMAINID_BCK1;
    }
  else if (clkid <= CLKID_SPI_LAST)            /* Domain 10: SPI_BASE */
    {
      return DOMAINID_SPI;
    }
  else /* if (clkid <= CLKID_SYSCLKO_LAST) */  /* Domain 11: SYSCLKO_BASE */
    {
      return DOMAINID_SYSCLKO;
    }
}
