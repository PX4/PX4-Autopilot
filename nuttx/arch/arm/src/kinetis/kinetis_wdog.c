/****************************************************************************
 * arch/arm/src/kinetis/kinetis_wdog.c
 * arch/arm/src/chip/kinetis_wdog.c
 *
 *   Copyright (C) 2011 Gregory Nutt. All rights reserved.
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

#include <arch/irq.h>

#include "up_arch.h"
#include "kinetis_internal.h"
#include "kinetis_wdog.h"

/****************************************************************************
 * Private Definitions
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
 * Name: kinetis_wdunlock
 *
 * Description:
 *   Watchdog timer unlock routine. Writing 0xc520 followed by 0xd928 will
 *   unlock the write once registers in the WDOG so they are writable
 *   within the WCT period.
 *
 ****************************************************************************/

static void kinetis_wdunlock(void)
{
  irqstate_t flags;

  /* This sequence must execute within 20 clock cycles.  Disable interrupts
   * to assure that the following steps are atomic.
   */

  flags = irqsave();

  /* Write 0xC520 followed by 0xD928 to the unlock register */

  putreg16(0xc520, KINETIS_WDOG_UNLOCK);
  putreg16(0xd928, KINETIS_WDOG_UNLOCK);
  irqrestore(flags);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: kinetis_wddisable
 *
 * Description:
 *   Disable the watchdog timer
 *
 ****************************************************************************/

void kinetis_wddisable(void)
{
  uint16_t regval;

  /* Unlock the watchdog so that we can write to registers */

  kinetis_wdunlock();
	
  /* Clear the WDOGEN bit to disable the watchdog */

  regval  = getreg16(KINETIS_WDOG_STCTRLH);
  regval &= ~WDOG_STCTRLH_WDOGEN;
  putreg16(regval, KINETIS_WDOG_STCTRLH);
}
