/****************************************************************************
 * board/up_network.c
 *
 *   Copyright (C) 2007, 2009 Gregory Nutt. All rights reserved.
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
 * 3. Neither the name Gregory Nutt nor the names of its contributors may be
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
#if defined(CONFIG_NET) && defined(CONFIG_NET_DM90x0)

#include <debug.h>
#include <arch/board/board.h>

#include "up_arch.h"
#include "up_internal.h"

#include "dm320_memorymap.h"
#include "dm320_emif.h"
#include "dm320_gio.h"

extern void dm9x_initialize(void);

/****************************************************************************
 * Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_netinitialize
 ****************************************************************************/

void up_netinitialize(void)
{
  /* CS4 is used for DM9000A Ethernet.  Interrupt is provided via GIO6
   * which must be configured to interrupt on the rising edge.  Bus
   * width is 16-bits.
   */

  nlldbg("CS4CTRL1=%04x CS4CTRL2=%04x\n",
         getreg16(DM320_EMIF_CS4CTRL1), getreg16(DM320_EMIF_CS4CTRL2));

  /* It is assumed that bootloader has already configured CS4.  Here,
   * we will only make certain that the GIO is properly configured
   */

  GIO_INPUT(GIO_DM9000A_INT);
  GIO_NONINVERTED(GIO_DM9000A_INT);
  GIO_INTERRUPT(GIO_DM9000A_INT);
  GIO_RISINGEDGE(GIO_DM9000A_INT);

  nlldbg("GIO DIR0=%04x INV0=%04x IRQPORT=%04x IRQEDGE=%04x\n",
         getreg16(DM320_GIO_DIR0), getreg16(DM320_GIO_INV0),
         getreg16(DM320_GIO_IRQPORT), getreg16(DM320_GIO_IRQEDGE));

  /* Then initialize the driver */

  dm9x_initialize();
}

#endif /* CONFIG_NET && CONFIG_NET_DM90x0 */
