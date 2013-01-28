/****************************************************************************
 * arch/z80/src/ez80/ez80_registerdump.c
 *
 *   Copyright (C) 2008-2009 Gregory Nutt. All rights reserved.
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

#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>

#include "chip/switch.h"
#include "os_internal.h"
#include "up_internal.h"

/****************************************************************************
 * Definitions
 ****************************************************************************/

/* Output debug info if stack dump is selected -- even if 
 * debug is not selected.
 */

#ifdef CONFIG_ARCH_STACKDUMP
# undef  lldbg
# define lldbg lowsyslog
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: z80_registerdump
 ****************************************************************************/

#ifdef CONFIG_ARCH_STACKDUMP
static void ez80_registerdump(void)
{
  if (current_regs)
    {
#ifdef CONFIG_EZ80_Z80MODE
      lldbg("AF: %04x  I: %04x\n",
            current_regs[XCPT_AF], current_regs[XCPT_I]);
      lldbg("BC: %04x DE: %04x HL: %04x\n",
            current_regs[XCPT_BC], current_regs[XCPT_DE], current_regs[XCPT_HL]);
      lldbg("IX: %04x IY: %04x\n",
            current_regs[XCPT_IX], current_regs[XCPT_IY]);
      lldbg("SP: %04x PC: %04x\n"
            current_regs[XCPT_SP], current_regs[XCPT_PC]);
#else
      lldbg("AF: %06x  I: %06x\n",
            current_regs[XCPT_AF], current_regs[XCPT_I]);
      lldbg("BC: %06x DE: %06x HL: %06x\n",
            current_regs[XCPT_BC], current_regs[XCPT_DE], current_regs[XCPT_HL]);
      lldbg("IX: %06x IY: %06x\n",
            current_regs[XCPT_IX], current_regs[XCPT_IY]);
      lldbg("SP: %06x PC: %06x\n"
            current_regs[XCPT_SP], current_regs[XCPT_PC]);
#endif
    }
}
#endif
