/****************************************************************************
 * arch/arm/src/armv7-m/up_memfault.c
 *
 *   Copyright (C) 2011 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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

#include <assert.h>
#include <debug.h>

#include <arch/irq.h>

#include "up_arch.h"
#include "os_internal.h"
#include "nvic.h"
#include "up_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#undef DEBUG_MEMFAULTS         /* Define to debug memory management faults */

#ifdef DEBUG_MEMFAULTS
# define mfdbg(format, arg...) lldbg(format, ##arg)
#else
# define mfdbg(x...)
#endif

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
 * Name: up_memfault
 *
 * Description:
 *   This is Memory Management Fault exception handler.  Normally we get here
 *   when the Cortex M3 MPU is enabled and an MPU fault is detected.  However,
 *   I understand that there are other error conditions that can also generate
 *   memory management faults.
 *
 ****************************************************************************/

int up_memfault(int irq, FAR void *context)
{
  /* Dump some memory management fault info */

  (void)irqsave();
  lldbg("PANIC!!! Memory Management Fault:\n");
  mfdbg("  IRQ: %d context: %p\n", irq, regs);
  lldbg("  CFAULTS: %08x MMFAR: %08x\n",
        getreg32(NVIC_CFAULTS), getreg32(NVIC_MEMMANAGE_ADDR));
  mfdbg("  R0: %08x %08x %08x %08x %08x %08x %08x %08x\n",
        regs[REG_R0],  regs[REG_R1],  regs[REG_R2],  regs[REG_R3],
        regs[REG_R4],  regs[REG_R5],  regs[REG_R6],  regs[REG_R7]);
  mfdbg("  R8: %08x %08x %08x %08x %08x %08x %08x %08x\n",
        regs[REG_R8],  regs[REG_R9],  regs[REG_R10], regs[REG_R11],
        regs[REG_R12], regs[REG_R13], regs[REG_R14], regs[REG_R15]);
  mfdbg("  PSR=%08x\n", regs[REG_XPSR]);

  PANIC(OSERR_UNEXPECTEDISR);
  return OK;
}
