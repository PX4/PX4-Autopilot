/****************************************************************************
 * arch/arm/src/armv7-m/up_hardfault.c
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
#include <string.h>
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

/* Debug output from this file may interfere with context switching! */

#ifdef CONFIG_DEBUG_HARDFAULT
# define hfdbg(format, arg...) lldbg(format, ##arg)
#else
# define hfdbg(x...)
#endif

#define INSN_SVC0        0xdf00 /* insn: svc 0 */

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
 * Name: up_hardfault
 *
 * Description:
 *   This is Hard Fault exception handler.  It also catches SVC call
 *   exceptions that are performed in bad contexts.
 *
 ****************************************************************************/

int up_hardfault(int irq, FAR void *context)
{
  uint32_t *regs = (uint32_t*)context;

  /* Get the value of the program counter where the fault occurred */

#ifndef CONFIG_ARMV7M_USEBASEPRI
  uint16_t *pc = (uint16_t*)regs[REG_PC] - 1;
  if ((void*)pc >= (void*)&_stext && (void*)pc < (void*)&_etext)
    {
      /* Fetch the instruction that caused the Hard fault */

      uint16_t insn = *pc;
      hfdbg("  PC: %p INSN: %04x\n", pc, insn);

      /* If this was the instruction 'svc 0', then forward processing
       * to the SVCall handler
       */

      if (insn == INSN_SVC0)
        {
          hfdbg("Forward SVCall\n");
          return up_svcall(irq, context);
        }
    }
#endif

  /* Dump some hard fault info */

  hfdbg("\nHard Fault:\n");
  hfdbg("  IRQ: %d regs: %p\n", irq, regs);
  hfdbg("  BASEPRI: %08x PRIMASK: %08x IPSR: %08x\n",
        getbasepri(), getprimask(), getipsr());
  hfdbg("  CFAULTS: %08x HFAULTS: %08x DFAULTS: %08x BFAULTADDR: %08x AFAULTS: %08x\n",
        getreg32(NVIC_CFAULTS), getreg32(NVIC_HFAULTS),
        getreg32(NVIC_DFAULTS), getreg32(NVIC_BFAULT_ADDR),
        getreg32(NVIC_AFAULTS));
  hfdbg("  R0: %08x %08x %08x %08x %08x %08x %08x %08x\n",
        regs[REG_R0],  regs[REG_R1],  regs[REG_R2],  regs[REG_R3],
        regs[REG_R4],  regs[REG_R5],  regs[REG_R6],  regs[REG_R7]);
  hfdbg("  R8: %08x %08x %08x %08x %08x %08x %08x %08x\n",
        regs[REG_R8],  regs[REG_R9],  regs[REG_R10], regs[REG_R11],
        regs[REG_R12], regs[REG_R13], regs[REG_R14], regs[REG_R15]);
  hfdbg("  PSR=%08x\n", regs[REG_XPSR]);

  (void)irqsave();
  lldbg("PANIC!!! Hard fault: %08x\n", getreg32(NVIC_HFAULTS));
  PANIC(OSERR_UNEXPECTEDISR);
  return OK;
}
