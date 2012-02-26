/************************************************************************************
 * configs/stm3240g-eval/src/up_ostest.c
 * arch/arm/src/board/up_ostest.c
 *
 *   Copyright (C) 2012 Gregory Nutt. All rights reserved.
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
 ************************************************************************************/

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <debug.h>

#include <arch/irq.h>
#include <arch/board/board.h>

#include "up_arch.h"
#include "up_internal.h"
#include "stm3240g-internal.h"

/************************************************************************************
 * Definitions
 ************************************************************************************/
/* Configuration ********************************************************************/

#undef HAVE_FPU
#if defined(CONFIG_ARCH_FPU) && defined(CONFIG_EXAMPLES_OSTEST_FPUSIZE) && \
    defined(CONFIG_SCHED_WAITPID) && !defined(CONFIG_DISABLE_SIGNALS) && \
   !defined(CONFIG_ARMV7M_CMNVECTOR)
#    define HAVE_FPU 1
#endif

#ifdef HAVE_FPU

#if CONFIG_EXAMPLES_OSTEST_FPUSIZE != (4*SW_FPU_REGS)
#  error "CONFIG_EXAMPLES_OSTEST_FPUSIZE has the wrong size"
#endif

/************************************************************************************
 * Private Data
 ************************************************************************************/

static uint32_t g_saveregs[XCPTCONTEXT_REGS];

/************************************************************************************
 * Private Functions
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/
/* Given an array of size CONFIG_EXAMPLES_OSTEST_FPUSIZE, this function will return
 * the current FPU registers.
 */

void arch_getfpu(FAR uint32_t *fpusave)
{
  irqstate_t flags;

  /* Take a snapshot of the thread context right now */

  flags = irqsave();
  up_saveusercontext(g_saveregs);

  /* Return only the floating register values */

  memcpy(fpusave, &g_saveregs[REG_S0], (4*SW_FPU_REGS));
  irqrestore(flags);
}

/* Given two arrays of size CONFIG_EXAMPLES_OSTEST_FPUSIZE this function
 * will compare them and return true if they are identical.
 */

bool arch_cmpfpu(FAR const uint32_t *fpusave1, FAR const uint32_t *fpusave2)
{
  return memcmp(fpusave1, fpusave2, (4*SW_FPU_REGS)) == 0;
}

#endif /* HAVE_FPU */
