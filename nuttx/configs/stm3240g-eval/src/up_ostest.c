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
 * Private Functions
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/
/* Given a uint8_t array of size CONFIG_EXAMPLES_OSTEST_FPUSIZE, this
 * function will return the current FPU registers.
 */

void arch_getfpu(FAR uint8_t *fpusave)
{
  irqstate_t flags;
  uint32_t regs[XCPTCONTEXT_REGS];

  flags = irqsave();
  up_savefpu(regs);
  irqrestore(flags);

  memcpy(fpusave, &regs[REG_S0], (4*SW_FPU_REGS));
}

/* Given a uint8_t array of size CONFIG_EXAMPLES_OSTEST_FPUSIZE, this
 * function will set the current FPU regisers to match the provided
 * register save set.
 */

void arch_setfpu(FAR const uint8_t *fpusave)
{
  irqstate_t flags;
  uint32_t regs[XCPTCONTEXT_REGS];

  memcpy(&regs[REG_S0], fpusave, (4*SW_FPU_REGS));

  flags = irqsave();
  up_restorefpu(regs);
  irqrestore(flags);
}

/* Given a uint8_t array of size CONFIG_EXAMPLES_OSTEST_FPUSIZE and a
 * seed value, this function will set the FPU registers to a known
 * values for testing purposes.  The contents of the FPU registers
 * must be uniqe for each sed value.
 */

void arch_initfpu(FAR uint8_t *fpusave, int seed)
{
  FAR uint32_t *dest = (FAR uint32_t *)fpusave;
  uint32_t mask = 0x01010101;
  uint32_t incr = 0x01010101;
  int i;

  for (i = 0; i < 32; i++)
    {
      *dest = (uint32_t)seed ^ mask;
      mask += incr;
    }
}

/* Given two uint8_t arrays of size CONFIG_EXAMPLES_OSTEST_FPUSIZE this
 * function will compare then an return true if they are identical.
 */

bool arch_cmpfpu(FAR const uint8_t *fpusave1, FAR const uint8_t *fpusave2)
{
  return memcmp(fpusave1, fpusave2, (4*32)) == 0;
}

#endif /* HAVE_FPU */
