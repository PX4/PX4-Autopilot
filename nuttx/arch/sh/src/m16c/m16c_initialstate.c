/****************************************************************************
 *  arch/sh/src/m16c/m16c_initialstate.c
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

#include <nuttx/arch.h>
#include <arch/board/board.h>

#include "up_internal.h"
#include "up_arch.h"

/****************************************************************************
 * Preprocessor Definitions
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
 * Name: up_initial_state
 *
 * Description:
 *   A new thread is being started and a new TCB has been created. This
 *   function is called to initialize the processor specific portions of the
 *   new TCB.
 *
 *   This function must setup the intial architecture registers and/or stack
 *   so that execution will begin at tcb->start on the next context switch.
 *
 ****************************************************************************/

void up_initial_state(FAR _TCB *tcb)
{
  FAR struct xcptcontext *xcp  = &tcb->xcp;
  FAR uint8_t            *regs = xcp->regs;

  /* Initialize the initial exception register context structure */

  memset(xcp, 0, sizeof(struct xcptcontext));

  /* Offset 0: FLG (bits 12-14) PC (bits 16-19) as would be present by an interrupt */

  *regs++ = ((M16C_DEFAULT_IPL << 4) | ((uint32_t)tcb->start >> 16));

  /* Offset 1: FLG (bits 0-7) */

#ifdef CONFIG_SUPPRESS_INTERRUPTS
  *regs++ = M16C_FLG_U;
#else
  *regs++ = M16C_FLG_U | M16C_FLG_I;
#endif

  /* Offset 2-3: 16-bit PC [0]:bits8-15 [1]:bits 0-7 */

  *regs++ = (uint32_t)tcb->start >> 8;  /* Bits 8-15 of PC */
  *regs++ = (uint32_t)tcb->start;       /* Bits 0-7 of PC */

  /* Offset 18-20: User stack pointer */

   regs   = &xcp->regs[REG_SP];
  *regs++ = (uint32_t)tcb->adj_stack_ptr >> 8;  /* Bits 8-15 of SP */
  *regs   = (uint32_t)tcb->adj_stack_ptr;       /* Bits 0-7 of SP */
}
