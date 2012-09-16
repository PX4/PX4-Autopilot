/****************************************************************************
 * arch/avr/src/avr32/up_internal.h
 *
 *   Copyright (C) 2011-2012 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_AVR_SRC_AVR32_AVR32_INTERNAL_H
#define __ARCH_AVR_SRC_AVR32_AVR32_INTERNAL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifndef __ASSEMBLY__
#  include <nuttx/compiler.h>
#  include <stdint.h>
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Macros to handle saving and restore interrupt state.  The state is copied
 * from the stack to the TCB, but only a referenced is passed to get the 
 * state from the TCB.
 */

#define up_savestate(regs)    up_copystate(regs, (uint32_t*)current_regs)
#define up_restorestate(regs) (current_regs = regs)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Variables
 ****************************************************************************/

#ifndef __ASSEMBLY__
/* This holds a references to the current interrupt level register storage
 * structure.  If is non-NULL only during interrupt processing.
 */

extern volatile uint32_t *current_regs;

/* This is the beginning of heap as provided from up_head.S. This is the first
 * address in DRAM after the loaded program+bss+idle stack.  The end of the
 * heap is CONFIG_DRAM_END
 */

extern uint32_t g_heapbase;

#endif /* __ASSEMBLY__ */

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#ifndef __ASSEMBLY__

/************************************************************************************
 * Name:  up_copystate
 *
 * Description:
 *   Copy the contents of a register state save structure from one location to
 *   another.
 *
 ************************************************************************************/

extern void up_copystate(uint32_t *dest, uint32_t *src);

/************************************************************************************
 * Name:  up_fullcontextrestore
 *
 * Description:
 *   Restore the full context of a saved thread/task.
 *
 ************************************************************************************/

extern void up_fullcontextrestore(uint32_t *restoreregs) noreturn_function;

/************************************************************************************
 * Name:  up_switchcontext
 *
 * Description:
 *   Switch from one thread/task context to another.
 *
 ************************************************************************************/

extern void up_switchcontext(uint32_t *saveregs, uint32_t *restoreregs);

/************************************************************************************
 * Name:  up_doirq
 *
 * Description:
 *   Dispatch an interrupt.
 *
 ************************************************************************************/

extern uint32_t *up_doirq(int irq, uint32_t *regs);

#endif /* __ASSEMBLY__ */
#endif  /* __ARCH_AVR_SRC_AVR32_AVR32_INTERNAL_H */

