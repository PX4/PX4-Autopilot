/************************************************************************************
 * arch/z80/src/z180/switch.h
 * arch/z80/src/chip/switch.h
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

#ifndef __ARCH_Z80_SRC_Z180_SWITCH_H
#define __ARCH_Z80_SRC_Z180_SWITCH_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/sched.h>
#include <nuttx/arch.h>

#include "up_internal.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Macros for portability ***********************************************************
 *
 * Common logic in arch/z80/src/common is customized for the z180 context switching
 * logic via the following macros.
 */

/* Initialize the IRQ state */

#define INIT_IRQCONTEXT()        current_regs = NULL

/* IN_INTERRUPT returns true if the system is currently operating in the interrupt
 * context.  IN_INTERRUPT is the inline equivalent of up_interrupt_context().
 */

#define IN_INTERRUPT()           (current_regs != NULL)

/* The following macro is used when the system enters interrupt handling logic */

#define DECL_SAVESTATE() \
  FAR chipreg_t *savestate

#define IRQ_ENTER(irq, regs) \
  do { \
    savestate    = (FAR chipreg_t *)current_regs; \
    current_regs = (regs); \
  } while (0)

/* The following macro is used when the system exits interrupt handling logic */

#define IRQ_LEAVE(irq)           current_regs = savestate

/* The following macro is used to sample the interrupt state (as a opaque handle) */

#define IRQ_STATE()              (current_regs)

/* Save the current IRQ context in the specified TCB */

#define SAVE_IRQCONTEXT(tcb)     z180_copystate((tcb)->xcp.regs, (FAR chipreg_t*)current_regs)

/* Set the current IRQ context to the state specified in the TCB */

#define SET_IRQCONTEXT(tcb)      z180_copystate((FAR chipreg_t*)current_regs, (tcb)->xcp.regs)

/* Save the user context in the specified TCB.  User context saves can be simpler
 * because only those registers normally saved in a C called need be stored.
 */

#define SAVE_USERCONTEXT(tcb)    z180_saveusercontext((tcb)->xcp.regs)

/* Restore the full context -- either a simple user state save or the full,
 * IRQ state save.
 */

#define RESTORE_USERCONTEXT(tcb) z180_restoreusercontext((tcb)->xcp.regs)

/* Dump the current machine registers */

#define _REGISTER_DUMP()         z180_registerdump()

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Variables
 ************************************************************************************/

#ifndef __ASSEMBLY__
/* This holds a references to the current interrupt level register storage structure.
 * If is non-NULL only during interrupt processing.
 */

extern volatile chipreg_t *current_regs;
#endif

/************************************************************************************
 * Public Function Prototypes
 ************************************************************************************/

#ifndef __ASSEMBLY__
#ifdef __cplusplus
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

/* Defined in z180_copystate.c */

EXTERN void z180_copystate(FAR chipreg_t *dest, FAR const chipreg_t *src);

/* Defined in z180_saveusercontext.asm */

EXTERN int z180_saveusercontext(FAR chipreg_t *regs);

/* Defined in z180_restoreusercontext.asm */

EXTERN void z180_restoreusercontext(FAR chipreg_t *regs);

/* Defined in z180_sigsetup.c */

EXTERN void z180_sigsetup(FAR _TCB *tcb, sig_deliver_t sigdeliver, FAR chipreg_t *regs);

/* Defined in z180_registerdump.c */

EXTERN void z180_registerdump(void);

#undef EXTERN
#ifdef __cplusplus
}
#endif
#endif

#endif  /* __ARCH_Z80_SRC_Z180_SWITCH_H */
