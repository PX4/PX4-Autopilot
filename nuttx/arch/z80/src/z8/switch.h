/************************************************************************************
 * arch/z80/src/z8/switch.h
 * arch/z80/src/chip/switch.h
 *
 *   Copyright (C) 2008-2009, 2011-2012 Gregory Nutt. All rights reserved.
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

#ifndef __Z80_SWITCH_H
#define __Z80_SWITCH_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#ifndef __ASSEMBLY__
#  include <stdint.h>
#  include <nuttx/sched.h>
#  include <nuttx/arch.h>
#endif
#include "common/up_internal.h"

/************************************************************************************
 * Definitions
 ************************************************************************************/

/* Z8_IRQSTATE_* definitions ********************************************************
 * These are used in the state field of 'struct z8_irqstate_s' structure to define
 * the current state of the interrupt handling.  These definition support "lazy"
 * interrupt context saving. See comments below associated with s'truct z8_irqstate_s'.
 */

#define Z8_IRQSTATE_NONE  0 /* Not handling an interrupt */
#define Z8_IRQSTATE_ENTRY 1 /* In interrupt, context has not been saved */
#define Z8_IRQSTATE_SAVED 2 /* In interrupt, context has been saved */

/* The information saved on interrupt entry can be retained in a array of two
 * uint24_t values.  These are
 *
 *   value[0] = RP (MS byte) and Flags (LS) byte
 *   value[1] = PC
 *
 * The pointer to the save structure is a stack pointer at the time that up_doirq()
 * was called:
 *
 *         PC[7:0]
 *         PC[15:8]
 *         Flags Register
 *   SP -> RP
 *
 * The stack pointer on return from interrupt can be obtained by adding 4 to the
 * pointer to the save structure.
 */

#define Z8_IRQSAVE_RPFLAGS    (0)                      /* Index 10: RP (MS) and FLAGS (LS) */
#define Z8_IRQSAVE_PC         (1)                      /* Index 2: PC[8:15] */
#define Z8_IRQSAVE_REGS       (2)                      /* Number 16-bit values saved */

/* Byte offsets */

#define Z8_IRQSAVE_RP_OFFS    (2*Z8_IRQSAVE_RPFLAGS)   /* Offset 0: RP */
#define Z8_IRQSAVE_FLAGS_OFFS (2*Z8_IRQSAVE_RPFLAGS+1) /* Offset 1: FLAGS */
#define Z8_IRQSAVE_PCH_OFFS   (2*Z8_IRQSAVE_PC)        /* Offset 2: PC[8:15] */
#define Z8_IRQSAVE_PCL_OFFS   (2*Z8_IRQSAVE_PC+1)      /* Offset 3: PC[0:7] */
#define Z8_IRQSAVE_SIZE       (2*Z8_IRQSAVE_REGS)      /* Number 8-bit values saved */

/* Macros for portability ***********************************************************
 *
 * Common logic in arch/z80/src/common is customized for the z8 context switching
 * logic via the following macros.
 */

/* Initialize the IRQ state */

#define INIT_IRQCONTEXT() \
  do { \
    g_z8irqstate.state = Z8_IRQSTATE_NONE; \
  } while (0)

/* IN_INTERRUPT returns true if the system is currently operating in the interrupt
 * context.  IN_INTERRUPT is the inline equivalent of up_interrupt_context().
 */

#define IN_INTERRUPT() \
  (g_z8irqstate.state != Z8_IRQSTATE_NONE)

/* The following macro is used when the system enters interrupt handling logic
 *
 * NOTE: Nested interrupts are not supported in this implementation.  If you want
 * to implement nested interrupts, you would have to change the way that
 * current_regs is handled.  The savestate variable would not work for
 * that purpose as implemented here because only the outermost nested
 * interrupt can result in a context switch (it can probably be deleted).
 */

#define DECL_SAVESTATE() \
  struct z8_irqstate_s savestate

#define IRQ_ENTER(irq, regs) \
  do { \
    savestate.state    = g_z8irqstate.state; \
    savestate.regs     = g_z8irqstate.regs; \
    g_z8irqstate.state = Z8_IRQSTATE_ENTRY; \
    g_z8irqstate.regs  = (regs); \
    up_maskack_irq(irq); \
  } while (0)

/* The following macro is used when the system exits interrupt handling logic */

#define IRQ_LEAVE(irq) \
  do { \
    g_z8irqstate.state = savestate.state; \
    g_z8irqstate.regs  = savestate.regs; \
    up_enable_irq(irq); \
  } while (0)

/* The following macro is used to sample the interrupt state (as a opaque handle) */

#define IRQ_STATE() \
  (g_z8irqstate.regs)

/* Save the current IRQ context in the specified TCB */

#define SAVE_IRQCONTEXT(tcb) \
  z8_saveirqcontext((tcb)->xcp.regs)

/* Set the current IRQ context to the state specified in the TCB */

#define SET_IRQCONTEXT(tcb) \
  do { \
    g_z8irqstate.state = Z8_IRQSTATE_SAVED; \
    g_z8irqstate.regs  = (tcb)->xcp.regs; \
  } while (0)

/* Save the user context in the specified TCB.  User context saves can be simpler
 * because only those registers normally saved in a C called need be stored.
 */

#define SAVE_USERCONTEXT(tcb) \
  z8_saveusercontext((tcb)->xcp.regs)

/* Restore the full context -- either a simple user state save or the full,
 * IRQ state save.
 */

#define RESTORE_USERCONTEXT(tcb) \
  z8_restorecontext((tcb)->xcp.regs)

/* Dump the current machine registers */

#define _REGISTER_DUMP() \
  z8_registerdump()

/************************************************************************************
 * Public Types
 ************************************************************************************/

/* In order to provide faster interrupt handling, the interrupt logic does "lazy"
 * context saving as described below:
 *
 * (1) At the time of the interrupt, minimum information is saved and the register
 *     pointer is changed so that the interrupt logic does not alter the state of
 *     the interrupted task's registers.
 * (2) If no context switch occurs during the interrupt processing, then the return
 *     from interrupt is also simple.
 * (3) If a context switch occurs during interrupt processing, then
 *     (a) The full context of the interrupt task is saved, and
 *     (b) A full context switch is performed when the interrupt exits (see
 *         z8_vector.S).
 *
 * The following structure is used to manage this "lazy" context saving.
 */

#ifndef __ASSEMBLY__
struct z8_irqstate_s
{
  uint8_t    state; /* See Z8_IRQSTATE_* definitions above */
  chipreg_t *regs;  /* Saved register information */
};
#endif

/************************************************************************************
 * Public Variables
 ************************************************************************************/

#ifndef __ASSEMBLY__
/* This structure holds information about the current interrupt processing state */

extern struct z8_irqstate_s g_z8irqstate;
#endif

/************************************************************************************
 * Public Function Prototypes
 ************************************************************************************/

#ifndef __ASSEMBLY__
#ifdef __cplusplus
extern "C"
{
#endif

/* Defined in z8_irq.c */

void up_maskack_irq(int irq);

/* Defined in z8_saveusercontext.asm */

int z8_saveusercontext(FAR chipreg_t *regs);

/* Defined in z8_saveirqcontext.c */

void z8_saveirqcontext(FAR chipreg_t *regs);

/* Defined in z8_restorecontext.asm */

void z8_restorecontext(FAR chipreg_t *regs);

/* Defined in z8_sigsetup.c */

void z8_sigsetup(FAR _TCB *tcb, sig_deliver_t sigdeliver, FAR chipreg_t *regs);

/* Defined in z8_registerdump.c */

void z8_registerdump(void);

#ifdef __cplusplus
}
#endif
#endif

#endif  /* __Z80_SWITCH_H */
